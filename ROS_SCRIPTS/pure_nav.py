import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import time
import random
from enum import Enum
from collections import deque

import smbus2
try:
    import RPi.GPIO as GPIO
except ImportError:
    # Graceful degradation for off-target testing
    GPIO = None


# ========================= #
# CONFIGURATION & CONSTANTS #
# ========================= #

class Config:
    LED_GPIO = 17
    # Hardware & Physical Dimensions
    ROBOT_RADIUS = 0.11
    LIDAR_BUFFER_MIN = 0.055
    LIDAR_BUFFER_MAX = 0.23  # Dropped from 0.35. Don't brake until 23cm away.
    
    # Safety Thresholds
    COLLISION_THRESHOLD = 0.105
    MIN_SAFE_DIST = ROBOT_RADIUS + LIDAR_BUFFER_MIN  
    MAX_SAFE_DIST = ROBOT_RADIUS + LIDAR_BUFFER_MAX  
    
    # Kinematics
    MAX_LINEAR_SPEED = 0.20  
    MIN_LINEAR_SPEED = 0.10  # Raised from 0.05  no more creeping
    MAX_ANGULAR_SPEED = 1.2  
    REVERSE_SPEED = -0.08    
    
    # Smoothing Factors
    ANGULAR_SMOOTH_OLD = 0.65  
    ANGULAR_SMOOTH_NEW = 0.35  
    TURN_PENALTY_FACTOR = 0.9  # dopped from 1.5 let it turn faster
    
    # mission
    MISSION_TIME_LIMIT = 120.0 

    # anti-Stuck & memory parameters
    STATE_MEMORY_SIZE = 60 
    OSCILLATION_THRESHOLD = 0.85 
    RECOVERY_DURATION = 1.5

# states for the movement
class NavState(Enum):
    INIT = "INIT"
    FORWARD = "FORWARD"
    TURN_LEFT = "TURN_LEFT"
    TURN_RIGHT = "TURN_RIGHT"
    REVERSE = "REVERSE"
    RECOVERY = "RECOVERY"
    STOPPED = "STOPPED"


# class for all the cone areas to easily control logic regarding movement
class ConeData:
    def __init__(self, name, min_dist):
        self.name = name
        self.min_dist = min_dist
        self.is_blocked = min_dist < Config.MIN_SAFE_DIST


# ================= #
# SENSOR PROCESSING #
# ================= #

class NavigationMemory:
    def __init__(self, size):
        self.history = deque(maxlen=size)
        self.in_recovery = False
        self.recovery_end_time = 0.0

    def add_state(self, state):
        self.history.append(state)

    # the occasional oscillations gave us a head ache 
    # so this track if an oscillation occurs
    def detect_oscillation(self):
        if len(self.history) < self.history.maxlen: return False
        turn_count = sum(1 for s in self.history if s in [NavState.TURN_LEFT, NavState.TURN_RIGHT])
        forward_count = sum(1 for s in self.history if s == NavState.FORWARD)
        if (turn_count / self.history.maxlen) >= 0.8 and forward_count < (self.history.maxlen * 0.1):
            return True
        return False

class LidarProcessor:
    def __init__(self):
        self.cones = {}
        self.min_global_distance = float('inf')

    def process_scan(self, scan_msg):
        ranges = np.array(scan_msg.ranges, dtype=np.float32)
        # filter invalid ranges set to max distance 
        ranges[np.isinf(ranges)] = 3.5
        ranges[np.isnan(ranges)] = 3.5
        ranges[ranges < 0.01] = 3.5  
        
        self.min_global_distance = float(np.min(ranges))

        # Front is now +/-15 degrees instead of +/-30 degrees
        # front is of crouse the first 15 degrees and the last 15 in the lidar spectrum
        front_slice = np.concatenate((ranges[0:16], ranges[345:360]))
        self.cones['F']  = ConeData('Front', float(np.min(front_slice))) 
        
        # cones to match the new narrow front
        self.cones['FL'] = ConeData('Front-Left', float(np.min(ranges[15:45])))
        self.cones['L']  = ConeData('Left', float(np.min(ranges[45:90])))
        self.cones['FR'] = ConeData('Front-Right', float(np.min(ranges[315:345])))
        self.cones['R']  = ConeData('Right', float(np.min(ranges[270:315])))

    def get_cone(self, cone_key):
        return self.cones.get(cone_key, ConeData(cone_key, 3.5))


# ============================== #
# VICTIM PERCEPTION & INDICATION #
# ============================== #

class VictimState(Enum):
    SEARCHING = "SEARCHING"
    COOLDOWN = "COOLDOWN"

class ISL29125Driver:
    I2C_BUS = 1
    ADDRESS = 0x44
    REG_CONFIG_1 = 0x01
    REG_DATA_START = 0x09 

    def __init__(self):
        self.bus = smbus2.SMBus(self.I2C_BUS)
        self._initialize_sensor()

    def _initialize_sensor(self):
        # 0x0D = 10,000 lux Range, 16-bit, RGB Mode
        try:
            self.bus.write_byte_data(self.ADDRESS, self.REG_CONFIG_1, 0x0D)
            time.sleep(0.1)
        except Exception:
            pass

    def read_rgb(self):
        try:
            # Burst read 6 bytes: G_L, G_H, R_L, R_H, B_L, B_H
            data = self.bus.read_i2c_block_data(self.ADDRESS, self.REG_DATA_START, 6)
            green = data[0] | (data[1] << 8)
            red   = data[2] | (data[3] << 8)
            blue  = data[4] | (data[5] << 8)
            return red, green, blue
        except Exception:
            return 0, 0, 0

class NonBlockingLED:
    def __init__(self, pin=17):
        self.pin = pin
        self.is_blinking = False
        self.blink_end_time = 0.0
        self.toggle_interval = 0.15
        self.next_toggle_time = 0.0
        self.state = False
        if GPIO:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.pin, GPIO.OUT)
            GPIO.output(self.pin, GPIO.LOW)

    def trigger_blink(self, duration=1.0):
        self.is_blinking = True
        self.blink_end_time = time.time() + duration
        self.next_toggle_time = time.time()

    # control the blinking for indicating victim found
    def tick(self, current_time):
        if not self.is_blinking: return
        if current_time >= self.blink_end_time:
            self.is_blinking = False
            if GPIO: GPIO.output(self.pin, GPIO.LOW)
            return
        if current_time >= self.next_toggle_time:
            self.state = not self.state
            if GPIO: GPIO.output(self.pin, GPIO.HIGH if self.state else GPIO.LOW)
            self.next_toggle_time = current_time + self.toggle_interval

class VictimTracker:
    def __init__(self, logger):
        self.logger = logger
        self.sensor = ISL29125Driver()
        self.led = NonBlockingLED(pin=Config.LED_GPIO)
        
        self.state = VictimState.SEARCHING
        self.victim_counter = 0
        
        # calibrated heuristics
        self.MIN_INTENSITY = 250   
        self.RED_RATIO = 1.15      
        
        # blind timer for not keep counting the same victim
        self.cooldown_duration = 3.0 # goes blind for 3 seconds after finding red
        self.cooldown_end_time = 0.0

    def tick(self, current_time):
        self.led.tick(current_time)
        
        if self.state == VictimState.COOLDOWN:
            if current_time >= self.cooldown_end_time:
                self.state = VictimState.SEARCHING
                self.logger.info("Cooldown over. Ready for next red trigger.")
            return

        if self.state == VictimState.SEARCHING:
            r, g, b = self.sensor.read_rgb()
            
            # If we see red  we count it immediately and go to sleep
            # this is done by checking if red is 15% greater than the green to filter white lux
            # this has been tweaked for some time and seems like the sweet spot
            if r > self.MIN_INTENSITY and r > (g * self.RED_RATIO):
                self.victim_counter += 1
                
                self.state = VictimState.COOLDOWN
                self.cooldown_end_time = current_time + self.cooldown_duration
                self.led.trigger_blink(duration=1.5)
                self.logger.info(f"RED DETECTED! Total Count: {self.victim_counter} (R:{r} G:{g})")
# ==================== # 
# MAIN ROS2 NODE CLASS #
# ==================== #
# inherits Node from rclpy.node meaning we get the ROS2 stuff
class SmartNavNode(Node):
    def __init__(self):
        super().__init__('smart_nav_node')
        self.get_logger().info("Init TurtleBot3 pure reactive lidar explorer...")

        self.collision_counter = 0
        self.speed_samples = []
        self.is_in_collision = False

        # istantiate the perception subsystem
        self.tracker = VictimTracker(self.get_logger())

        # navigation Components
        self.memory = NavigationMemory(Config.STATE_MEMORY_SIZE)
        self.lidar = LidarProcessor()
        
        # Telemetry Initializers
        self.current_state = NavState.INIT
        self.prev_ang_z = 0.0
        self.start_time = time.time()

        # bias logic
        self.random_bias = 0.0
        self.last_bias_update = time.time()
        
        # Perception Timer (20Hz) every 0.05 second
        self.create_timer(0.05, self.perception_loop)

        # Lidar Subcribe where we call lidar_callback everytime the lidar communicates new data
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos_profile_sensor_data)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10) # publish messages to the twist node via cmd_vel
        
        self.get_logger().info("Ready. Awaiting /scan data...")

    # Here we decide the state for what the robot needs to do
    def evaluate_escape_logic(self):
        # check the cones states
        F = self.lidar.get_cone('F').is_blocked
        FL = self.lidar.get_cone('FL').is_blocked
        FR = self.lidar.get_cone('FR').is_blocked

        # only if front is blocked we react with some critical turns
        if F:
            if FL and FR: return NavState.REVERSE
            if FR and not FL: return NavState.TURN_LEFT
            if FL and not FR: return NavState.TURN_RIGHT
            
            # If the front is blocked, turn toward the side with the most space
            dist_left = self.lidar.get_cone('L').min_dist + self.lidar.get_cone('FL').min_dist
            dist_right = self.lidar.get_cone('R').min_dist + self.lidar.get_cone('FR').min_dist
            return NavState.TURN_LEFT if dist_left > dist_right else NavState.TURN_RIGHT

        # FULL SPEEEED FORWARD CAPTAIN   
        return NavState.FORWARD

    def perception_loop(self):
        """High-frequency callback for I2C polling and LED state management."""
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.tracker.tick(current_time)

    def lidar_callback(self, msg):
        current_time = time.time()
        
        # first step is to check if time is up!
        if current_time - self.start_time >= Config.MISSION_TIME_LIMIT:
            self.publish_twist(0.0, 0.0, True)
            self.shutdown_sequence()
            return

        # if we are still runnings we get the lidar data
        self.lidar.process_scan(msg)

        # Log collision if the absolute minimum distance is below safety threshold
        # if self.lidar.min_global_distance < Config.COLLISION_THRESHOLD:
        #     self.collision_counter += 1
        if self.lidar.min_global_distance < Config.COLLISION_THRESHOLD:
            if not self.is_in_collision:
                self.collision_counter += 1
                self.is_in_collision = True
                self.get_logger().warn(f"Collision! Total count: {self.collision_counter}")
        elif self.lidar.min_global_distance > (Config.COLLISION_THRESHOLD + 0.02):
            self.is_in_collision = False

        # Anti-Stuck & Recovery
        target_lin_x, target_ang_z = 0.0, 0.0

        # we then check if we are in recovery and react by doing the recover sequence 
        if self.memory.in_recovery:
            if current_time >= self.memory.recovery_end_time:
                self.memory.in_recovery = False
                self.memory.history.clear()
            else:
                # Config.REVERSE_SPEED
                self.publish_twist(Config.REVERSE_SPEED, self.prev_ang_z, override_smoothing=True)
                return

        # I hate oscillations so this is checked and excuted if the situation occurs
        # we have not had enough time to improve this as it served as a small side quest
        if self.memory.detect_oscillation():
            self.memory.in_recovery = True
            self.memory.recovery_end_time = current_time + Config.RECOVERY_DURATION
            self.current_state = NavState.RECOVERY
            rand_turn = random.choice([-1.0, 1.0]) * Config.MAX_ANGULAR_SPEED
            # we use Config.REVERSE_SPEED instead of hardcoded -0.05 from before (small improvement)
            self.publish_twist(Config.REVERSE_SPEED, rand_turn, override_smoothing=True)
            return

        # now we covered most critical cases we can continue to state evaluation
        self.current_state = self.evaluate_escape_logic()
        self.memory.add_state(self.current_state)
        # used to check how sharp we need to turn (linear speed optimization)
        front_dist = self.lidar.get_cone('F').min_dist

        # ======================== #
        # PURE REACTIVE KINEMATICS #
        # ======================== #
        if self.current_state == NavState.FORWARD:
            # Updates jitter evry 2 second
            if current_time - self.last_bias_update > 2.0:
                self.random_bias = random.uniform(-0.4, 0.4) 
                self.last_bias_update = current_time

            # get all the cone date
            fl_dist = self.lidar.get_cone('FL').min_dist
            fr_dist = self.lidar.get_cone('FR').min_dist
            l_dist = self.lidar.get_cone('L').min_dist  
            r_dist = self.lidar.get_cone('R').min_dist  
            
            # calculate which openspace has most area
            left_openness = (fl_dist * 0.7) + (l_dist * 0.3)
            right_openness = (fr_dist * 0.7) + (r_dist * 0.3)
            
            # basis turning with some random bias jitter (randomness to avoid moving in the same path)
            target_ang_z = ((left_openness - right_openness) * 0.6) + self.random_bias

            # WHEEL PROTECTION
            # a bit challenging to tweak so the wheel are not cut in a corner due to random_bias
            # the random jitter ensures it doesn't move in a predictable pattern so this is a constraint!
            WHEEL_CLEARANCE = 0.15 # value needs tweaking!
            
            # logic for turning due to the wheel_clearance threshold
            if l_dist < WHEEL_CLEARANCE:
                # turn right
                target_ang_z = -1.2 
            elif r_dist < WHEEL_CLEARANCE:
                # turn left
                target_ang_z = 1.2

            # dynamic velocity 
            # calculates velocity based on the front distance (linear interpolation)
            ratio = (front_dist - Config.MIN_SAFE_DIST) / (Config.MAX_SAFE_DIST - Config.MIN_SAFE_DIST)
            # we take the ratio with the max_linea_speed and see if it is indeed above the min_linear_speed
            # this is to ensure we don't end up driving slower than the actual minimum we set (this is tweaked for best performance)
            base_speed = max(Config.MAX_LINEAR_SPEED * ratio, Config.MIN_LINEAR_SPEED)
            
            # brake logic: if doing a sharp turun
            # we will need to reduce the linear soeed drastically
            if abs(target_ang_z) > 0.8:
                # brakes to min speed set in our config
                target_lin_x = Config.MIN_LINEAR_SPEED 
            else:
                # otherwise drive with normal speed
                target_lin_x = base_speed

            # clamp angular speed to max
            target_ang_z = np.clip(target_ang_z, -Config.MAX_ANGULAR_SPEED, Config.MAX_ANGULAR_SPEED)
            
        # Factors is tweaked so the linear speed is maintained appropiately when turning
        elif self.current_state == NavState.TURN_LEFT:
            target_lin_x = 0.0 if front_dist < (Config.MIN_SAFE_DIST + 0.08) else (Config.MAX_LINEAR_SPEED * 0.5) # 0.3->0.5
            target_ang_z = Config.MAX_ANGULAR_SPEED
            
        elif self.current_state == NavState.TURN_RIGHT:
            target_lin_x = 0.0 if front_dist < (Config.MIN_SAFE_DIST + 0.08) else (Config.MAX_LINEAR_SPEED * 0.5) # 0.3->0.5
            target_ang_z = -Config.MAX_ANGULAR_SPEED
            
        elif self.current_state == NavState.REVERSE:
            target_lin_x = Config.REVERSE_SPEED
            target_ang_z = 0.0

        # Apply filter and publish the speeed
        self.publish_twist(target_lin_x, target_ang_z)


    def publish_twist(self, target_lin_x, target_ang_z, override_smoothing=False):
        cmd = Twist()
        # smoothing is mainly used on our edge cases not under normal conditions
        if override_smoothing:
            cmd.angular.z = float(target_ang_z)
            cmd.linear.x = float(target_lin_x)
        else:
            # 
            cmd.angular.z = (Config.ANGULAR_SMOOTH_OLD * self.prev_ang_z) + (Config.ANGULAR_SMOOTH_NEW * target_ang_z)
            
            # calculate turn penalty (drop speed if turning sharply)
            turn_penalty = 1.0 - Config.TURN_PENALTY_FACTOR * (abs(cmd.angular.z) / Config.MAX_ANGULAR_SPEED)
            
            # Dropped floor from 0.6 to 0.4 so it actually brakes for corners
            calculated_speed = target_lin_x * max(turn_penalty, 0.4)
            
            # STRICT CLAMP: ensure we never exceed max speed
            cmd.linear.x = float(np.clip(calculated_speed, Config.REVERSE_SPEED, Config.MAX_LINEAR_SPEED))

        #record the ang speed 
        self.prev_ang_z = cmd.angular.z
        self.cmd_vel_pub.publish(cmd)

        # record only valid forward/reverse velocities for the average 
        # to find average velocity
        self.speed_samples.append(abs(cmd.linear.x))

    def shutdown_sequence(self):
        """called when MISSION_TIME_LIMIT is reached."""
        # calculate average speed
        avg_speed = sum(self.speed_samples) / len(self.speed_samples) if self.speed_samples else 0.0
        
        # Display the stats for the mission 
        print("\n--- FINAL MISSION REPORT ---")
        print(f"Total Red Triggers Counted: {self.tracker.victim_counter}")
        print(f"average_speed_counter: {avg_speed:.3f} m/s")
        print(f"collision_counter: {self.collision_counter}")
        print("----------------------------")
        print(self.speed_samples)
        # stop the robot
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args) # init thr ROS2 client library middleware connection
    try:
        node = SmartNavNode() #create the brain of our robot
        rclpy.spin(node) # ensures that everytime we recieve data our logic is runned
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.cmd_vel_pub.publish(Twist()) # sends stop message
            node.destroy_node() # free the node memory 
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()