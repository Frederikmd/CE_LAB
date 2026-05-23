import time
import RPi.GPIO as GPIO
from gpiozero import LED

GPIO.setmode(GPIO.BCM)

class Dist():
    def __init__(self):
        pass
    
    def Measure(self, gp):
        GPIO_TRIGECHO = gp 
        GPIO.setup(GPIO_TRIGECHO, GPIO.OUT)
        GPIO.output(GPIO_TRIGECHO, False)
        
        # init measurement
        GPIO.output(GPIO_TRIGECHO, True)
        time.sleep(0.00001)
        GPIO.output(GPIO_TRIGECHO, False)
        
        start = time.time()
        stop = time.time() 

        # wait for echo response start
        GPIO.setup(GPIO_TRIGECHO, GPIO.IN)
        while GPIO.input(GPIO_TRIGECHO) == 0:
            start = time.time()

        # wait for echo response end
        while GPIO.input(GPIO_TRIGECHO) == 1:
            stop = time.time()

        GPIO.setup(GPIO_TRIGECHO, GPIO.OUT)
        GPIO.output(GPIO_TRIGECHO, False)

        elapsed = stop - start
        distance = (elapsed * 34300) / 2.0
        time.sleep(0.1)
        return distance


if __name__ == '__main__':
    # pin config
    SONAR_PIN = 18
    LED_PIN = 17
    
    # init
    sonar = Dist()
    led = LED(LED_PIN)
    
    # state variable to track if the LED is already blinking
    is_blinking = False

    print("Program started")

    try:
        while True:
            distance = sonar.Measure(SONAR_PIN)
            print(f"Distance: {distance:.2f} cm")
            
            if distance < 30:
                # idf under 30 cm and not already blinking start blinking
                if not is_blinking:
                    print("Object detected under 30 cm!")
                    led.blink(on_time=0.2, off_time=0.2) 
                    is_blinking = True
            else:
                # If over 30 cm and currently blinking turn it off
                if is_blinking:
                    print("Clear path. Turning LED off.")
                    led.off()
                    is_blinking = False
            
            # Short pause before next measurement to prevent high load
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nStopping program")
        led.off()
        GPIO.cleanup()