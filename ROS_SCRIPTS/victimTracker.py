#!/usr/bin/env python3
import smbus
import time
from gpiozero import LED

# CONFIG
LED_GPIO = 17 # GPIO17 = physical pin 11
I2C_BUS = 1
ADDR = 0x44   # ISL29125 Address

# init LED
led = LED(LED_GPIO)

# init I2C Bus
bus = smbus.SMBus(I2C_BUS)

# Sensor Sensitivity Setup
# 0x0D = 10,000 Lux range (prevents saturation in normal room light)
bus.write_byte_data(ADDR, 0x01, 0x0D)
time.sleep(0.5)

# Thresholds
RED_RATIO_THRESHOLD = 1.3  # Red must be 30% stronger than Green
MIN_RED_INTENSITY = 400    # Ignore noise in very dark settings

# Tracking variables
red_count = 0
is_currently_red = False

print("Tracking Red Marks...")
print("-----------------------------------------")

try:
    while True:
        # read Raw 16-bit RGB values from sensor
        data = bus.read_i2c_block_data(ADDR, 0x09, 6)
        
        green16 = data[0] | (data[1] << 8)
        red16   = data[2] | (data[3] << 8)
        blue16  = data[4] | (data[5] << 8)

        # logic - is the sensor seeing a red object right now?
        see_red_now = (red16 > (green16 * RED_RATIO_THRESHOLD)) and (red16 > MIN_RED_INTENSITY)

        # state Machine Trigger
        if see_red_now and not is_currently_red:
            # RED MARK DETECTED
            red_count += 1
            is_currently_red = True
            
            # Quick LED flash (on for 0.1s then off)
            # n = 1 tells gpiozero to perform exactly one blink cycle
            led.blink(on_time=0.1, off_time=0.1, n=1)
            
            print(f"🔴 MARK #{red_count} FOUND! (Raw R:{red16} G:{green16})")

        elif not see_red_now and is_currently_red:
            # MARK CLEARED
            is_currently_red = False
            print("   (Clear)")

        # Fast polling for accuracy
        time.sleep(0.05)

except KeyboardInterrupt:
    print(f"\nFinal Count: {red_count} red marks.")
finally:
    led.off()
    print("Program Ended Cleanly.")