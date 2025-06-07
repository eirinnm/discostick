import pwmio
import board
import time
import math
import busio
import digitalio
from adafruit_lsm6ds.lsm6ds3 import LSM6DS3

# Set up PWM for the 3-pin RGB LED
led_red = pwmio.PWMOut(board.LED_RED, frequency=5000, duty_cycle=0)
led_green = pwmio.PWMOut(board.LED_GREEN, frequency=5000, duty_cycle=0)
led_blue = pwmio.PWMOut(board.LED_BLUE, frequency=5000, duty_cycle=0)

def set_led_color(red, green, blue):
    # Convert 0.0-1.0 RGB values to 16-bit duty cycle values.
    led_red.duty_cycle = int(red * 65535)
    led_green.duty_cycle = int(green * 65535)
    led_blue.duty_cycle = int(blue * 65535)

def hue_to_rgb(hue):
    # Convert hue to RGB values in the range 0.0-1.0.
    if hue < 0 or hue > 360:
        raise ValueError("Hue must be between 0 and 360 degrees")
    
    c = 1.0
    x = c * (1 - abs((hue / 60) % 2 - 1))
    m = 0.0
    
    if hue < 60:
        r, g, b = c, x, 0
    elif hue < 120:
        r, g, b = x, c, 0
    elif hue < 180:
        r, g, b = 0, c, x
    elif hue < 240:
        r, g, b = 0, x, c
    elif hue < 300:
        r, g, b = x, 0, c
    else:
        r, g, b = c, 0, x
    
    return (r + m), (g + m), (b + m)

def set_led_hue(hue):
    # Compute the PWM values for the hue and set the LED color.
    r, g, b = hue_to_rgb(hue)
    set_led_color(r, g, b)

class IMU(LSM6DS3):
    def __init__(self):
        dpwr = digitalio.DigitalInOut(board.IMU_PWR)
        dpwr.direction = digitalio.Direction.OUTPUT
        dpwr.value = 1
        time.sleep(1)
        i2c = busio.I2C(board.IMU_SCL, board.IMU_SDA)
        super().__init__(i2c)


def compute_pitch_roll(acceleration, gyro, dt, previous_pitch=0.0, previous_roll=0.0, alpha=0.98):
    # Unpack sensor values
    ax, ay, az = acceleration
    gx, gy, _ = gyro  # Ignore the z-axis for pitch and roll
    
    # Calculate pitch and roll from the accelerometer in degrees
    pitch_acc = math.degrees(math.atan2(ax, math.sqrt(ay * ay + az * az)))
    roll_acc = math.degrees(math.atan2(ay, math.sqrt(ax * ax + az * az)))
    
    # Integrate the gyroscope data to estimate angles
    pitch_gyro = previous_pitch + gx * dt
    roll_gyro = previous_roll + gy * dt
    
    # Combine the two estimates with a complementary filter
    pitch = alpha * pitch_gyro + (1 - alpha) * pitch_acc
    roll = alpha * roll_gyro + (1 - alpha) * roll_acc
    
    return pitch, roll


if __name__ == "__main__":
    imu = IMU()
    last_time = time.time()
    pitch, roll = 0.0, 0.0
    while True:
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        
        # Read accelerometer and gyroscope values from the IMU
        acceleration = imu.acceleration  # Assuming this returns (ax, ay, az)
        gyro = imu.gyro  # Assuming this returns (gx, gy, gz)
        
        pitch, roll = compute_pitch_roll(acceleration, gyro, dt, pitch, roll, alpha=0.9)
        
        print("Pitch: %.2f°, Roll: %.2f°" % (pitch, roll))
        # Map pitch to hue for LED color
        hue = (pitch + 180) % 360
        set_led_hue(hue)
        time.sleep(0.01)