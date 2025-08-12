import RPi.GPIO as GPIO
import time
import math

# === CONFIGURATION ===
PWM_FREQ = 50
MIN_DUTY_US = 1000
MAX_DUTY_US = 2000
MIN_DUTY_US = 1000
MAX_DUTY_US = 2000

# Define GPIO pins for motors
motor_pins = [17, 18, 27, 22]

# Define input vector
input_v = [0, 0, 1]

GPIO.setmode(GPIO.BCM)

# Setup and initialize motors
motors = []
for pin in motor_pins:
    GPIO.setup(pin, GPIO.OUT)
    pwm = GPIO.PWM(pin, 50)  # 50 Hz for ESCs
    pwm.start(5)             # Idle duty cycle
    motors.append(pwm)

# Helper: Convert microseconds to % duty cycle
def us_to_duty(us):
    return us / 20000 * 100

def power_to_us(power):
    power = max(0.0, min(1.0, power))
    return MIN_DUTY_US + power * (MAX_DUTY_US - MIN_DUTY_US)

while True:
    x, y, z = tuple(input_v)

    # Clamp x and y to [-1, 1]
    x = max(-1, min(1, x))
    y = max(-1, min(1, y))
    z = max(-1, min(1, z))  # Optional, if z can go out of bounds too

    # Calculate raw powers
    powers = [
        z + y + x,  # M1 - Front Left
        z + y - x,  # M2 - Front Right
        z - y + x,  # M3 - Back Left
        z - y - x   # M4 - Back Right
    ]

    # Calculate magnitude of input vector
    mag = math.sqrt(x**2 + y**2 + z**2)

    # Normalize if magnitude > 1
    if mag > 1:
        powers = [p / mag for p in powers]

    # Send to motors
    for p, m in zip(powers, motors):
        m.ChangeDutyCycle(power_to_us(p * 100.00))
  ]

  for p, m in zip(powers, motors):
    m.ChangeDutyCycle(power_to_us(p * 100.00))

GPIO.cleanup()
