import RPi.GPIO as GPIO
import time

# === CONFIGURATION ===
PWM_FREQ = 50
MIN_DUTY_US = 1000
MAX_DUTY_US = 2000
MIN_DUTY_US = 1000
MAX_DUTY_US = 2000

# Define GPIO pins for motors
motor_pins = [17, 18, 27, 22]

# Define input vector
input_v = [0, 0, 0]

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
  powers = [
    z + y + x,  # M1 - Front Left
    z + y - x,  # M2 - Front Right
    z - y + x,  # M3 - Back Left
    z - y - x   # M4 - Back Right
  ]

  for p, m in zip(powers, motors):
    m.ChangeDutyCycle(power_to_us(p))

GPIO.cleanup()
