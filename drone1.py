import RPi.GPIO as GPIO
import time

# Define GPIO pins for motors
motor_pins = [17, 18, 27, 22]

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

# Arm ESCs
print("Arming ESCs...")
for m in motors:
    m.ChangeDutyCycle(us_to_duty(1000))  # 1000 µs idle
time.sleep(5)

# Spin motors
print("Spinning motors...")
for m in motors:
    m.ChangeDutyCycle(us_to_duty(1200))  # Low throttle
time.sleep(5)

# Stop motors
print("Stopping motors...")
for m in motors:
    m.ChangeDutyCycle(us_to_duty(1000))
time.sleep(2)

# Cleanup
for m in motors:
    m.stop()
GPIO.cleanup()
