import machine
import time
import json

# === CONFIGURATION ===
PWM_FREQ = 50
PWM_PINS = [15, 14, 13, 12]  # M1-FL, M2-FR, M3-BL, M4-BR
MIN_DUTY_US = 1000
MAX_DUTY_US = 2000

# === GLOBAL VECTOR ===
vec = {"x": 0.0, "y": 0.0, "z": 0.0}
prev_vec = {"x": 0.0, "y": 0.0, "z": 0.0}

# === INIT MOTORS ===
motors = []
for pin in PWM_PINS:
    pwm = machine.PWM(machine.Pin(pin))
    pwm.freq(PWM_FREQ)
    motors.append(pwm)

def us_to_duty(us, freq):
    period_us = 1_000_000 // freq
    return int((us / period_us) * 65535)

def power_to_us(power):
    power = max(0.0, min(1.0, power))
    return MIN_DUTY_US + power * (MAX_DUTY_US - MIN_DUTY_US)

def read_accelerometer():
    # Dummy function mimicking accelerometer readings
    return (0.0, 0.0, 1.0)  # ax, ay, az

def read_gyroscope():
    # Dummy function mimicking gyroscope readings
    return (0.0, 0.0, 0.0)  # gx, gy, gz

def set_motor_power(motor_index, power):
    us = power_to_us(power)
    motors[motor_index].duty_u16(us_to_duty(us, PWM_FREQ))

def stabilize_drone(target_vec):
    global vec, prev_vec
    # Read sensor data
    ax, ay, az = read_accelerometer()
    gx, gy, gz = read_gyroscope()

    # Update current vector using sensor data
    vec["x"] = prev_vec["x"] + ax * 0.1  # Integrate acceleration
    vec["y"] = prev_vec["y"] + ay * 0.1
    vec["z"] = prev_vec["z"] + az * 0.1

    # Compute motor speeds based on target vector and current vector
    power_x = target_vec["x"] - vec["x"]
    power_y = target_vec["y"] - vec["y"]
    power_z = target_vec["z"] - vec["z"]

    update_motors({"x": power_x, "y": power_y, "z": power_z})

    # Store current vector for the next iteration
    prev_vec = vec.copy()

def update_motors(vec):
    x = vec["x"]
    y = vec["y"]
    z = vec["z"]

    powers = [
        z + y + x,  # M1 - Front Left
        z + y - x,  # M2 - Front Right
        z - y + x,  # M3 - Back Left
        z - y - x   # M4 - Back Right
    ]

    max_power = max(abs(p) for p in powers)
    if max_power > 1:
        powers = [p / max_power for p in powers]

    for i in range(4):
        us = power_to_us(powers[i])
        motors[i].duty_u16(us_to_duty(us, PWM_FREQ))

    if int(time.ticks_ms() / 200) % 10 == 0:
        print("vec:", vec, "| powers:", [round(p, 2) for p in powers])

def read_serial_input():
    # Simulating serial input as JSON string
    input_command = '{"command": "move", "x": 0.1, "y": 0.0, "z": 0.5}'  # Example command
    return json.loads(input_command)

def compute_motor_speeds(command):
    target_vec = {"x": 0.0, "y": 0.0, "z": 0.0}
    if command["command"] == "move":
        target_vec["x"] = command.get("x", 0.0)
        target_vec["y"] = command.get("y", 0.0)
        target_vec["z"] = command.get("z", 0.0)
    return target_vec

def main():
    print("Starting advanced movement simulation loop...")
    target_vec = {"x": 0.0, "y": 0.0, "z": 0.0}
    while True:
        command = read_serial_input()
        target_vec = compute_motor_speeds(command)
        
        stabilize_drone(target_vec)
        time.sleep(0.1)

# === RUN ===
main()
