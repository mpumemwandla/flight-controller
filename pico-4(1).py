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

def read_sensor():
    # Hardcoded sensor readings for now
    return {"x": 0.0, "y": 0.0, "z": 1.0} 

def set_motor_power(motor_index, power):
    us = power_to_us(power)
    motors[motor_index].duty_u16(us_to_duty(us, PWM_FREQ))

def stabilize_drone():
    global vec
    sensor_data = read_sensor()
    vec["x"] = sensor_data["x"]
    vec["y"] = sensor_data["y"]
    vec["z"] = sensor_data["z"]
    update_motors(vec)

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
    input_command = '{"command": "takeoff"}'  # Hardcoded command for testing
    return json.loads(input_command)

def compute_motor_speeds(command):
    # Hardcoded logic to compute motor speeds based on command
    if command["command"] == "takeoff":
        return [1.0, 1.0, 1.0, 1.0]  # Full throttle for takeoff
    elif command["command"] == "land":
        return [0.0, 0.0, 0.0, 0.0]  # Cut throttle for landing
    else:
        return [0.5, 0.5, 0.5, 0.5]  # Maintain hover

def main():
    print("Starting advanced movement simulation loop...")
    while True:
        command = read_serial_input()
        motor_speeds = compute_motor_speeds(command)
        for i in range(4):
            set_motor_power(i, motor_speeds[i])
        
        stabilize_drone()
        time.sleep(0.1)

# === RUN ===
main()
