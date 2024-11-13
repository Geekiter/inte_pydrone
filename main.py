import time

import ble_simple_peripheral
import bluetooth
import drone
from machine import Pin, PWM

# Initialize the drone
d = drone.DRONE(flightmode=0, debug=1)
"""
while True:
 
 #打印校准信息，当返回 3 个值均少于 5000 时校准通过。
 print(d.read_cal_data())
 
 #校准通过
 if d.read_calibrated():
 
 print(d.read_cal_data())
 
 break
 
 time.sleep_ms(100)
"""
while True:
    print(d.read_cal_data())
    if d.read_calibrated():
        print(d.read_cal_data())
        break
    time.sleep_ms(1000)

# Initialize motors with GPIO pins
M1 = PWM(Pin(4), freq=10000, duty=0)
M2 = PWM(Pin(5), freq=10000, duty=0)
M3 = PWM(Pin(40), freq=10000, duty=0)
M4 = PWM(Pin(41), freq=10000, duty=0)

# Initialize Bluetooth
ble = bluetooth.BLE()
p = ble_simple_peripheral.BLESimplePeripheral(ble, name='pyDrone')


# Cap the duty cycle to 80%
def cap_duty_cycle(value):
    return min(int((value / 100) * 1023), int(0.8 * 1023))


# Function to process control data for motors M2 and M3 (Handle A)
def control_motors_m2_m3(control_data):
    y_val = control_data[1]
    x_val = control_data[0]

    if y_val > 0:
        duty_m2 = cap_duty_cycle(y_val)
        duty_m3 = cap_duty_cycle(y_val)
        print(f"Before M2 duty cycle: {duty_m2} / 1023, M3 duty cycle: {duty_m3} / 1023")
        if x_val == 100:
            M2.duty(0)
            M3.duty(duty_m3)
        elif x_val == -100:
            M2.duty(duty_m2)
            M3.duty(0)
        else:
            proportion = (x_val + 100) / 200  # Map x to range between 0 and 1
            duty_m2b = int(duty_m2 * (1 - proportion))
            duty_m3b = int(duty_m3 * proportion)
            print(f"calculation M2 duty cycle: {duty_m2b} / 1023, M3 duty cycle: {duty_m3b} / 1023")
            M2.duty(duty_m2b)
            M3.duty(duty_m3b)
    else:
        M2.duty(0)
        M3.duty(0)

    # Print the current duty cycles for M2 and M3
    print(f"M2 duty cycle: {M2.duty()} / 1023, M3 duty cycle: {M3.duty()} / 1023")


# Function to process control data for motors M1 and M4 (Handle B)
def control_motors_m1_m4(control_data):
    y_val = control_data[3]
    x_val = control_data[2]

    if y_val > 0:
        duty_m1 = cap_duty_cycle(y_val)
        duty_m4 = cap_duty_cycle(y_val)

        if x_val == 100:
            M1.duty(0)
            M4.duty(duty_m4)
        elif x_val == -100:
            M1.duty(duty_m1)
            M4.duty(0)
        else:
            proportion = (x_val + 100) / 200  # Map x to range between 0 and 1
            M1.duty(int(duty_m1 * (1 - proportion)))
            M4.duty(int(duty_m4 * proportion))
    else:
        M1.duty(0)
        M4.duty(0)

    # Print the current duty cycles for M1 and M4
    print(f"M1 duty cycle: {M1.duty()} / 1023, M4 duty cycle: {M4.duty()} / 1023")


# Bluetooth callback function
def on_rx(text):
    control_data = [None] * 4

    # Process control data
    for i in range(4):
        if 100 < text[i + 1] < 155:
            control_data[i] = 0
        elif text[i + 1] <= 100:
            control_data[i] = text[i + 1] - 100
        else:
            control_data[i] = text[i + 1] - 155

    print('control:', control_data)

    # Control M2 and M3 (Handle A)
    control_motors_m2_m3(control_data)

    # Control M1 and M4 (Handle B)
    control_motors_m1_m4(control_data)

    # Detect button press
    if text[5] == 24:  # Y button pressed
        print('Y')
        # Motor beep can be added here
    elif text[5] == 136:  # X button pressed, stop all motors
        print('X')
        M1.duty(0)
        M2.duty(0)
        M3.duty(0)
        M4.duty(0)
        d.stop()

    # Read drone state
    states = d.read_states()
    print('states:', states)
    state_buf = [None] * 18
    for i in range(9):
        for j in range(2):
            if j == 0:
                state_buf[i * 2 + j] = int((states[i] + 32768) / 256)
            else:
                state_buf[i * 2 + j] = int((states[i] + 32768) % 256)

    p.send(bytes(state_buf))  # Send state back via Bluetooth


high = 0


def on_rx2(text):
    control_data2 = [None] * 4

    # Process control data
    for i in range(4):
        if 100 < text[i + 1] < 155:
            control_data2[i] = 0
        elif text[i + 1] <= 100:
            control_data2[i] = text[i + 1] - 100
        else:
            control_data2[i] = text[i + 1] - 155

    print('control:', control_data2)
    global high
    # Read drone state
    states = d.read_states()
    print('states:', states)
    control_data = [0, 0, 0, 0]
    roll, pitch, yaw, jc_roll, jc_pitch, jc_yaw, jc_thrust, battery, cur_high = states

    angle = 60
    current_angle = yaw

    if current_angle - 10 < angle < current_angle + 10:
        angle_status = "keep"
    # 移动角度
    elif angle < current_angle:
        # 左转
        control_motors_m1_m4([0, 0, 100, 100])
    elif angle > current_angle:
        # 右转
        control_motors_m1_m4([0, 0, -100, 100])

    if high != cur_high:
        high = cur_high
    else:
        high = 0

    print('high:', high)
    if high == 0:
        control_data[1] = 0
        control_motors_m2_m3(control_data)
        # Control M1 and M4 (Handle B)
        control_motors_m1_m4(control_data)
    elif high <= 50:
        control_data[1] = 80
        for _ in range(10):
            control_motors_m2_m3(control_data)
            # Control M1 and M4 (Handle B)
            control_motors_m1_m4(control_data)
    elif 50 < high <= 100:
        control_data[1] = 60
        control_motors_m2_m3(control_data)
        # Control M1 and M4 (Handle B)
        control_motors_m1_m4(control_data)

    #

    states = d.read_states()
    print('states:', states)
    state_buf = [None] * 18
    for i in range(9):
        for j in range(2):
            if j == 0:
                state_buf[i * 2 + j] = int((states[i] + 32768) / 256)
            else:
                state_buf[i * 2 + j] = int((states[i] + 32768) % 256)

    p.send(bytes(state_buf))  # Send state back via Bluetooth


# Register the Bluetooth callback function
p.on_write(on_rx2)
# Main loop (optional for additional logic)
