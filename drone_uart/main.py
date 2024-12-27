import json

import ble_simple_peripheral
import bluetooth
from machine import Pin, Timer, PWM, UART

import drone

# 构建四轴对象，无头方式
d = drone.DRONE(flightmode=0, debug=1)
M1 = PWM(Pin(4), freq=10000, duty=0)
M2 = PWM(Pin(5), freq=10000, duty=0)
M3 = PWM(Pin(40), freq=10000, duty=0)
M4 = PWM(Pin(41), freq=10000, duty=0)

uart = UART(1, tx=43, rx=44)
ble = bluetooth.BLE()
p = ble_simple_peripheral.BLESimplePeripheral(ble, name='pyDrone')


def cap_duty_cycle(value):
    return min(int((value / 100) * 1023), int(0.8 * 1023))


def control_motors_m2_m3(control_data):
    y_val = control_data[1]
    x_val = control_data[0]

    duty_m2 = cap_duty_cycle(abs(y_val))
    duty_m3 = cap_duty_cycle(abs(x_val))
    if y_val > 20:
        if x_val > 80:
            left(duty_m2)
        elif x_val < -80:
            right(duty_m2)
        else:
            forward(duty_m2)
    elif y_val < -20:
        backward(duty_m2)
    else:
        reset()


def control_motors_m1_m4(control_data):
    print('control data:', control_data)
    y_val = control_data[3]
    x_val = control_data[2]

    duty_m1 = cap_duty_cycle(abs(y_val))
    duty_m4 = cap_duty_cycle(abs(x_val))
    if y_val > 5:
        up(duty_m1)
    elif y_val < -20:
        reset()


def reset():
    M1.duty(0)
    M2.duty(0)
    M3.duty(0)
    M4.duty(0)


def up(l=50):
    M1.duty(l)
    # M3.duty(l)
    # M2.duty(l)
    M4.duty(l)
    # reset()


def right(l=100):
    M4.duty(l)
    M2.duty(l)
    M1.duty(0)
    M3.duty(0)

    # reset()


def left(l=100):
    M1.duty(l)
    M3.duty(l)
    M2.duty(0)
    M4.duty(0)

    # reset()


def backward(l=100):
    M4.duty(l)
    M1.duty(l)
    M2.duty(0)
    M3.duty(0)
    # reset()


def forward(l=100):
    M2.duty(l)
    M3.duty(l)
    M1.duty(0)
    M4.duty(0)
    # reset()


def Socket_fun(tim):
    pass


def get_json(uart_data):
    try:
        print("uart_data: ", uart_data)
        # find { and } first appear position, then cut the string
        if "{" in uart_data and "}" in uart_data:
            uart_data = uart_data[uart_data.index("{"): uart_data.index("}") + 1]
            print("json data: ", uart_data)
            return json.loads(uart_data)
        else:
            return {}
    except Exception as e:
        print("get json error, original data: ", e)
        return {}


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


def read_uart(tim):
    global uart
    if uart.any():
        data = get_json(uart.read().decode("utf-8"))
        control = data.get("control", 'down')
        speed = cap_duty_cycle(int(data.get("speed", 0)))
        if control == 'forward':
            forward(speed)
        elif control == 'backward':
            backward(speed)
        elif control == 'left':
            left(speed)
        elif control == 'right':
            right(speed)
        elif control == 'up':
            up(speed)
        elif control == 'down':
            up(0)
    # else:
    # reset()


p.on_write(on_rx)
tim = Timer(1)
tim.init(period=50, mode=Timer.PERIODIC, callback=read_uart)
