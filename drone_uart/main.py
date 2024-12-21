import json

from machine import Pin, Timer, PWM, UART

import drone

# 构建四轴对象，无头方式
d = drone.DRONE(flightmode=0, debug=1)
M1 = PWM(Pin(4), freq=10000, duty=0)
M2 = PWM(Pin(5), freq=10000, duty=0)
M3 = PWM(Pin(40), freq=10000, duty=0)
M4 = PWM(Pin(41), freq=10000, duty=0)

uart = UART(1, tx=43, rx=44)


def cap_duty_cycle(value):
    return min(int((value / 100) * 1023), int(0.8 * 1023))


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


def control_motors_m1_m4(control_data):
    print('control data:', control_data)
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
            M1.duty(duty_m1)
            M4.duty(duty_m1)
            # proportion = (x_val + 100) / 200  # Map x to range between 0 and 1
            # M1.duty(int(duty_m1 * (1 - proportion)))
            # M4.duty(int(duty_m4 * proportion))


    else:
        M1.duty(0)
        M4.duty(0)

    # Print the current duty cycles for M1 and M4
    print(f"M1 duty cycle: {M1.duty()} / 1023, M4 duty cycle: {M4.duty()} / 1023")


def up(l=50):
    control_data = [0, l, 0, l]
    # control_data[1] = l
    control_motors_m2_m3(control_data)
    control_motors_m1_m4(control_data)


def right(l=100):
    control_motors_m1_m4([0, 0, 0, 0])
    control_motors_m2_m3([100, l, 0, 0])


def left(l=100):
    control_motors_m1_m4([0, 0, 0, 0])
    control_motors_m2_m3([-100, l, 0, 0])


def backward(l=100):
    control_motors_m2_m3([0, 0, 0, 0])
    control_motors_m1_m4([0, 0, 0, l])


def forward(l=100):
    control_motors_m2_m3([0, l, 0, 0])
    control_motors_m1_m4([0, 0, 0, 0])


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


def read_uart(tim):
    global uart
    if uart.any():
        data = get_json(uart.read().decode("utf-8"))
        control = data.get("control", 'down')
        speed = int(data.get("speed", 0))
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

tim = Timer(1)
tim.init(period=50, mode=Timer.PERIODIC, callback=read_uart)
