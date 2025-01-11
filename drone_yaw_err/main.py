import json
import time

import ble_simple_peripheral
import bluetooth
from machine import Pin, Timer, PWM, UART

import drone

# 读取wifi账号密码
# wificonf.txt文件内容格式为：
"""
ssid=xxx
pw=xxx
"""

# 构建四轴对象，无头方式
d = drone.DRONE(flightmode=0, debug=1)
M1 = PWM(Pin(4), freq=10000, duty=0)
M2 = PWM(Pin(5), freq=10000, duty=0)
M3 = PWM(Pin(40), freq=10000, duty=0)
M4 = PWM(Pin(41), freq=10000, duty=0)

state_buf = [None] * 18

uart = UART(1, tx=43, rx=44)
ble = bluetooth.BLE()
p = ble_simple_peripheral.BLESimplePeripheral(ble, name='pyDrone')

cal_led = Pin(42, Pin.OUT)  # green LED
cal_led.value(0)
while True:

    # 打印校准信息，当返回3个值均少于5000时校准通过。
    print(d.read_cal_data())

    # 校准通过
    if d.read_calibrated():
        print(d.read_cal_data())

        break

    time.sleep_ms(100)

# init_yaw = d.read_states()[2] / 100.0
# print("init_yaw:", init_yaw)
# current_yaw = init_yaw
# start_time = time.time()
# times = 10
# end_time = start_time + times
# print("start_time:", start_time)
# print("end_time:", end_time)
# # 把校准数据写入文件
# with open("cal_data.txt", "w") as f:
#     # start_time, end_time, init_yaw, ave_add_yaw
#     f.write("start_time: {}\n".format(start_time))
#     f.write("end_time: {}\n".format(end_time))
#     f.write("init_yaw: {}\n".format(init_yaw))

# while time.time() < end_time:
#     current_yaw = d.read_states()[2] / 100.0
#     print("current_yaw:", current_yaw)
#     with open("cal_data.txt", "a") as f:
#         f.write("current_yaw: {}\n".format(current_yaw))
#     time.sleep(1)

timer_period = 50
# ave_add_yaw = (current_yaw - init_yaw) / times
# ave_add_yaw = 0.075
# print("ave_add_yaw:", ave_add_yaw)
with open("cal_data.txt", "w") as f:
    f.write("cal data" + "\n")

cal_led.value(1)

# Global variable to count elapsed time in milliseconds
elapsed_ms = 0


def cap_duty_cycle(value):
    return min(int((value / 100) * 1023), int(0.8 * 1023))


def control_motors_m1_m4(control_data):
    y_val = control_data[3]
    x_val = control_data[2]

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
        # reset()
        pass


def control_motors_m2_m3(control_data):
    print('control data:', control_data)

    y_val = control_data[1]
    x_val = control_data[0]

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
    # M1.duty(l)
    # adj = int(0.8 * l)
    adj = int(0.25 * l)
    l_up = l + adj
    l_down = l - adj
    print('l_up:', l_up)
    print('l_down:', l_down)

    M3.duty(l_up)
    M2.duty(l_down)
    # M4.duty(l)


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


cal_first = False
cal_count = 0
calibrate = False
init_yaw = 0
current_yaw = 0
ave_add_yaw = 0


def on_rx(text):
    global state_buf
    global uart
    global cal_first
    global cal_count
    global calibrate
    global init_yaw
    global current_yaw
    global ave_add_yaw
    global cal_led

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
    # calibrate
    if not cal_first:
        cal_first = True
        init_yaw = states[2]
        cal_count = 0
    if not calibrate:
        if cal_count == 100:
            current_yaw = states[2]
            ave_add_yaw = (current_yaw - init_yaw) / 100
            calibrate = True
            with open("cal_data.txt", "a") as f:
                f.write("ave_add_yaw: {}\n".format(ave_add_yaw))
            cal_led.value(0)
            cal_led.value(1)
    cal_count += 1
    states = list(states)
    print('before states:', states)
    states[2] = states[2] - cal_count * ave_add_yaw

    # print("minus:", (time.time() - start_time) * ave_add_yaw * 100)
    # with open("cal_data.txt", "a") as f:
    #     seconds = time.time() - start_time
    #     f.write("time: {}\n".format(seconds))
    #     f.write("minus: {}\n".format(seconds * ave_add_yaw * 100))
    print('after states:', states)

    state_buf = [None] * 18
    for i in range(9):
        for j in range(2):
            if j == 0:
                state_buf[i * 2 + j] = int((states[i] + 32768) / 256)
            else:
                state_buf[i * 2 + j] = int((states[i] + 32768) % 256)

    p.send(bytes(state_buf))  # Send state back via Bluetooth

    uart_out_enable = 1

    if uart_out_enable:
        # 1. Check if state_buf has any None
        # if any(x is None for x in state_buf):
        if (state_buf[2] is None):
            # If there's at least one None, skip sending
            print("Skipping UART Yaw send because Yaw has None.")
        else:
            # Otherwise, send data
            YAW_val = states[2] / 100.0
            yaw_str = "YAW={:.2f}\n".format(YAW_val)
            uart.write(yaw_str)
            print("UART ->", yaw_str.strip())  # For debugging


#             uart.write(bytearray(state_buf))
#             print("Sent data:", state_buf)


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


def write_uart(t):
    global uart
    global state_buf

    global elapsed_ms

    # ----- Your periodic task here -----
    # For example, sending something via UART:
    # uart.write("Hello from Timer!\n")

    # Increase elapsed time by 'period' of the timer (in ms)
    elapsed_ms += 50  # because period=50 ms below

    # Check if 30 seconds (30,000 ms) have passed
    if elapsed_ms >= 30000:
        # Stop the timer
        t.deinit()
        print("Timer stopped after 30 seconds")

    # 1. Check if state_buf has any None
    if any(x is None for x in state_buf):
        # If there's at least one None, skip sending
        print("Skipping UART send because state_buf has None.")
    else:
        # Otherwise, send data
        uart.write(bytearray(state_buf))
        print("Sent data:", state_buf)


p.on_write(on_rx)
tim = Timer(1)
tim.init(period=50, mode=Timer.PERIODIC, callback=read_uart)
# tim.init(period=50, mode=Timer.PERIODIC, callback=write_uart)
