import json
import time

import ble_simple_peripheral
import bluetooth
from machine import Timer, UART, Pin

from ModDrone import myDrone

# 构建四轴对象，无头方式
myDrone = myDrone()
state_buf = [None] * 18

uart = UART(1, tx=43, rx=44)
blue_port = bluetooth.BLE()
blue = ble_simple_peripheral.BLESimplePeripheral(blue_port, name='pyDrone')

cal_led = Pin(42, Pin.OUT)  # green LED
cal_led.value(0)
while True:
    # 打印校准信息，当返回3个值均少于5000时校准通过。
    print(myDrone.devices.read_cal_data())
    # 校准通过
    if myDrone.devices.read_calibrated():
        print(myDrone.devices.read_cal_data())
        break
    time.sleep_ms(100)

cal_led.value(1)

cal_first = False
cal_count = 0
calibrate = False
init_yaw = 0
current_yaw = 0
ave_add_yaw = 0


def on_controller(text):
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
    myDrone.control_motors_m2_m3(control_data)

    # Control M1 and M4 (Handle B)
    myDrone.control_motors_m1_m4(control_data)
    # myDrone.reset()

    # Detect button press
    if text[5] == 24:  # Y button pressed
        print('Y')
    elif text[5] == 136:  # X button pressed, stop all motors
        print('X')
        myDrone.reset()

    # Read drone state
    states = myDrone.devices.read_states()
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
            cal_led.value(0)
            cal_led.value(1)
    cal_count += 1
    states = list(states)
    print('before states:', states)
    states[2] = states[2] - cal_count * ave_add_yaw

    print('after states:', states)
    state_buf = [None] * 18
    for i in range(9):
        for j in range(2):
            if j == 0:
                state_buf[i * 2 + j] = int((states[i] + 32768) / 256)
            else:
                state_buf[i * 2 + j] = int((states[i] + 32768) % 256)

    blue.send(bytes(state_buf))  # Send state back via Bluetooth

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
            if calibrate:
                uart.write(json.dumps({
                    "yaw": YAW_val
                }))
            print("UART ->", yaw_str.strip())  # For debugging


def get_json(uart_data):
    try:
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
            myDrone.forward(speed)
        elif control == 'backward':
            myDrone.backward(speed)
        elif control == 'left':
            myDrone.left(speed)
        elif control == 'right':
            myDrone.right(speed)
        elif control == 'up':
            myDrone.up(speed)
        elif control == 'down':
            myDrone.up(0)


blue.on_write(on_controller)
tim = Timer(1)
tim.init(period=50, mode=Timer.PERIODIC, callback=read_uart)
