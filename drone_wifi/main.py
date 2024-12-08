'''
实验名称：WiFi遥控四轴飞行器（pyDrone四轴代码）
版本：v1.0
日期：2022.6
作者：01Studio
说明：通过Socket UDP连接，周期接收手柄发来的控制信息，并回传自身姿态信息。
'''

import socket
import time

# 导入相关模块
import network
from machine import Pin, Timer, PWM, SoftI2C

import drone
wifi_name = "xxx"
wifi_password = 'xxx'
# 构建四轴对象，无头方式
d = drone.DRONE(flightmode=0, debug=1)
M1 = PWM(Pin(4), freq=10000, duty=0)
M2 = PWM(Pin(5), freq=10000, duty=0)
M3 = PWM(Pin(40), freq=10000, duty=0)
M4 = PWM(Pin(41), freq=10000, duty=0)
i2c = SoftI2C(scl=Pin(18), sda=Pin(17))
devices = i2c.scan()
print('i2c devices: ', devices)


# tof = VL53L1X(i2c)


# def getDistance():
#     return tof.read() / 10  # mm


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


# 开启AP热点
def startAP():
    wlan_ap = network.WLAN(network.AP_IF)

    print('Connect pyDrone AP to Config WiFi.')

    # 启动热点，名称为pyDrone，不加密。
    wlan_ap.active(True)
    wlan_ap.config(essid='pyDrone', authmode=0)

    while not wlan_ap.isconnected():  # 等待AP接入

        pass


def WIFI_Connect():
    WIFI_LED = Pin(46, Pin.OUT)  # 初始化 WIFI 指示灯
    wlan = network.WLAN(network.STA_IF)  # STA 模式
    wlan.active(True)  # 激活接口
    start_time = time.time()  # 记录时间做超时判断

    if not wlan.isconnected():
        print('connecting to network...')
        wlan.connect(wifi_name, wifi_password)  # 输入 WIFI 账号密码

        while not wlan.isconnected():
            # LED 闪烁提示
            WIFI_LED.value(1)
            time.sleep_ms(300)
            WIFI_LED.value(0)
            time.sleep_ms(300)

            # 超时判断, 15 秒没连接成功判定为超时
            if time.time() - start_time > 15:
                print('WIFI Connected Timeout!')
                wlan.active(False)
                break

    if wlan.isconnected():
        # LED 点亮
        WIFI_LED.value(1)
        # 串口打印信息
        print('network information:', wlan.ifconfig())


# 启动AP
# startAP()

WIFI_Connect()

# 创建socket UDP接口。
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('0.0.0.0', 2390))  # 本地IP：192.168.4.1;端口:2390

# 等待设备Socket接入，获取对方IP地址和端口
data, addr = s.recvfrom(128)
print(addr)

# 连接对方IP地址和端口
s.connect(addr)
s.setblocking(False)  # 非阻塞模式


# Socket接收数据
def Socket_fun(tim):
    try:
        text = s.recv(128)  # 单次最多接收128字节
        print("text: ", text)
        text1 = text.decode()
        for i in text1:
            print(i)

        if len(text1) > 0:
            if text1[0] == "u":
                up(40)
            elif text1[0] == 'n':
                up(0)
            elif text1[0] == 'l':
                left(40)
            elif text1[0] == 'r':
                right(40)
            elif text1[0] == 'f':
                forward(40)
            elif text1[0] == 'b':
                backward(40)


    except OSError:
        pass


# 开启定时器,周期50ms，执行socket通信接收任务
tim = Timer(1)
tim.init(period=50, mode=Timer.PERIODIC, callback=Socket_fun)

# while True:
#
#     time.sleep_ms(200)
