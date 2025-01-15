from machine import Pin, PWM

import drone


def cap_duty_cycle(value):
    return min(int((value / 100) * 1023), int(0.8 * 1023))


class myDrone:
    def __init__(self, m1_pin=4, m2_pin=5, m3_pin=40, m4_pin=41):
        self.devices = drone.DRONE(flightmode=0, debug=1)
        self.m1 = PWM(Pin(m1_pin), freq=10000, duty=0)
        self.m2 = PWM(Pin(m2_pin), freq=10000, duty=0)
        self.m3 = PWM(Pin(m3_pin), freq=10000, duty=0)
        self.m4 = PWM(Pin(m4_pin), freq=10000, duty=0)

    def control_motors_m2_m3(self, control_data):
        y_val = control_data[1]
        x_val = control_data[0]

        duty_m2 = cap_duty_cycle(abs(y_val))
        duty_m3 = cap_duty_cycle(abs(x_val))
        if y_val > 20:
            if x_val > 80:
                self.left(duty_m2)
            elif x_val < -80:
                self.right(duty_m2)
            else:
                self.forward(duty_m2)
        elif y_val < -20:
            self.backward(duty_m2)
        else:
            self.reset()

    def control_motors_m1_m4(self, control_data):
        print('control data:', control_data)
        y_val = control_data[3]
        x_val = control_data[2]

        duty_m1 = cap_duty_cycle(abs(y_val))
        duty_m4 = cap_duty_cycle(abs(x_val))
        if y_val > 5:
            self.up(duty_m1)
        elif y_val < -20:
            self.reset()

    def reset(self):
        self.m1.duty(0)
        self.m2.duty(0)
        self.m3.duty(0)
        self.m4.duty(0)

    def up(self, l=50):
        # self.m1.duty(l)
        adj = int(0.35 * l)
        l_up = l + adj
        l_down = l - adj
        print(l_up)

        self.m3.duty(l_up)
        self.m2.duty(l_down)

        # self.m1.duty(0) # 不影响转弯
        # self.m4.duty(0)

    def right(self, l=100):
        self.m4.duty(0)
        self.m1.duty(l)
        # self.m3.duty(0) # 不影响定高
        # self.m2.duty(0)

    def left(self, l=100):
        self.m1.duty(0)
        self.m4.duty(l)
        # self.m3.duty(0)
        # self.m2.duty(0)

    def backward(self, l=100):
        self.right(l)

    def forward(self, l=100):
        # self.m2.duty(l)
        # self.m3.duty(l)
        self.m1.duty(l)
        self.m4.duty(l)
