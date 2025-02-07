import json
import math

from maix import camera, display, i2c, pinmap, uart, image
from maix.image import ApriltagFamilies

from face_tracking import servos
from pid import PID
from vl53l1x import VL53L1X

# 机体偏向
structure_trend = "right"

# maixcam
cam_width = 480  # 480
cam_height = 280  # 280
safe_x = cam_width / 8
safe_y = cam_height / 4

safe_x1 = cam_width / 2 - safe_x
safe_x2 = cam_width / 2 + safe_x
safe_y1 = 0
safe_y2 = cam_height / 2 + safe_y
safe_y3 = cam_height

cam = camera.Camera(cam_width, cam_height)
disp = display.Display()

# 激光高度传感器
pinmap.set_pin_function("A15", "I2C5_SCL")
pinmap.set_pin_function("A27", "I2C5_SDA")
bus1 = i2c.I2C(5, i2c.Mode.MASTER)
slaves = bus1.scan()
print("find slaves:", slaves)
tof = VL53L1X(bus1, slaves[0])


def get_distance():
    dis = 0
    count = 10
    for i in range(count):
        dis += tof.read()
        # time.sleep_ms(10)
    out = dis / 10 / count
    # print("distance: ", out)
    return out


# 摄像头舵机
ROLL_PWM_PIN_NAME = "A19"
init_roll = 50  # 50 means middle
ROLL_DUTY_MIN = 2.5  # Minimum duty cycle for x-axis servos.
ROLL_DUTY_MAX = 12.5  # Maxmum duty cycle for x-axis servos.
roll = servos.Servos(ROLL_PWM_PIN_NAME, init_roll, ROLL_DUTY_MIN, ROLL_DUTY_MAX)

# Apriltag
families = ApriltagFamilies.TAG36H11

# drone通信
ports = uart.list_devices()
print("uart port: ", ports)
serial = uart.UART(ports[0])


class Drone:
    def __init__(self, serial):
        self.serial = serial

    def left(self, s=0):
        self.serial.write_str(json.dumps({
            "control": "left",
            "speed": s
        }))

    def right(self, s=0):
        self.serial.write_str(json.dumps({
            "control": "right",
            "speed": s
        }))

    def up(self, s=0):
        self.serial.write_str(json.dumps({
            "control": "up",
            "speed": s
        }))

    def forward(self, s=0):
        self.serial.write_str(json.dumps({
            "control": "forward",
            "speed": s
        }))

    def backward(self, s=0):
        self.serial.write_str(json.dumps({
            "control": "backward",
            "speed": s
        }))

    def stop(self):
        self.up(0)

    def duty(self, m1=0, m2=0, m3=0, m4=0):
        self.serial.write_str(json.dumps({
            "control": "duty",
            "m1": m1,
            "m2": m2,
            "m3": m3,
            "m4": m4
        }))


drone = Drone(serial)
target_height = 170
altitude_pid = PID(
    p=8,
    i=0,
    d=8,
    v_max=800.0,
    v_min=60.0,
    target=target_height + 40
)
angle_pid = PID(
    p=5,
    i=0.01,
    d=5,
    v_max=500,
    v_min=-500,
    target=0
)


def get_json(uart_data):
    try:
        uart_data = uart_data.decode('utf-8').strip()
        # find { and } first appear position, then cut the string
        if "{" in uart_data and "}" in uart_data:
            uart_data = uart_data[uart_data.index("{"): uart_data.index("}") + 1]
            # print("json data: ", uart_data)
            return json.loads(uart_data)
        else:
            return {}
    except Exception as e:
        print("get json error, original data: ", e)
        return {}


# 调整摄像头
roll.dir(30)

# 稳定次数
altitude_stable_count = 0
current_angle = 0.0

adjust_stable_count = 0

# 定高
# 80cm
while altitude_stable_count < 10:
    current_height = get_distance()
    print("current height: ", current_height)
    if math.fabs(current_height - target_height) < 10:
        altitude_stable_count += 1
    speed = altitude_pid.update(current_height)
    drone.up(speed)

target_action = [
    {"mode": "find_apriltags", "id": 22, "action": "find"},
    {"mode": "find_apriltags", "id": 23, "action": "find"},
    {"mode": "find_apriltags", "id": 24, "action": "find"},
    {"mode": "find_apriltags", "id": 25, "action": "find"},
    {"mode": "find_apriltags", "id": 26, "action": "find"},
    {"mode": "find_apriltags", "id": 27, "action": "find"},
    {"mode": "find_apriltags", "id": "", "action": "finished"},
]
current_action_index = 0
rotate = 30
adjust_status = False
# 旋转
while target_action[current_action_index]["action"] != "finished":
    img = cam.read()
    # 定高
    current_height = get_distance()
    up_speed = altitude_pid.update(current_height)
    # 和drone通信
    incoming_data = serial.read()
    json_data = get_json(incoming_data)
    if json_data:
        current_angle = json_data.get("yaw", -1)
        drone.up(up_speed)
    else:
        drone.up(up_speed)

        continue

    # 查找apriltag
    if adjust_status:
        disp.show(img)

        # 如果当前状态稳定了，就继续
        if math.fabs(current_angle - angle_pid.target) < 10:
            adjust_stable_count += 1
            if adjust_stable_count > 10:
                adjust_status = False
                adjust_stable_count = 0
        # 否则继续调整
        else:
            angel_speed = math.fabs(angle_pid.update(current_angle))
            if current_angle > angle_pid.target:
                # left
                drone.duty(m2=up_speed - angel_speed * 2, m3=up_speed + angel_speed * 2, m1=0, m4=0)
                print("left, current angle: ", current_angle, "target angle: ", angle_pid.target)
            else:
                # right
                drone.duty(m2=up_speed, m3=up_speed - angel_speed, m1=0, m4=0)
                print("right, current angle: ", current_angle, "target angle: ", angle_pid.target)
            continue

    tags = img.find_apriltags(families=ApriltagFamilies.TAG36H11)
    if not tags:
        drone.up(up_speed)
        # rotate = {30: 40, 40: 50, 50: 60, 60: 30}[rotate]
        # 默认会自转
        # roll.dir(rotate)
        disp.show(img)
        continue

    for a in tags:
        if a.id() != target_action[current_action_index]["id"]:
            continue

        corners = a.corners()
        for i in range(4):
            img.draw_line(corners[i][0], corners[i][1], corners[(i + 1) % 4][0], corners[(i + 1) % 4][1],
                          image.COLOR_GREEN, 2)
        # 确认是前进还是后退
        tag_x = a.x() + a.w() / 2
        tag_y = a.y() + a.h() / 2

        target_angle = 0.0
        forward_speed = 0

        if tag_x < cam_width / 2:
            target_angle = current_angle - math.atan((cam_width / 2 - tag_x) / (cam_height - tag_y)) * (
                    180 / math.pi)

        else:
            target_angle = current_angle + math.atan((tag_x - cam_width / 2) / (cam_height - tag_y)) * (
                    180 / math.pi)
        # target_angle在-180-0 0-180之间
        if target_angle > 180:
            target_angle = target_angle - 360
        elif target_angle < -180:
            target_angle = target_angle + 360

        angle_pid.target = target_angle
        # 计算tag面积
        tag_area = a.w() * a.h()

        if (safe_x1 < tag_x < safe_x2 and safe_y2 < tag_y < safe_y3) or tag_area > cam_width * cam_height / 3 * 2:
            current_action_index += 1
            print("next action: ", target_action[current_action_index])
            continue
        elif safe_x1 < tag_x < safe_x2 and safe_y1 < tag_y < safe_y2:
            forward_speed = 500
            drone.duty(m2=up_speed, m3=up_speed, m1=forward_speed, m4=forward_speed)
        else:
            adjust_status = True
        """
        angel_speed = angle_pid.update(current_angle)
        # print("angle: ", target_angle, "angel_speed: ", angel_speed)
        # print("pid target: ", angle_pid.target)
        if current_height < target_height / 2:
            turn_speed = up_speed - angel_speed
        else:
            turn_speed = 100

        if angel_speed < 0:
            angel_speed = abs(angel_speed)
            # left
            print('left')
            if structure_trend == "right":
                for _ in range(10):
                    drone.duty(m2=turn_speed, m3=up_speed + angel_speed, m1=forward_speed - angel_speed,
                               m4=forward_speed + angel_speed)
                    drone.up(up_speed)

            else:
                drone.duty(m2=up_speed, m3=up_speed, m1=forward_speed,
                           m4=forward_speed)

        elif tag_x >= safe_x2:
            angel_speed = abs(angel_speed)
            # right
            print('right')
            if structure_trend == "right":
                drone.duty(m2=up_speed, m3=up_speed, m1=forward_speed,
                           m4=forward_speed)
            else:
                drone.duty(m2=up_speed, m3=turn_speed, m1=forward_speed + angel_speed,
                           m4=forward_speed - angel_speed)
        """
    # drone.right(00)
    # 画安全区域
    img.draw_line(int(safe_x1), 0, int(safe_x1), cam_height, image.COLOR_RED, 2)
    img.draw_line(int(safe_x2), 0, int(safe_x2), cam_height, image.COLOR_RED, 2)
    img.draw_line(0, int(safe_y2), cam_width, int(safe_y2), image.COLOR_RED, 2)

    disp.show(img)
