import json

from maix import camera, display, i2c, pinmap, uart
from maix.image import ApriltagFamilies


from pid import PID
from vl53l1x import VL53L1X

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
    return dis / 10 / count


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

drone = Drone(serial)

cam_width = 480
cam_height = 280
cam = camera.Camera(cam_width, cam_height)
disp = display.Display()
families = ApriltagFamilies.TAG36H11

altitude_pid = PID(
    p=2,
    i=0,
    d=6,
    v_max=100.0,
    v_min=-100.0,
    target=100
)

angle_pid = PID(
    p=6,
    i=0.01,
    d=6,
    v_max=500,
    v_min=-500,
    target=-40
)


def get_json(uart_data):
    try:
        uart_data = uart_data.decode('utf-8').strip()
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


while True:
    img = cam.read()
    incoming_data = serial.read()
    json_data = get_json(incoming_data)
    if json_data:
        yaw = json_data.get("yaw", angle_pid.target * 100)
        angle = yaw
        angel_speed = angle_pid.update(angle)
        print("angle: ", angle, "angel_speed: ", angel_speed)
        if angel_speed > 0:
            drone.left(angel_speed)
        else:
            drone.right(abs(angel_speed))
    # drone.right(00)
    disp.show(img)
