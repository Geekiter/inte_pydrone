import socket

from maix import camera, display, image
from maix.image import ApriltagFamilies

udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
ip = "192.168.124.35"
port = 2390
# 定义服务器的 IP 地址和端口号
server_address = (ip, port)


def udp_send(msg='ping!'):
    udp_socket.sendto(msg.encode('utf-8'), server_address)


def left():
    udp_send("l")


def right():
    udp_send("r")


def up():
    udp_send("u")


def forward():
    udp_send("f")


def backward():
    udp_send("b")


def stop():
    udp_send("n")


udp_send()
udp_send()
udp_send()

cam_width = 480
cam_height = 280
cam = camera.Camera(cam_width, cam_height)
disp = display.Display()
families = ApriltagFamilies.TAG36H11
safe_resolution = 100
safe_x_left = safe_resolution
safe_x_right = cam_width - safe_resolution
safe_y_up = safe_resolution
safe_y_down = cam_height - safe_resolution

while 1:
    img = cam.read()
    # 识别apriltag

    apriltags = img.find_apriltags(families=ApriltagFamilies.TAG36H11)
    for a in apriltags:
        print("rotation", a.rotation())
        corners = a.corners()
        for i in range(4):
            img.draw_line(corners[i][0], corners[i][1], corners[(i + 1) % 4][0], corners[(i + 1) % 4][1],
                          image.COLOR_GREEN, 2)

        x = a.x()
        y = a.y()

        print("x", x, "y", y)
        if safe_y_up < y < safe_y_down and safe_x_left < x < safe_x_right:
            print("down")
            up()
        else:
            if x < safe_x_left:
                print("left")
                left()
            elif x > safe_x_right:
                print("right")
                right()
            if y < safe_y_up:
                print("up")
                forward()
            elif y > safe_y_down:
                print("down")
                backward()


    disp.show(img)

# 靠近位置

# 下降
