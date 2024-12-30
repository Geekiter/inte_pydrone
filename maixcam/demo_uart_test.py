from maix import uart, app, time, camera, display

def on_received(serial, data):
    print("received:", data)
    # send back
    # serial.write(data)
ports = uart.list_devices()
print(ports)
serial = uart.UART(ports[0])
cam_width = 480
cam_height = 280
cam = camera.Camera(cam_width, cam_height)
disp = display.Display()
serial.set_received_callback(on_received)
while True:
    img = cam.read()
    obj = '{"control":"up", "speed": "0"}'
    serial.write_str(obj)
    time.sleep_ms(1000)
    # data = serial.readline()
    # if data:
    #     print(data)
#         serial.write_str(obj + data)
#         time.sleep_ms(1000)
    disp.show(img)