import json


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
