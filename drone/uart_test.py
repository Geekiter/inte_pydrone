import utime
from machine import UART

uart = UART(1, tx=43, rx=44)

while True:
    # obj = 'drone'

    utime.sleep_ms(50)
    if uart.any():
        
        data = str(uart.readline())
        print(data)

        utime.sleep_ms(1000)
        uart.write("drone")
