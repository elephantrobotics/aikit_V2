from pymycobot.mycobot280 import MyCobot280
import time
from pymycobot import PI_BAUD, PI_PORT
import serial
import serial.tools.list_ports

plist = [
    str(x).split(" - ")[0].strip() for x in serial.tools.list_ports.comports()
]

mc = MyCobot280(plist[0], 115200)

coords = [ 
        [135.0, -65.5, 280.1, 178.99, 5.38, -179.9],
        [136.1, -141.6, 243.9, 178.99, 5.38, -179.9]
    ]

angles = [0, 0, 0, 0, 0, 0]

mc.send_coords(coords[0], 20, 1)
time.sleep(3)
mc.send_coords(coords[1], 20, 1)


