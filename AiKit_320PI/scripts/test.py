import time

from pymycobot.mycobot320 import MyCobot320

mc = MyCobot320('/dev/ttyAMA0', 115200)

coords = [ 
        [135.0, -65.5, 280.1, 178.99, 5.38, -179.9],
        [136.1, -141.6, 243.9, 178.99, 5.38, -179.9]
    ]

angles = [0, 0, 0, 0, 0, 0]

mc.send_coords(coords[0], 20, 1)
time.sleep(3)
mc.send_coords(coords[1], 20, 1)


