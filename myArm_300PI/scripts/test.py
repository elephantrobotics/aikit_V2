from pymycobot.myarm import MyArm
import time
from pymycobot import PI_BAUD, PI_PORT

mc = MyArm('/dev/ttyAMA0')

coords = [ 
        [135.0, -65.5, 280.1, 178.99, 5.38, -179.9],
        [136.1, -141.6, 243.9, 178.99, 5.38, -179.9]
    ]

angles = [0, 0, 0, 0, 0, 0]

#mc.send_coords(coords[0], 20, 1)
#time.sleep(3)
#mc.send_coords(coords[1], 20, 1)
x = 185.19
y = 3.78
mc.send_angles([0, 0, 0, -90, 0, -83, 0], 35)  # [126.1, 0.7, 217.0, 179.64, -1.14, -179.64]
time.sleep(3)

# send coordinates to move myArm
mc.send_coords([x, y, 190.5, -179.72, 6.5, -179.43], 40)  # [164.2, 0.9, 190.5, 179.65, -1.14, 179.94]
time.sleep(3)

# self.ma.send_coords([x, y, 150, 179.87, -3.78, -62.75], 25, 0)
# time.sleep(3)

mc.send_coords([x, y, 109.6, -179.72, 6.5, -179.43], 40)  # [165.0, 0.9, 109.6, 179.69, 0.08, 179.78]
time.sleep(3)

# mc.send_angles([0, -22, 0, -70, 0, -90, 0], 50)
# time.sleep(3)
#
# mc.send_angles([0, -40, 0, -90, 0, -52, 0], 50)
# time.sleep(3)

# open pump
#self.gpio_status(True)
time.sleep(1.5)

tmp = []
while True:
    if not tmp:
        tmp = mc.get_angles()
    else:
        break
time.sleep(0.5)

# print(tmp)
mc.send_angles([tmp[0], 0, 0, -90, -0.79, -83, tmp[6]], 50)  # [18.8, -7.91, -54.49, -23.02, -0.79, -14.76]
time.sleep(3)

