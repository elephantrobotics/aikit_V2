from pymycobot.mycobot import MyCobot
import time
from pymycobot import PI_BAUD, PI_PORT
import serial
import serial.tools.list_ports

plist = [
    str(x).split(" - ")[0].strip() for x in serial.tools.list_ports.comports()
]

# mc = MyCobot(PI_PORT, PI_BAUD)
mc = MyCobot(plist[0], 115200)

# 物块上方
cube_up = [180, 18.3, 150.4, -174.18, -1.4, -52.71]

# coords = [
#         [135.0, -65.5, 280.1, 178.99, 5.38, -179.9],
#         [136.1, -141.6, 243.9, 178.99, 5.38, -179.9]
#     ]
#
angles = [18.8, -7.91, -54.49, -23.02, 89.56, -14.76]
#
# mc.send_coords(coords[0], 20, 1)
# time.sleep(3)
# mc.send_coords(coords[1], 20, 1)
move_coords = [
    [113.1, -209.0, 324.9, -173.58, -0.02, -129.64],  # D Sorting area
    [227.0, -193.5, 326.0, -161.98, 0.08, -113.07],  # C Sorting area
    [153.6, 191.3, 315.9, -175.25, 0.07, -17.65],  # A Sorting area [134.5, 182.5, 336.4, -165.84, 0.04, -13.79]
    [-6.2, 218.1, 311.4, -177.53, -0.09, 25.57],  # B Sorting area
]
move_angles = [
    [-39.9, -10.1, -42.62, -30.84, 90.0, -0.26],  # D Sorting area [-41.04, -10.1, -42.09, -26.45, 90.0, 0.17]
    [-23.37, -39.72, -7.64, -24.6, 90.17, -0.26],  # C Sorting area [-23.29, -39.72, -7.38, -19.16, 89.91, 0.26]
    [72.15, -12.91, -42.62, -29.7, 90.08, -0.17],  # A Sorting area [76.37, -0.87, -54.66, -20.3, 90.0, 0.17]
    [115.4, -0.17, -57.65, -29.7, 89.91, -0.17],  # B Sorting area [115.48, -0.08, -57.83, -23.2, 89.82, 0.26]
]

# mc.send_angles(angles, 40)
#
# time.sleep(5)
# mc.send_coords(move_coords[2], 60, 0)
# time.sleep(7)
# print(mc.get_coords())
# print(mc.get_angles())
# time.sleep(5)
# mc.send_angles(angles, 40)
# time.sleep(5)
# mc.send_coords(move_coords[1], 80, 1)
# time.sleep(5)

# mc.send_angles(angles, 40)
# time.sleep(5)
# mc.send_coords(move_coords[2], 80, 1)
# time.sleep(4)

# mc.send_angles(angles, 40)
# time.sleep(5)
# mc.send_coords(move_coords[3], 80, 1)
# time.sleep(5)
# print(mc.get_coords())
# print(mc.get_angles())
# mc.send_angles(angles, 40)

radians_ = [
    [-0.71, -0.16, -0.71, -0.53, 1.57, 0.0],  # D
    [-0.42, -0.67, -0.09, -0.42, 1.57, 0.0],  # C
    [1.27, -0.21, -0.71, -0.51, 1.57, 0.0],  # A
    [2.03, 0.02, -0.99, -0.51, 1.57, 0.0],  # B
]

init_ = [0.29, -0.07, -0.90, -0.44, 1.57, 0.0]

mc.send_radians(init_, 50)
time.sleep(6)
print('init:', mc.get_coords(), mc.get_angles())
time.sleep(3)
#
# mc.send_radians(radians_[0], 50)
time.sleep(6)
print('D:', mc.get_coords(), mc.get_angles())

# time.sleep(3)
# mc.send_radians(init_, 50)
# time.sleep(4)
#
# mc.send_radians(radians_[1], 50)
# time.sleep(6)
# print('C:', mc.get_coords(), mc.get_angles())
#
# time.sleep(3)
# mc.send_radians(init_, 50)
# time.sleep(4)
#
# mc.send_radians(radians_[2], 50)
# time.sleep(6)
# print('A:', mc.get_coords(), mc.get_angles())

# time.sleep(3)
# mc.send_radians(init_, 50)
# time.sleep(4)

# mc.send_radians(radians_[3], 50)
# time.sleep(6)
# print('B:', mc.get_coords(), mc.get_angles())
#
# time.sleep(3)
# mc.send_radians(init_, 50)
# time.sleep(4)
