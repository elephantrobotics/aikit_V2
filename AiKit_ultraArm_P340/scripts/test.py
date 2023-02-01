
import serial
import os
import serial.tools.list_ports
from pymycobot.ultraArm import ultraArm
import time

plist = [
        str(x).split(" - ")[0].strip() for x in serial.tools.list_ports.comports()
    ]
# print(plist[0])

# mc = MyCobot(plist[0], 115200)
# print(mc.get_angles())

# img_path = r'{}'.format(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# print(img_path)
# img = os.path.dirname(os.path.abspath(__file__))
# print(img)
# path = os.path.split(os.path.abspath(__file__))
# print(path)
# path1 = os.path.dirname(os.path.split(os.path.abspath(__file__))[0])
# print(path1)



ua = ultraArm(plist[0])
ua.go_zero()

# move_angles = [
#     [0.0, 0.0, 0.0],  # init the point
#     [19.48, 0.0, 0.0],  # point to grab
#     # [17.4, -10.1, -87.27, 5.8],  # point to grab
# ]

# # 移动坐标
# move_coords = [
#     [121.35, 127.48, 120.0],   # D Sorting area
#     [217.09, 113.01, 98.36],    # C Sorting area
#     [107.54, -171.23, 117.11],   # A Sorting area
#     [-6.91, -175.86, 120.0],     # B Sorting area
# ]
# x = 160
# y = 10

#识别抓取前物块上方的点
pre_radaus = [-0.62, 0.0, 0.0]
pre_angles = [-35.52, 0.0, 0.0, 0.0]
pre_coords = [191.27, -136.53, 130.0, -35.52]
# 抓取物块的点
cube_radaus = [-0.59, 0.25, 0.47]
cube_angles = [-33.8, 14.32, 26.93, 0.0]
cube_coords = [209.39, -140.17, 62.55, -33.8]

# 抓取物块后的点
fin_radaus = [-0.62, 0.12, 0.0]
fin_angles = [-35.52, 6.88, 0.0, 0.0]
fin_coords = [203.94, -145.58, 129.06, -35.52]

# 识别抓取后 位置缓冲点
cube_pre_radaus= [-1.40, 0.0, 0.17]
cube_pre_angles =[-80.21, 0.0, 9.74, 0.0]
cube_pre_coords = [39.62, -229.59, 106.32, -80.21]

ua.set_angles([91.57, 7.6, 0.16], 50)
ua.sleep(2)
# ua.set_radians(pre_radaus, 30)
ua.set_angles(pre_angles, 30)
time.sleep(5)
print('pre_coords:',ua.get_angles_info(),ua.get_coords_info())
ua.set_radians(cube_radaus, 30)
time.sleep(5)
print('cube_coords:',ua.get_angles_info(),ua.get_coords_info())

ua.set_gpio_state(0)

ua.set_radians(fin_radaus, 30)
time.sleep(5)
print('finish_angles:', ua.get_angles_info(), ua.get_coords_info())

time.sleep(2)

ua.set_radians(cube_pre_radaus, 30)
time.sleep(5)
print('finish-pre:', ua.get_angles_info(), ua.get_coords_info())

time.sleep(3)

# a_radaus = [-1.14, 0.41, 0.66]
# a_angles = [-65.32, 23.49, 37.82, 0.0]
# a_coords =[107.48, -233.9, 33.38, -65.32]
# # ua.set_radians(a_radaus, 30)
# ua.set_coords(a_coords, 30)
# time.sleep(8)
# ua.set_gpio_state(1)
# print('aaaa:',ua.get_angles_info(),ua.get_coords_info())
# time.sleep(8)

# time.sleep(3)
# ua.set_radians(pre_radaus, 30)

# init_radus = [0.446, 0.0, 0.266]
# init_angles =[25.55, 0.0, 15.24]
# init_coords =[207.58, 99.23, 93.2]
# ua.set_coords([290.43, 174.51, 78.24], 50)
# time.sleep(8)

# print('555',ua.get_angles_info(),ua.get_coords_info())
# time.sleep(4)

# b_raduis = [-1.37, 0.31,0.77]
# b_angles = [-78.5, 17.76, 44.12, 0.0]
# b_coords = [46.88, -230.44, 26.34, -78.5]
# # ua.set_radians(b_raduis, 50)
# ua.set_angles(b_angles, 50)
# time.sleep(8)
# ua.set_gpio_state(1)
# print('bbb:',ua.get_angles_info(),ua.get_coords_info())
# time.sleep(8)

# c_raduis = [-1.61, 0.31,0.77]
# c_angles = [-92.25, 17.76, 44.12, 0.0]
# c_coords = [-9.23, -234.98, 26.34, -92.25]
# # ua.set_radians(c_raduis, 50)
# ua.set_coords(c_coords, 50)
# time.sleep(8)
# ua.set_gpio_state(1)
# print('ccc',ua.get_angles_info(),ua.get_coords_info())
# time.sleep(8)

d_raduis = [-1.84, 0.32,0.71]
d_angles = [-105.42, 18.33, 40.68, 0.0]
d_coords = [-64.36, -233.34, 32.15, -105.42]
ua.set_radians(d_raduis, 50)
# ua.set_coords(c_coords, 50)
time.sleep(8)
ua.set_gpio_state(1)
print('ddd',ua.get_angles_info(),ua.get_coords_info())
time.sleep(8)


