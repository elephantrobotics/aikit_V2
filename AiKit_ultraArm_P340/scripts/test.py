
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
print(plist[0])
# print(plist[1])
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


"""ultraArm传送带套装点位调试"""

#识别抓取前物块上方的点
pre_radaus = [-0.62, 0.0, 0.0]
pre_angles = [-33.8, -3.44, -2.86, 0.0]
pre_coords = [188.65, -126.29, 136.75, -33.8]
# 抓取物块的点
cube_radaus = [-0.59, 0.19, 0.52]
cube_angles = [-33.8, 10.89, 29.79, 0.0]
cube_coords = [200.32, -134.1, 58.1, -33.8]
# cube_coords = [208.32, -140.1, 60.1, -33.8]
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
ua.set_angles(pre_angles, 60)
time.sleep(5)
print('pre_coords:',ua.get_angles_info(),ua.get_coords_info())
ua.set_radians(cube_radaus, 60)
time.sleep(5)
print('cube_coords:',ua.get_angles_info(),ua.get_coords_info())

ua.set_gpio_state(0)

ua.set_radians(fin_radaus, 60)
time.sleep(5)
print('finish_angles:', ua.get_angles_info(), ua.get_coords_info())

time.sleep(2)

ua.set_radians(cube_pre_radaus, 60)
time.sleep(5)
print('finish-pre:', ua.get_angles_info(), ua.get_coords_info())

time.sleep(3)

a_radaus = [-1.14, 0.41, 0.66]
a_angles = [-66.32, 18.49, 37.82, 0.0]
# a_coords =[107.48, -233.9, 33.38, -65.32]
a_coords =[99.13, -226.06, 37.44, -65.32]
# ua.set_radians(a_radaus, 30)
ua.set_coords(a_coords, 30)
time.sleep(8)
# ua.set_gpio_state(1)
print('aaaa:',ua.get_angles_info(),ua.get_coords_info())
time.sleep(8)
ua.set_gpio_state(1)
time.sleep(3)
# ua.set_radians(pre_radaus, 30)
#
# init_radus = [0.446, 0.0, 0.266]
# init_angles =[25.55, 0.0, 15.24]
# init_coords =[207.58, 99.23, 93.2]
# ua.set_coords([290.43, 174.51, 78.24], 50)
# time.sleep(8)
#
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
#
# c_raduis = [-1.61, 0.31,0.77]
# c_angles = [-92.25, 17.76, 44.12, 0.0]
# c_coords = [-9.23, -234.98, 26.34, -92.25]
# # ua.set_radians(c_raduis, 50)
# ua.set_coords(c_coords, 50)
# time.sleep(8)
# ua.set_gpio_state(1)
# print('ccc',ua.get_angles_info(),ua.get_coords_info())
# time.sleep(8)
#
# d_raduis = [-1.84, 0.32,0.71]
# d_angles = [-105.42, 18.33, 40.68, 0.0]
# d_coords = [-64.36, -233.34, 32.15, -105.42]
# ua.set_radians(d_raduis, 50)
# # ua.set_coords(c_coords, 50)
# time.sleep(8)
# ua.set_gpio_state(1)
# print('ddd',ua.get_angles_info(),ua.get_coords_info())
# time.sleep(8)


"""ultraArm滑轨套装点位调试"""
"""
# 初始点
init_radaus = [0.97, 0.0, 0.0]
init_angles = [55.58, 0.0, 0.0, 0.0]
init_coords = [132.83, 193.86, 130.0, 55.58]
ua.set_radians(init_radaus, 70)
time.sleep(3)
print('init',ua.get_angles_info(),ua.get_coords_info())
time.sleep(1)

#识别抓取前物块上方的点
pre_radaus = [0.24, 0.0, 0.07]
pre_angles = [13.75, 0.0, 4.01, 0.0]
pre_coords = [227.93, 55.77, 120.21, 13.75]

ua.set_radians(pre_radaus, 70)
time.sleep(3)

print('pre_robot',ua.get_angles_info(),ua.get_coords_info())
time.sleep(1)

# 抓取物块的点1
# cube_radaus = [0.24, 0.21, 0.56]
# cube_angles = [13.75, 12.03, 32.09, 0.0]
# cube_coords = [233.81, 57.21, 52.77, 13.75]
# ua.set_radians(cube_radaus, 70)
# time.sleep(3)
# ua.set_gpio_state(0)
# print('cube',ua.get_angles_info(),ua.get_coords_info())
# time.sleep(1)

# 抓取物块的点2

# cube_2_radians = [0.32, -0.15, 0.57]
# cube_2_angles = [18.33, -8.59, 32.66, 0.0]
# cube_2_coords = [183.63, 60.84, 52.99, 18.33]
# ua.set_radians(cube_2_radians, 70)
# time.sleep(3)
# ua.set_gpio_state(0)
# print('cube2',ua.get_angles_info(),ua.get_coords_info())
# time.sleep(1)

# 抓取物块的点3
# cube_3_radians = [0.24, 0.21, 0.55]
# cube_3_angles = [13.75, 12.03, 31.51, 0.0]
# cube_3_coords = [234.53, 57.39, 53.97, 13.75]
# ua.set_radians(cube_3_radians, 70)
# time.sleep(3)
# # ua.set_gpio_state(0)
# print('cube3',ua.get_angles_info(),ua.get_coords_info())
# time.sleep(1)

# # 抓取物块的点4
# cube_4_radians = [0.32, -0.17, 0.55]
# cube_4_angles = [18.33, -9.74, 31.51, 0.0]
# cube_4_coords = [182.6, 60.5, 54.96, 18.33]
# ua.set_radians(cube_4_radians, 70)
# time.sleep(3)
# # ua.set_gpio_state(0)
# print('cube4',ua.get_angles_info(),ua.get_coords_info())
# time.sleep(1)

# 抓取物块右上
right_up_radians = [0.13, 0.16, 0.58]
right_up_coords = [230.86, 30.19, 51.62, 7.45]
# ua.set_radians(right_up_radians, 70)
time.sleep(4)
# ua.set_gpio_state(0)
print('right_up',ua.get_angles_info(),ua.get_coords_info())
time.sleep(3)

ua.set_angles([0,0,0], 70)

# 抓取物块右下
right_down_radians = [0.16, -0.25, 0.55]
right_down_coords = [179.87, 29.04, 52.79, 9.17]
# ua.set_radians(right_down_radians, 70)
time.sleep(4)
# ua.set_gpio_state(0)
print('right_down',ua.get_angles_info(),ua.get_coords_info())
time.sleep(3)

ua.set_angles([0,0,0], 70)

# 抓取物块左上
left_up_radians = [-0.06, 0.14, 0.58]
left_up_coords = [229.83, -13.82, 52.01, -3.44]
# ua.set_radians(left_up_radians, 70)
time.sleep(4)
# ua.set_gpio_state(0)
print('left_up',ua.get_angles_info(),ua.get_coords_info())
time.sleep(3)

ua.set_angles([0,0,0], 70)

# 抓取物块左下
left_down_radians = [-0.07, -0.25, 0.56]
left_down_coords = [181.01, -12.69, 51.59, -4.01]
# ua.set_radians(left_down_radians, 70)
time.sleep(4)
# ua.set_gpio_state(0)
print('left_down',ua.get_angles_info(),ua.get_coords_info())
time.sleep(3)

ua.set_angles([0,0,0], 70)

# 抓取物块后的点
fin_radaus = [0.24, 0, 0.0]
fin_angles = [13.75, 0.0, 0.0, 0.0]
fin_coords = [228.27, 55.86, 130.0, 13.75]
# ua.set_radians(fin_radaus, 70)
time.sleep(3)
print('finish_',ua.get_angles_info(),ua.get_coords_info())
time.sleep(1)

# 抓取后的放置点
init_radaus = [0.97, 0.0, 0.0]
init_angles = [55.58, 0.0, 0.0, 0.0]
init_coords = [132.83, 193.86, 130.0, 55.58]
# ua.set_radians(init_radaus, 70)
time.sleep(3)
print('after',ua.get_angles_info(),ua.get_coords_info())
# ua.set_gpio_state(1)
time.sleep(1)
"""
