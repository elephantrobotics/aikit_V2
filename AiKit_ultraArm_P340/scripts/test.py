
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

# ua.set_angles(move_angles[0], 20)
# time.sleep(3)

# # send coordinates to move ultraArm
# ua.set_coords([x, -y, 58.84], 20)
# time.sleep(1.5)
# ua.set_coords([x, -y, 21.8], 20)
# time.sleep(2)

# # open pump

# time.sleep(1.5)

# ua.set_angle(2, 0, 30)
# time.sleep(0.3)
# ua.set_angle(3, 0, 30)
# time.sleep(1)

# ua.set_coords(move_coords[0], 20)



# radaus = [0.0, 0.25, 0.0]
# angles =[0.0, 14.32, 0.0]
# coords = [267.15, 0.0, 125.96]
# ua.set_radians(radaus, 30)
# time.sleep(4)
# print('111',ua.get_angles_info(),ua.get_coords_info())
# time.sleep(4)

# radaus1 = [0.0, 0.295, 0.434]
# angles1 =[0.0, 16.9, 24.87]
# coords1 = [259.81, 0.0, 65.51]
# ua.set_radians(radaus1, 30)
# time.sleep(4)
# print('222',ua.get_angles_info(),ua.get_coords_info())
# time.sleep(4)


# radaus2 = [0.0, 0.79, 1.047]
# angles2 =[0.0, 45.26, 59.99]
# coords2 = [257.36, 0.0, -29.73]
# ua.set_radians(radaus2, 30)
# time.sleep(4)
# print('333',ua.get_angles_info(),ua.get_coords_info())
# time.sleep(4)

# ua.set_angle(2, 0, 30)
# time.sleep(0.3)
# ua.set_angle(3, 0, 30)
# time.sleep(1)




d_radaus = [0.81, 0.0, 0.664]
d_angles = [47.9, 10.83, 24.58]
d_coords =[141.53, 148.67, 43.73]
ua.set_radians(d_radaus, 50)
time.sleep(8)
print('ddd',ua.get_angles_info(),ua.get_coords_info())
time.sleep(8)

# init_radus = [0.446, 0.0, 0.266]
# init_angles =[25.55, 0.0, 15.24]
# init_coords =[207.58, 99.23, 93.2]
# ua.set_coords([290.43, 174.51, 78.24], 50)
# time.sleep(8)

# print('555',ua.get_angles_info(),ua.get_coords_info())
# time.sleep(4)

c_raduis = [0.55, 0.556,0.419]
c_angles = []
c_coords = [287.85, 186.93, 51.57]
ua.set_radians(c_raduis, 50)
# ua.set_coords(c_coords, 50)
time.sleep(8)

print('ccc',ua.get_angles_info(),ua.get_coords_info())
time.sleep(8)

# a_raduis = [-0.541, 0.728,0.332]
# a_angles = []
# a_coords = []
# ua.set_radians(a_raduis, 50)
# time.sleep(8)

# print('aaa',ua.get_angles_info(),ua.get_coords_info())
# time.sleep(8)

# b_raduis = [-0.827, 0.05,0.603]
# b_angles = []
# b_coords = []
# ua.set_radians(b_raduis, 50)
# time.sleep(8)

# print('aaa',ua.get_angles_info(),ua.get_coords_info())
# time.sleep(8)


