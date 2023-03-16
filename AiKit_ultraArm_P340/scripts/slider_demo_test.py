import time 

import serial
import serial.tools.list_ports
from pymycobot.ultraArm import ultraArm
from megaAiKit import megaAikit

plist = [
        str(x).split(" - ")[0].strip() for x in serial.tools.list_ports.comports()
    ]

# ua = ultraArm(plist[0])
# ua.go_zero()
kit = megaAikit(plist[0])
time.sleep(0.05)
print(plist[0])

move_angles = [
            [-55, 0, 1, 0.0],  # init the point
            [13.75, 0.0, 4.01, 0.0],  # 识别抓取前物块上方的点
            [13.75, 12.03, 31.51, 0.0],  # 抓取物块的点
            [-79, -8, 12, 0.0],  # 识别抓取之后的缓冲点
            [85.58, 0.0, 0.0, 0.0],  # point to grab
        ]

# 移动坐标
move_coords = [
            [131.08, -208.72, 45.88, -57.87],  # A区域
            [73.01, -212.63, 35.35, -71.05],  # B区域
            [19.33, -212.36, 37.04, -84.8],  # C区域
            [-35.89, -210.19, 37.04, -99.69],  # D区域
        ]

# 抓取物块的点

# for i in range(0, 4):

#     ua.set_angles(move_angles[0], 80)
#     time.sleep(3)
    
#     ua.set_angles([0,0,0], 80)

#     ua.set_coords([239.86, 24.89, 41.15], 80)
#     time.sleep(0.5)

#     ua.set_gpio_state(0)

#     ua.set_angle(3, 0, 80)
#     time.sleep(0.5)
#     # self.ua.set_angle(2, 0, 80)

#     # 放置物料盒之前的缓冲点
#     ua.set_angles(move_angles[3], 80)
#     time.sleep(3)
#     ua.set_coords(move_coords[i], 80)

#     time.sleep(4)
#     ua.set_gpio_state(1)

#     time.sleep(1)

#     ua.set_angles(move_angles[0], 80)

direction = 0


def slider_rail():
    global direction
    # if flag:
    while True:
        print('111111111111')
        time.sleep(0.05)
        direction = 0
        kit.write_distance(7, 80, 11)
        # a=kit.get_dir(1)
        # print(a)
        print('22222222222')
        time.sleep(44)
        print('3333333333333')
        
        kit.write_distance(2, 80, 11)
        direction = 1
        time.sleep(36)
        print('44444444444444')
            
    # else:
    #     return None
import threading   
# kit.write_distance_zero(100)
# t = threading.Thread(target = slider_rail)
# t.setDaemon = True
# t.start()
# slider_rail()
# if direction == 0:
#     print('000000000')
# elif direction == 1:
#     print('00001111111111')
# time.sleep(8)
# a=kit.get_dir(1)
# print(a)
kit.write_distance(7, 100, 11)
# print(kit.get_distance())
# time.sleep(5)
# kit.set_dir(1, 1)
# a=kit.get_dir(1)
# print(a)
# time.sleep(0.5)
# kit.write_distance(7,80, 11)
# time.sleep(30)

# time.sleep(4)
# print(kit.get_distance())
# kit.write_distance(2, 100, 11)
# time.sleep(2)
# a=kit.get_dir(1)
# print(a)
