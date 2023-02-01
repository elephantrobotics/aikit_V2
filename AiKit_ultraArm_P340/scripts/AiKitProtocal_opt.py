from time import time
from threading import Thread
import time
from pymycobot.ultraArm import ultraArm
from Mega2560_AiKit import Mega2560_AiKit

aikit = Mega2560_AiKit("COM10")
ua = ultraArm("COM8", 115200)

default_down_z_postion = 126.47
down_x = -6.91
down_y = 252.1
down_z_offset = 40
first_point_anges = [91.57, 7.6, 0.16]
leave_to_default_angles = [91.57, 0, 0]
slide_rail_up_angles = [38, 10, -3]
slide_rail_down_angles = [38, 15, 23]

#初始点位
def initial_point(speed, times):
    ua.set_angles(first_point_anges, speed)
    time.sleep(times)

#吸取的上下动作，吸取时打开吸泵
def pickup(down_z, speed, times):
    # time.sleep(3)
    # print("Before Angles:", ua.get_angles_info())
    # time.sleep(0.01)
    # print("Before Coords:", ua.get_coords_info())
    # time.sleep(3)

    #下
    ua.set_coords([down_x, down_y, down_z], speed)
    time.sleep(times)

    # time.sleep(3)
    # print("After Angles:", ua.get_angles_info())
    # time.sleep(0.01)
    # print("After Coords:", ua.get_coords_info())
    # time.sleep(3)

    #打开吸泵
    ua.set_gpio_state(0)
    time.sleep(times)

    #上
    ua.set_angles(leave_to_default_angles, speed)
    time.sleep(times)

#移动吸取的物体到滑轨
def move_to_slide_rail(speed, times):
    #移动到指定默认的滑轨上方位置
    ua.set_angles(slide_rail_up_angles, speed)
    time.sleep(times)
    
    #下放到滑轨的位置
    ua.set_angles(slide_rail_down_angles, speed)
    time.sleep(times)

    #关闭吸泵
    ua.set_gpio_state(1)
    time.sleep(times)

    #打开传送带，必须用1秒
    # aikit.control_slide_rail(1)
    # time.sleep(1)

    #回到指定默认滑轨上方位置，为下一次吸取做准备
    ua.set_angles(slide_rail_up_angles, speed)
    time.sleep(times)

    #关闭传送带，必须用1秒
    # aikit.control_slide_rail(0)
    # time.sleep(1)

#检查识别物体的距离区间范围
def detect_tof_distance(dist, min_range, max_range):
    if dist >= min_range and dist <= max_range:
        return True
    else:
        return False

#获取传感器距离的工作接口，延时必须给 0.5
class ThreadTof(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.running = True
        self.dist = None
    def run(self):
        while self.running:
            if (not self.running):
                self.dist = None
                break
            else:
                dist = aikit.get_tof_dist()
                if (None != dist):
                    self.dist = dist
                    # print(self.dist)
                time.sleep(0.5)
    def get_tof_dist(self):
        return self.dist
    def set_flag(self, flag):
        self.running = flag

is_detect = False

def motion_plan(speed, ms):
    tof_thread = ThreadTof()
    # tof_thread.setDaemon(True)
    tof_thread.daemon = True
    tof_thread.start()
    ua.go_zero()
    
    down_z = None
    # while True:
    for i in range(0, 5):
        down_z = default_down_z_postion
        count = -1  #根据木块决定计算偏移量的位置
        is_detect = False
        curr_detect_dist = tof_thread.get_tof_dist()
        print("Current detected distance:", curr_detect_dist)
        if (245 <= curr_detect_dist and 361 >= curr_detect_dist):
            #最上方木块
            if (detect_tof_distance(curr_detect_dist, 245, 285)):
                is_detect = True
                count = 1
            #第二块木块
            if (detect_tof_distance(curr_detect_dist, 286, 335)):
                is_detect = True
                count = 2
            #第三块木块
            if (detect_tof_distance(curr_detect_dist, 336, 350)):
                is_detect = True
                count = 3
            #第四块木块
            if (detect_tof_distance(curr_detect_dist, 351, 361)):
                is_detect = True
                count = 4
        else:
            print("Detect out of range!")
            exit(0)

        if (is_detect):
            if (-1 <= count and 4 >= count):
                if (-1 == count):
                    down_z = default_down_z_postion
                elif (1 == count):
                    down_z -= down_z_offset / 2
                else:
                    down_z -= (count * down_z_offset) - (down_z_offset / 2)
                print("Current Z axis point1:", down_z)
                # initial_point(speed, ms)
                # pickup(down_z, speed, ms)
                # move_to_slide_rail(speed, ms)
                # is_detect = False
                # break
    print("Current Z axis point2:", down_z)            
    initial_point(speed, ms)
    pickup(down_z, speed, ms)
    move_to_slide_rail(speed, ms)
    is_detect = False


motion_plan(90, 2)

# while True:
#     print('start plan....')
#     time.sleep(2)
#     # print(aikit.get_tof_dist())
#     motion_plan(90, 2)
#     print('end plan.......')
#     time.sleep(2)
    

 
    
    
    
