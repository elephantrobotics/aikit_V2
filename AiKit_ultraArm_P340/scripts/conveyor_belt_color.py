# encoding:utf-8

# ultrArm P340 传送带套装v1.0
# 此处有距离传感器的使用

import cv2
from pymycobot.ultraArm import ultraArm
import numpy as np
import time
from threading import Thread
import serial
import serial.tools.list_ports
from megaAiKit import megaAikit

IS_CV_4 = cv2.__version__[0] == '4'

# 距离传感器处的默认初始点坐标值
# z轴
default_down_z_postion = 126.47
down_x = -6.91
down_y = 252.1
# 木块大小为40 mm，故木块与木块之间的偏移量为40 mm
down_z_offset = 40

# 传感器上下距离范围
lower_distance_limit = 255 # 245
upper_distance_limit = 375 # 374

is_detect = False

# 动态获取串口信息
plist = [
    str(x).split(" - ")[0].strip() for x in serial.tools.list_ports.comports()
]

# 初始化距离传感器串口
kit = megaAikit(plist[1])
print(plist[0], plist[1])

tof_thread = None

# 获取传感器距离的工作接口，延时必须给 0.5
class ThreadTof(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.running = True
        self.dist = kit.get_tof_distance()

    def run(self):
        while True:
            if self.running:
                self.dist = kit.get_tof_distance()
                # print(self.dist)
            else:
                break
            if (20 > self.dist):
                continue
            time.sleep(0.5)

    def distance(self):
        return self.dist

    def set_running_flag(self, flag):
        self.running = flag

    def get_running_flag(self):
        return self.running


class Object_detect():
    def __init__(self, camera_x=263, camera_y=-108):  # 252,-114

        # initialize ultraArm
        self.ua = ultraArm(plist[0], 115200)

        self.ua.go_zero()
        # choose place to set cube
        self.color = 0
        # parameters to calculate camera clipping parameters
        self.x1 = self.x2 = self.y1 = self.y2 = 0
        # set cache of real coord
        self.cache_x = self.cache_y = 0
        # set color HSV
        self.HSV = {
            # "yellow": [np.array([11, 85, 70]), np.array([59, 255, 245])],
            "yellow": [np.array([20, 100, 100]), np.array([30, 255, 255])],
            "red": [np.array([0, 43, 46]), np.array([8, 255, 255])],
            "green": [np.array([35, 43, 35]), np.array([90, 255, 255])],
            "blue": [np.array([78, 43, 46]), np.array([110, 255, 255])],
            "cyan": [np.array([78, 43, 46]), np.array([99, 255, 255])],
        }
        # use to calculate coord between cube and ultraArm
        self.sum_x1 = self.sum_x2 = self.sum_y2 = self.sum_y1 = 0
        # The coordinates of the grab center point relative to the ultraArm
        self.camera_x, self.camera_y = camera_x, camera_y
        # The coordinates of the cube relative to the ultraArm
        self.c_x, self.c_y = 0, 0
        # The ratio of pixels to actual values
        self.ratio = self.ratio2 = 0
        # Get ArUco marker dict that can be detected.
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        # Get ArUco marker params.
        self.aruco_params = cv2.aruco.DetectorParameters_create()

    # 控制吸泵      
    def pub_pump(self, flag):
        if flag:
            self.ua.set_gpio_state(0)
        else:
            self.ua.set_gpio_state(1)

    # Grasping motion
    
    def move(self, x, y, color):
        global tof_thread
        print('x,y:', round(x, 2), round(y, 2))
        print('color index:', color)
        # 移动角度
        move_angles = [
            [0.0, 0.0, 0.0],  # init the point
            [25.55, 0.0, 15.24],  # 第二初始点
            [-33.8, -3.44, -2.86, 0.0],  # 识别抓取前物块上方的点
            [-33.8, 10.89, 29.79, 0.0],  # 抓取物块的点
            [-80.21, 0.0, 9.74, 0.0],  # 识别抓取之后的缓冲点
            [0.0, 14.32, 0.0],  # point to grab
        ]

        # 移动坐标
        move_coords = [
            [99.13, -226.06, 37.44, -65.32], # A区域
            [46.88, -230.44, 26.34, -78.5],  # B区域
            [-9.23, -234.98, 26.34, -92.25],  # C区域
            [-64.36, -233.34, 32.15, -105.42],  # D区域
        ]

        # move to ultraArm
        # 到达物块上方
        # self.ua.set_angles(move_angles[2], 50)
        self.ua.set_coords([x, y, 130], 60)
        self.ua.sleep(1.5)

        # 抓取物块
        # self.ua.set_angles(move_angles[3], 30)
        # self.ua.set_coords([200.32, -134.1, 58.1, -33.8], 60)
        self.ua.set_coords([x, y, 59], 60)
        self.ua.sleep(2)
        # open pump
        self.pub_pump(True)
        time.sleep(2)
        self.ua.set_angles(move_angles[2], 60)
        time.sleep(3)
        self.ua.set_angles(move_angles[4], 60)
        time.sleep(3)
        self.ua.set_coords(move_coords[color], 60)  # 0红1绿2蓝3黄
        time.sleep(4)

        # close pump
        self.pub_pump(False)
        time.sleep(2)
        self.ua.set_angles(move_angles[4], 60)
        time.sleep(3)
        self.ua.set_angles(move_angles[1], 60)
        time.sleep(4)
        self.pump_to_send()

        tof_thread.set_running_flag(True)

    # decide whether grab cube
    def decide_move(self, x, y, color):

        # print(x, y, self.cache_x, self.cache_y)
        # detect the cube status move or run
        if (abs(x - self.cache_x) + abs(y - self.cache_y)) / 2 > 5:  # mm
            self.cache_x, self.cache_y = x, y
            return
        else:
            self.cache_x = self.cache_y = 0
            self.move(x, y, color)

    # 从右边获得物块并放到传送带上
    def pump_to_send(self):
        global tof_thread
        global is_detect
        pump_angles = [
            [91.57, 7.6, 0.16],  # first_point_anges
            [91.57, 0, 0],  # leave_to_default_angles
            [38, 10, -3],  # slide_rail_up_angles
            [38, 15, 23],  # slide_rail_down_angles
        ]
        # print("start thread")
        tof_thread = ThreadTof()
        tof_thread.daemon = True
        tof_thread.start()
        time.sleep(2)
        current_detect_dist = -1
        down_z = None

        while 1:
            down_z = default_down_z_postion
            count = -1
            is_detect = False
            if tof_thread.get_running_flag():
                current_detect_dist = tof_thread.distance()
                # print("3:",current_detect_dist)
                time.sleep(0.5)
            print('Current detected distance:', current_detect_dist)
            if lower_distance_limit <= current_detect_dist <= upper_distance_limit:
                # 停止传感器测距
                # 最上方木块
                if 255 <= current_detect_dist <= 310:
                    is_detect = True
                    count = 1
                    break
                elif 311 <= current_detect_dist <= 340:
                    is_detect = True
                    count = 2
                    break
                elif 341 <= current_detect_dist <= 360:
                    is_detect = True
                    count = 3
                    break
                elif 361 <= current_detect_dist <= 375:
                    is_detect = True
                    count = 4
                    break
            else:
                print('Detect out of range!')
                # exit(0)
                tof_thread.set_running_flag(True)
                continue

        if is_detect:
            tof_thread.set_running_flag(False)

            if -1 <= count <= 4:
                # if count == -1:
                #     down_z = default_down_z_postion
                if count == 1:
                    down_z -= down_z_offset / 2
                else:
                    down_z -= (count * down_z_offset) - (down_z_offset / 2)
        print('Current Z axis point:', down_z)
        # 初始点
        self.ua.set_angles(pump_angles[0], 60)
        # 吸取的上下动作，吸取时打开吸泵
        # 下
        self.ua.set_coords([down_x, down_y, down_z], 60)
        time.sleep(2)
        # 打开吸泵
        self.ua.set_gpio_state(0)
        time.sleep(2)
        # 上
        self.ua.set_angles(pump_angles[1], 60)
        time.sleep(2)
        # 移动到指定默认的滑轨上方位置
        self.ua.set_angles(pump_angles[2], 60)
        time.sleep(2)
        # 下放到滑轨的位置
        self.ua.set_angles(pump_angles[3], 60)
        time.sleep(3)
        # 关闭吸泵
        self.ua.set_gpio_state(1)
        time.sleep(2)
        
        # 打开传送带
        for i in range(5):
            try:
                kit.control_conveyor_by_switch(1, 80)
                break
            except:
                if i == 2:
                    exit(0)
                pass
        time.sleep(3.5)
        # 关闭传送带
        kit.control_conveyor_by_switch(0, 80)
        time.sleep(2)
        kit.control_conveyor_by_switch(0, 80)
        # 开启传感器测距
        # tof_thread.set_running_flag(True)

        is_detect = False

    # init ultraArm
    def run(self):
        self.pump_to_send()

    # draw aruco
    def draw_marker(self, img, x, y):
        # draw rectangle on img
        cv2.rectangle(
            img,
            (x - 20, y - 20),
            (x + 20, y + 20),
            (0, 255, 0),
            thickness=2,
            lineType=cv2.FONT_HERSHEY_COMPLEX,  # 字体
        )
        # add text on rectangle
        cv2.putText(img, "({},{})".format(x, y), (x, y), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (243, 0, 0), 2, )

        # get points of two aruco

    def get_calculate_params(self, img):
        # Convert the image to a gray image
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Detect ArUco marker.
        corners, ids, rejectImaPoint = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )

        """
        Two Arucos must be present in the picture and in the same order.
        There are two Arucos in the Corners, and each aruco contains the pixels of its four corners.
        Determine the center of the aruco by the four corners of the aruco.
        """
        if corners is not None and ids is not None and len(corners) == len(ids) == 2 and ids[0] != 1:
            center_x1, center_y1 = np.mean(corners[0][0], axis=0).astype(int)
            center_x2, center_y2 = np.mean(corners[1][0], axis=0).astype(int)
            # print('ce', center_x1, center_x2, center_y1, center_y2)
            return center_x1, center_x2, center_y1, center_y2
        else:
            return None

    # set camera clipping parameters    
    def set_cut_params(self, x1, y1, x2, y2):
        self.x1 = int(x1)
        self.y1 = int(y1) + 360
        self.x2 = int(x2)
        self.y2 = int(y2)
        # print(self.x1, self.y1, self.x2, self.y2)

    # set parameters to calculate the coords between cube and ultraArm   
    def set_params(self, c_x, c_y, ratio):
        self.c_x = c_x
        self.c_y = c_y
        self.ratio = 98.0 / ratio

    # calculate the coords between cube and ultraArm
    def get_position(self, x, y):
        # return  ((y - self.c_y)*self.ratio + self.camera_x), ((x - self.c_x)*self.ratio + self.camera_y)
        # self.stop_or_run(True)
        return ((x - self.c_y) * self.ratio + self.camera_x), ((-y + self.c_x) * self.ratio + self.camera_y)

    """
    Calibrate the camera according to the calibration parameters.
    Enlarge the video pixel by 1.5 times, which means enlarge the video size by 1.5 times.
    If two ARuco values have been calculated, clip the video.
    """

    def transform_frame(self, frame):
        # enlarge the image by 1.5 times
        fx = 1.5
        fy = 1.5
        frame = cv2.resize(frame, (0, 0), fx=fx, fy=fy, interpolation=cv2.INTER_CUBIC)
        if self.x1 != self.x2:
            # the cutting ratio here is adjusted according to the actual situation
            frame = frame[int(self.y2 * 0.2):int(self.y1 * 0.7), int(self.x1 * 0.8):int(self.x2 * 1.1)]
        return frame

    # detect cube color
    def color_detect(self, img):

        # set the arrangement of color'HSV
        x = y = 0
        # transfrom the img to model of gray 将图像转换为灰度模型
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        for mycolor, item in self.HSV.items():
            # print("mycolor:",mycolor)
            redLower = np.array(item[0])
            redUpper = np.array(item[1])

            # print("hsv",hsv)

            # wipe off all color expect color in range 擦掉所有颜色期望范围内的颜色
            mask = cv2.inRange(hsv, item[0], item[1])

            # a etching operation on a picture to remove edge roughness
            # 对图片进行蚀刻操作以去除边缘粗糙度
            erosion = cv2.erode(mask, np.ones((1, 1), np.uint8), iterations=2)

            # the image for expansion operation, its role is to deepen the color depth in the picture
            # 用于扩展操作的图像，其作用是加深图片中的颜色深度
            dilation = cv2.dilate(erosion, np.ones(
                (1, 1), np.uint8), iterations=2)

            # adds pixels to the image 向图像添加像素
            target = cv2.bitwise_and(img, img, mask=dilation)

            # the filtered image is transformed into a binary image and placed in binary
            # 将过滤后的图像转换为二值图像并放入二值
            ret, binary = cv2.threshold(dilation, 127, 255, cv2.THRESH_BINARY)

            # get the contour coordinates of the image, where contours is the coordinate value, here only the contour is detected
            # 获取图像的轮廓坐标，其中contours为坐标值，这里只检测轮廓
            contours, hierarchy = cv2.findContours(
                dilation, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) > 0:
                # do something about misidentification
                boxes = [
                    box
                    for box in [cv2.boundingRect(c) for c in contours]
                    if min(img.shape[0], img.shape[1]) / 10
                       < min(box[2], box[3])
                       < min(img.shape[0], img.shape[1]) / 1
                ]
                if boxes:
                    for box in boxes:
                        x, y, w, h = box
                    # find the largest object that fits the requirements 找到符合要求的最大对象
                    c = max(contours, key=cv2.contourArea)
                    # get the lower left and upper right points of the positioning object
                    # 获取定位对象的左下和右上点
                    x, y, w, h = cv2.boundingRect(c)
                    # locate the target by drawing rectangle 通过绘制矩形来定位目标
                    cv2.rectangle(img, (x, y), (x + w, y + h), (153, 153, 0), 2)
                    # calculate the rectangle center 计算矩形中心
                    x, y = (x * 2 + w) / 2, (y * 2 + h) / 2
                    # calculate the real coordinates of ultraArm P340 relative to the target
                    #  计算 ultraArm 相对于目标的真实坐标

                    if mycolor == "yellow":

                        self.color = 3
                        break

                    elif mycolor == "red":
                        self.color = 0
                        break

                    elif mycolor == "cyan":
                        self.color = 2
                        break

                    elif mycolor == "blue":
                        self.color = 2
                        break
                    elif mycolor == "green":
                        self.color = 1
                        break

        # 判断是否正常识别
        if abs(x) + abs(y) > 0:
            return x, y
        else:
            return None


def runs():
    # open the camera
    cap_num = 1
    cap = cv2.VideoCapture(cap_num)
    if not cap.isOpened():
        cap.open()

    # init a class of Object_detect
    detect = Object_detect()

    # init ultraArm
    detect.run()

    _init_ = 20  # 
    init_num = 0
    nparams = 0
    num = 0
    real_sx = real_sy = 0
    while cv2.waitKey(1) < 0:
        # read camera
        _, frame = cap.read()
        # deal img
        frame = detect.transform_frame(frame)

        if _init_ > 0:
            _init_ -= 1
            continue
        # calculate the parameters of camera clipping
        if init_num < 20:
            if detect.get_calculate_params(frame) is None:
                cv2.imshow("figure", frame)
                continue
            else:
                x1, x2, y1, y2 = detect.get_calculate_params(frame)
                detect.sum_x1 += x1
                detect.sum_x2 += x2
                detect.sum_y1 += y1
                detect.sum_y2 += y2
                init_num += 1
                continue
        elif init_num == 20:
            detect.set_cut_params(
                (detect.sum_x1) / 20.0,
                (detect.sum_y1) / 20.0,
                (detect.sum_x2) / 20.0,
                (detect.sum_y2) / 20.0,
            )
            detect.sum_x1 = detect.sum_x2 = detect.sum_y1 = detect.sum_y2 = 0
            init_num += 1
            continue

        # calculate params of the coords between cube and ultraArm
        if nparams < 10:
            if detect.get_calculate_params(frame) is None:
                cv2.imshow("figure", frame)
                continue
            else:
                x1, x2, y1, y2 = detect.get_calculate_params(frame)
                detect.sum_x1 += x1
                detect.sum_x2 += x2
                detect.sum_y1 += y1
                detect.sum_y2 += y2
                nparams += 1
                continue
        elif nparams == 10:
            nparams += 1
            # calculate and set params of calculating real coord between cube and ultraArm
            detect.set_params(
                (detect.sum_x1 + detect.sum_x2) / 20.0,
                (detect.sum_y1 + detect.sum_y2) / 20.0,
                abs(detect.sum_x1 - detect.sum_x2) / 10.0 + abs(detect.sum_y1 - detect.sum_y2) / 10.0
            )
            # print(abs(detect.sum_x1 - detect.sum_x2) / 10.0 + abs(detect.sum_y1 - detect.sum_y2) / 10.0)  # ratio
            print("ok")
            continue

        detect_result = detect.color_detect(frame)

        if detect_result is None:
            # detect.stop_or_run(False)
            cv2.imshow("figure", frame)
            continue
        else:
            x, y = detect_result
            # calculate real coord between cube and ultraArm
            real_x, real_y = detect.get_position(x, y)
            if num == 20:
                detect.decide_move(real_sx / 20.0, real_sy / 20.0, detect.color)
                num = real_sx = real_sy = 0

            else:
                num += 1
                real_sy += real_y
                real_sx += real_x
        # print(frame)
        cv2.imshow("figure", frame)


if __name__ == "__main__":
    t = Thread(target=runs)
    t.start()
    # runs()
