#!/usr/bin/env python3
import platform

import cv2
import numpy as np
import time
import os
import sys

from pymycobot.mycobot import MyCobot

IS_CV_4 = cv2.__version__[0] == '4'
__version__ = "1.0"


class Object_detect():

    def __init__(self, camera_x=256, camera_y=0):
        # inherit the parent class
        super(Object_detect, self).__init__()
        # declare mycobot320
        self.mc = None

        # 移动角度
        self.move_angles = [
            [0.61, 45.87, -92.37, -41.3, 89.56, 9.58],  # init the point
            [18.8, -7.91, -54.49, -23.02, 89.56, -14.76],  # point to grab
            [17.22, -5.27, -52.47, -25.75, 89.73, -0.26],
        ]

        # 移动坐标
        self.move_coords = [
            [154.8, -210.6, 217.4, -175.54, -3.46, -122.78],  # D Sorting area
            [280.4, -201.0, 249.2, -157.32, -0.83, -110.8],  # C Sorting area
            [136.3, 221.4, 244.6, -171.64, 0.48, -11.7],  # A Sorting area
            [-13.6, 218.7, 225.3, -177.09, 0.84, 27.55],  # B Sorting area
        ]

        # which robot: USB* is m5; ACM* is wio; AMA* is raspi
        self.robot_m5 = os.popen("ls /dev/ttyUSB*").readline()[:-1]
        self.robot_wio = os.popen("ls /dev/ttyACM*").readline()[:-1]
        self.robot_raspi = os.popen("ls /dev/ttyAMA*").readline()[:-1]
        self.robot_jes = os.popen("ls /dev/ttyTHS1").readline()[:-1]
        self.raspi = False
        if "dev" in self.robot_m5:
            self.Pin = [2, 5]
        elif "dev" in self.robot_wio:
            # self.Pin = [20, 21]
            self.Pin = [2, 5]

            # for i in self.move_coords:
            #     i[2] -= 20
        elif "dev" in self.robot_raspi or "dev" in self.robot_jes:
            import RPi.GPIO as GPIO
            GPIO.setwarnings(False)
            self.GPIO = GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(20, GPIO.OUT)
            GPIO.setup(21, GPIO.OUT)
            GPIO.output(20, 1)
            GPIO.output(21, 1)
            self.raspi = True
        if self.raspi:
            self.gpio_status(False)

        # choose place to set cube 选择放置立方体的地方
        self.color = 0
        # parameters to calculate camera clipping parameters 计算相机裁剪参数的参数
        self.x1 = self.x2 = self.y1 = self.y2 = 0
        # set cache of real coord 设置真实坐标的缓存
        self.cache_x = self.cache_y = 0
        # set color HSV
        self.HSV = {
            "yellow": [np.array([11, 85, 70]), np.array([59, 255, 245])],
            # "yellow": [np.array([22, 93, 0]), np.array([45, 255, 245])],
            "red": [np.array([0, 43, 46]), np.array([8, 255, 255])],
            "green": [np.array([35, 43, 35]), np.array([90, 255, 255])],
            "blue": [np.array([78, 43, 46]), np.array([110, 255, 255])],
            "cyan": [np.array([78, 43, 46]), np.array([99, 255, 255])],
        }

        # use to calculate coord between cube and mycobot320
        # 用于计算立方体和 mycobot 之间的坐标
        self.sum_x1 = self.sum_x2 = self.sum_y2 = self.sum_y1 = 0
        # The coordinates of the grab center point relative to the mycobot320
        # 抓取中心点相对于 mycobot 的坐标
        self.camera_x, self.camera_y = camera_x, camera_y
        # The coordinates of the cube relative to the mycobot320
        # 立方体相对于 mycobot 的坐标
        self.c_x, self.c_y = 0, 0
        # The ratio of pixels to actual values
        # 像素与实际值的比值
        self.ratio = 0

        # Get ArUco marker dict that can be detected.
        # 获取可以检测到的 ArUco 标记字典。
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        # Get ArUco marker params. 获取 ArUco 标记参数
        self.aruco_params = cv2.aruco.DetectorParameters_create()

    # pump_control pi
    def gpio_status(self, flag):
        if flag:
            self.GPIO.output(20, 0)
            self.GPIO.output(21, 0)
        else:
            self.GPIO.output(20, 1)
            self.GPIO.output(21, 1)

    # 开启吸泵 m5
    def pump_on(self):
        # 让2号位工作
        self.mc.set_basic_output(2, 0)
        # 让5号位工作
        self.mc.set_basic_output(5, 0)

    # 停止吸泵 m5
    def pump_off(self):
        # 让2号位停止工作
        self.mc.set_basic_output(2, 1)
        # 让5号位停止工作
        self.mc.set_basic_output(5, 1)

    # Grasping motion
    def move(self, x, y, color):
        """
        Functions that control a series of movements of the robotic arm and grab blocks
        :param x: The x-axis coordinate of the block relative to the robot arm
        :param y: The y-axis coordinate of the block relative to the robot arm
        :param color: The index of where the block is placed(0-C,1-D,2-A,3-B)
        :return: None
        """
        # send Angle to move mycobot320
        print(color)
        print('x,y:', round(x, 2), round(y, 2))
        self.mc.send_angles(self.move_angles[2], 50)
        time.sleep(3)

        # send coordinates to move mycobot
        self.mc.send_coords([x, y, 250, -173.84, -0.14, -74.37], 50, 0)
        time.sleep(2.5)

        self.mc.send_coords([x, y, 143, -173.84, -0.14, -74.37], 50, 0)
        time.sleep(3)

        # open pump
        if "dev" in self.robot_m5 or "dev" in self.robot_wio:
            self.pump_on()
        elif "dev" in self.robot_raspi or "dev" in self.robot_jes:
            self.gpio_status(True)
        time.sleep(1.5)

        tmp = []
        while True:
            if not tmp:
                tmp = self.mc.get_angles()
            else:
                break
        time.sleep(0.5)

        # print(tmp)
        self.mc.send_angles([tmp[0], -0.71, -54.49, -23.02, 89.56, tmp[5]],
                            25)  # [18.8, -7.91, -54.49, -23.02, -0.79, -14.76]
        time.sleep(3)

        self.mc.send_coords(self.move_coords[color], 50, 0)
        # self.pub_marker(self.move_coords[color][0]/1000.0, self.move_coords[color]
        #                 [1]/1000.0, self.move_coords[color][2]/1000.0)
        time.sleep(3)

        # close pump
        if "dev" in self.robot_m5 or "dev" in self.robot_wio:
            self.pump_off()
        elif "dev" in self.robot_raspi or "dev" in self.robot_jes:
            self.gpio_status(False)
        time.sleep(5)

        self.mc.send_angles(self.move_angles[0], 50)
        time.sleep(4.5)

    # decide whether grab cube 决定是否抓取立方体
    def decide_move(self, x, y, color):
        # print(x, y, self.cache_x, self.cache_y)
        # detect the cube status move or run 检测立方体状态移动或运行
        if (abs(x - self.cache_x) + abs(y - self.cache_y)) / 2 > 5:  # mm
            self.cache_x, self.cache_y = x, y
            return
        else:
            self.cache_x = self.cache_y = 0
            # 调整吸泵吸取位置，y增大,向左移动;y减小,向右移动;x增大,前方移动;x减小,向后方移动
            self.move(x, y, color)

    # init mycobot320
    def run(self):
        """
        Initialize the robot object
        :return: None
        """
        if "dev" in self.robot_wio:
            self.mc = MyCobot(self.robot_wio, 115200)
        elif "dev" in self.robot_m5:
            self.mc = MyCobot(self.robot_m5, 115200)
        elif "dev" in self.robot_raspi:
            self.mc = MyCobot(self.robot_raspi, 115200)
        if not self.raspi:
            self.pub_pump(False, self.Pin)
        self.mc.send_angles([0.61, 45.87, -92.37, -41.3, 89.56, 9.58], 20)
        time.sleep(2.5)

    # draw aruco
    def draw_marker(self, img, x, y):
        # draw rectangle on img 在 img 上绘制矩形
        cv2.rectangle(
            img,
            (x - 20, y - 20),
            (x + 20, y + 20),
            (0, 255, 0),
            thickness=2,
            lineType=cv2.FONT_HERSHEY_COMPLEX,
        )
        # add text on rectangle
        cv2.putText(img, "({},{})".format(x, y), (x, y),
                    cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (243, 0, 0), 2, )

    # get points of two aruco 获得两个 aruco 的点位
    def get_calculate_params(self, img):
        """
        Get the center coordinates of two ArUco codes in the image
        :param img: Image, in color image format.
        :return: If two ArUco codes are detected, returns the coordinates of the centers of the two codes; otherwise returns None.
        """
        # Convert the image to a gray image 将图像转换为灰度图像
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
            print('center_x1,x2:', center_x1, center_x2, center_y1, center_y2)
            return center_x1, center_x2, center_y1, center_y2
        else:
            return None

    # set camera clipping parameters 设置相机裁剪参数 
    def set_cut_params(self, x1, y1, x2, y2):
        self.x1 = int(x1)
        self.y1 = int(y1)
        self.x2 = int(x2)
        self.y2 = int(y2)

    # set parameters to calculate the coords between cube and mycobot320
    # 设置参数以计算立方体和 mycobot 之间的坐标
    def set_params(self, c_x, c_y, ratio):
        self.c_x = c_x
        self.c_y = c_y
        self.ratio = 320.0 / ratio

    # calculate the coords between cube and mycobot320
    # 计算立方体和 mycobot 之间的坐标
    def get_position(self, x, y):
        return ((y - self.c_y) * self.ratio + self.camera_x), ((x - self.c_x) * self.ratio + self.camera_y)

    """
    Calibrate the camera according to the calibration parameters.
    Enlarge the video pixel by 1.5 times, which means enlarge the video size by 1.5 times.
    If two ARuco values have been calculated, clip the video.
    """

    def transform_frame(self, frame):
        # enlarge the image by 1.5 times
        fx = 1.5
        fy = 1.5
        frame = cv2.resize(frame, (0, 0), fx=fx, fy=fy,
                           interpolation=cv2.INTER_CUBIC)
        if self.x1 != self.x2:
            # the cutting ratio here is adjusted according to the actual situation
            frame = frame[int(self.y2 * 0.78):int(self.y1 * 1.1),
                    int(self.x1 * 0.86):int(self.x2 * 1.08)]
        return frame

    # detect cube color
    def color_detect(self, img):
        """
        Color recognition detection function
        :param img: Image information read under the camera
        :return: image coordinates
        """
        # set the arrangement of color'HSV
        x = y = 0
        for mycolor, item in self.HSV.items():
            # print("mycolor:",mycolor)
            redLower = np.array(item[0])
            redUpper = np.array(item[1])

            # transfrom the img to model of gray 将图像转换为灰度模型
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
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
                    # calculate the real coordinates of mycobot320 relative to the target
                    #  计算 mycobot 相对于目标的真实坐标

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


def main():
    """
    Turn on the camera and call the detection function to get the detection result and move the robot arm
    :return:
    """
    # open the camera
    if platform.system() == "Windows":
        cap_num = 1
        cap = cv2.VideoCapture(cap_num, cv2.CAP_DSHOW)
        if not cap.isOpened():
            cap.open(1)
    elif platform.system() == "Linux":
        cap_num = 0
        cap = cv2.VideoCapture(cap_num, cv2.CAP_V4L)
        if not cap.isOpened():
            cap.open()
    # init a class of Object_detect
    detect = Object_detect()
    # init mycobot320
    detect.run()

    _init_ = 20
    init_num = 0
    nparams = 0
    num = 0
    real_sx = real_sy = 0
    while cv2.waitKey(1) < 0:
        # read camera
        _, frame = cap.read()
        # deal img
        frame = detect.transform_frame(frame)
        # calculate the parameters of camera clipping 计算相机裁剪的参数
        if init_num < 20:
            calculate_params = detect.get_calculate_params(frame)
            if calculate_params is not None:
                x1, x2, y1, y2 = calculate_params
                detect.draw_marker(frame, x1, y1)
                detect.draw_marker(frame, x2, y2)
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

        # calculate params of the coords between cube and mycobot320 计算立方体和 mycobot 之间坐标的参数
        if nparams < 10:
            calculate_params = detect.get_calculate_params(frame)
            if calculate_params is not None:
                x1, x2, y1, y2 = calculate_params
                detect.draw_marker(frame, x1, y1)
                detect.draw_marker(frame, x2, y2)
                detect.sum_x1 += x1
                detect.sum_x2 += x2
                detect.sum_y1 += y1
                detect.sum_y2 += y2
                nparams += 1
                continue
        elif nparams == 10:
            nparams += 1
            # calculate and set params of calculating real coord between cube and mycobot320
            # 计算和设置计算立方体和mycobot之间真实坐标的参数
            detect.set_params(
                (detect.sum_x1 + detect.sum_x2) / 20.0,
                (detect.sum_y1 + detect.sum_y2) / 20.0,
                abs(detect.sum_x1 - detect.sum_x2) / 10.0 +
                abs(detect.sum_y1 - detect.sum_y2) / 10.0
            )
            print("ok")
            continue
        else:
            # get detect result 获取检测结果
            detect_result = detect.color_detect(frame)
            if detect_result is not None:
                x, y = detect_result
                # calculate real coord between cube and mycobot320 计算立方体和 mycobot 之间的真实坐标
                real_x, real_y = detect.get_position(x, y)
                # print('real_x',round(real_x, 3),round(real_y, 3))
                if num == 20:
                    detect.decide_move(real_sx / 20.0, real_sy / 20.0, detect.color)
                    num = real_sx = real_sy = 0
                else:
                    num += 1
                    real_sy += real_y
                    real_sx += real_x

        cv2.imshow("figure", frame)

        # close the window
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            sys.exit()


if __name__ == '__main__':
    main()