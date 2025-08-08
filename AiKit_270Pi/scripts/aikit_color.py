#!/usr/bin/env python3
import platform
import sys
import time
import traceback

import RPi.GPIO as GPIO
import cv2
import numpy as np
from pymycobot.mecharm270 import MechArm270

from offset_utils import load_offset_from_txt

IS_CV_4 = cv2.__version__[0] == '4'
__version__ = "1.0"


offset_path = '/home/er/AiKit_UI/libraries/offset/mechArm 270 for Pi_color.txt'

camera_x, camera_y, camera_z = load_offset_from_txt(offset_path)


class Object_detect():

    def __init__(self, camera_x=camera_x, camera_y=camera_y):
        # inherit the parent class
        super(Object_detect, self).__init__()
        # declare mecharm 270
        self.mc = None
        # 移动角度
        self.move_angles = [
            [0, 0, 0, 0, 90, 0],  # init the point
            [-33.31, 2.02, -10.72, -0.08, 95, -54.84],  # point to grab
        ]

        self.new_move_coords_to_angles = [
            [-52.64, 35.06, -39.63, -2.28, 82.35, 55.45],  # D
            [-34.18, 60.9, -69.08, -0.96, 70.04, 88.06],  # C
            [32.34, 58.35, -62.13, 4.3, 61.52, 15.64],  # A
            [55.19, 42.71, -46.4, -0.96, 84.19, 15.99]  # B
        ]

        # choose place to set cube 选择放置立方体的地方
        self.color = 0
        # parameters to calculate camera clipping parameters 计算相机裁剪参数的参数
        self.x1 = self.x2 = self.y1 = self.y2 = 0
        # set cache of real coord 设置真实坐标的缓存
        self.cache_x = self.cache_y = 0
        # set color HSV
        self.HSV = {
            # "yellow": [np.array([11, 85, 70]), np.array([59, 255, 245])],
            # "yellow": [np.array([22, 93, 0]), np.array([45, 255, 245])],
            "yellow": [np.array([26, 43, 46]), np.array([34, 255, 255])],
            # "red": [np.array([0, 43, 46]), np.array([8, 255, 255])],
            "red": [np.array([170, 100, 100]), np.array([179, 255, 255])],
            "green": [np.array([35, 43, 35]), np.array([90, 255, 255])],
            "blue": [np.array([100, 43, 46]), np.array([124, 255, 255])],
            "cyan": [np.array([78, 43, 46]), np.array([99, 255, 255])],
        }

        # use to calculate coord between cube and mecharm 270
        # 用于计算立方体和 mycobot 之间的坐标
        self.sum_x1 = self.sum_x2 = self.sum_y2 = self.sum_y1 = 0
        # The coordinates of the grab center point relative to the mecharm270
        # 抓取中心点相对于 mycobot 的坐标
        self.camera_x, self.camera_y = camera_x, camera_y
        # The coordinates of the cube relative to the mecharm270
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

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(20, GPIO.OUT)
        GPIO.setup(21, GPIO.OUT)

        self.camera_z = camera_z

    # 开启吸泵
    def pump_on(self):
        GPIO.output(20, 0)
        time.sleep(0.05)

    # 停止吸泵
    def pump_off(self):
        GPIO.output(20, 1)
        time.sleep(0.05)
        # 打开泄气阀门
        GPIO.output(21, 0)
        time.sleep(1)
        GPIO.output(21, 1)
        time.sleep(0.05)

    def check_position(self, data, ids, max_same_data_count=50):
        """
        循环检测是否到位某个位置
        :param data: 角度或者坐标
        :param ids: 角度-0，坐标-1
        :return:
        """
        try:
            same_data_count = 0
            last_data = None
            start_time = time.time()
            while True:
                # 超时检测
                if (time.time() - start_time) >= 5:
                    break
                res = self.mc.is_in_position(data, ids)
                # print('res', res, data)
                if data == last_data:
                    same_data_count += 1
                else:
                    same_data_count = 0

                last_data = data
                # print('count:', same_data_count)
                if res == 1 or same_data_count >= max_same_data_count:
                    break
                time.sleep(0.1)
        except Exception as e:
            e = traceback.format_exc()
            print(e)

    # Grasping motion
    def move(self, x, y, color):
        # send Angle to move mecharm270
        print(color, 'real_x: ', x, 'real_y: ', y)
        if x > 206:
            print('The object is too far away and the target point cannot be reached. Please reposition the identifiable object!')
            return
        self.mc.send_angles(self.move_angles[0], 50)
        self.check_position(self.move_angles[0], 0)

        # send coordinates to move mycobot
        self.mc.send_coords([x, y, 150, -176.1, 2.4, -125.1], 70, 1)  # usb :rx,ry,rz -173.3, -5.48, -57.9
        self.mc.send_coords([x, y, self.camera_z, -176.1, 2.4, -125.1], 70, 1)  # -178.77, -2.69, 40.15     pi
        # self.check_position([x, y, 115, -176.1, 2.4, -125.1], 1)
        while self.mc.is_moving():
            time.sleep(0.2)
        time.sleep(1)
        if self.mc.is_in_position([x, y, self.camera_z, -176.1, 2.4, -125.1], 1) != 1:
            self.mc.send_coords([x, y, self.camera_z, -176.1, 2.4, -125.1], 70, 1)
        time.sleep(1)
        # open pump
        self.pump_on()
        time.sleep(1.5)

        tmp = []
        while True:
            if not tmp:
                tmp = self.mc.get_angles()
            else:
                break
        time.sleep(0.5)

        # print(tmp)
        self.mc.send_angles([tmp[0], 17.22, -32.51, tmp[3], 97, tmp[5]],
                            50)  # [18.8, -7.91, -54.49, -23.02, -0.79, -14.76]
        self.check_position([tmp[0], 17.22, -32.51, tmp[3], 97, tmp[5]], 0)

        self.mc.send_angles(self.new_move_coords_to_angles[color], 50)
        self.check_position(self.new_move_coords_to_angles[color], 0)

        # close pump
        self.pump_off()
        time.sleep(2)

        self.mc.send_angles(self.move_angles[1], 50)
        self.check_position(self.move_angles[1], 0)

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
            self.move(round(x, 2), round(y, 2), color)

    # init mecharm270
    def run(self):
        self.mc = MechArm270('/dev/ttyAMA0', 1000000)
        if self.mc.get_fresh_mode() != 0:
            self.mc.set_fresh_mode(0)
        self.pump_off()
        self.mc.send_angles([-33.31, 2.02, -10.72, -0.08, 95, -54.84], 50)
        self.check_position([-33.31, 2.02, -10.72, -0.08, 95, -54.84], 0)

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
        if len(corners) > 0:
            if ids is not None:
                if len(corners) <= 1 or ids[0] == 1:
                    return None
                x1 = x2 = y1 = y2 = 0
                point_11, point_21, point_31, point_41 = corners[0][0]
                # print('point_11:', point_11)
                # print('point_21:', point_21)
                # print('point_31:', point_31)
                # print('point_41:', point_41)
                x1, y1 = int((point_11[0] + point_21[0] + point_31[0] + point_41[0]) / 4.0), int(
                    (point_11[1] + point_21[1] + point_31[1] + point_41[1]) / 4.0)
                point_1, point_2, point_3, point_4 = corners[1][0]
                x2, y2 = int((point_1[0] + point_2[0] + point_3[0] + point_4[0]) / 4.0), int(
                    (point_1[1] + point_2[1] + point_3[1] + point_4[1]) / 4.0)

                return x1, x2, y1, y2
        return None

    # set camera clipping parameters 设置相机裁剪参数
    def set_cut_params(self, x1, y1, x2, y2):
        self.x1 = int(x1)
        self.y1 = int(y1)
        self.x2 = int(x2)
        self.y2 = int(y2)

    # set parameters to calculate the coords between cube and mecharm270
    # 设置参数以计算立方体和 mycobot 之间的坐标
    def set_params(self, c_x, c_y, ratio):
        self.c_x = c_x
        self.c_y = c_y
        self.ratio = 235.0 / ratio

    # calculate the coords between cube and mecharm270
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
            frame = frame[int(self.y2 * 0.66):int(self.y1 * 1.1),
                    int(self.x1 * 0.86):int(self.x2 * 1.08)]
        return frame

    # detect cube color
    def color_detect(self, img):
        # set the arrangement of color'HSV
        x = y = None
        for mycolor, item in self.HSV.items():
            # print("mycolor:",mycolor)
            # transfrom the img to model of gray 将图像转换为灰度模型
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

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
                    valid_boxes = []
                    for box in boxes:
                        _, _, w, h = box
                        area = w * h
                        if area < 10000:  # 可以根据实际图像尺寸调整这个阈值
                            continue
                        valid_boxes.append(box)

                    if valid_boxes:
                        # Select the rectangle with the largest area from all valid boxes
                        largest_box = max(valid_boxes, key=lambda b: b[2] * b[3])
                        # Unpack the top-left corner (x, y) and width-height (w, h) of the largest box
                        x, y, w, h = largest_box
                        # locate the target by drawing rectangle
                        cv2.rectangle(img, (x, y), (x + w, y + h), (153, 153, 0), 2)
                        # calculate the rectangle center
                        x, y = (x * 2 + w) / 2, (y * 2 + h) / 2

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

        # Judging whether it is recognized normally
        if x is not None and y is not None and (abs(x) + abs(y) > 0):
            return x, y
        else:
            return None


if __name__ == "__main__":

    # open the camera
    if platform.system() == "Windows":
        cap_num = 1
        cap = cv2.VideoCapture(cap_num, cv2.CAP_V4L)

        if not cap.isOpened():
            cap.open(1)
    elif platform.system() == "Linux":
        cap_num = 0
        cap = cv2.VideoCapture(cap_num, cv2.CAP_V4L)

        if not cap.isOpened():
            cap.open()

    # init a class of Object_detect
    detect = Object_detect()
    # init mecharm270
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
        if _init_ > 0:
            _init_ -= 1
            continue

        # calculate the parameters of camera clipping 计算相机裁剪的参数
        if init_num < 20:
            if detect.get_calculate_params(frame) is None:
                cv2.imshow("figure", frame)
                continue
            else:
                x1, x2, y1, y2 = detect.get_calculate_params(frame)
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

        # calculate params of the coords between cube and mecharm270 计算立方体和 mycobot 之间坐标的参数
        if nparams < 10:
            if detect.get_calculate_params(frame) is None:
                cv2.imshow("figure", frame)
                continue
            else:
                x1, x2, y1, y2 = detect.get_calculate_params(frame)
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
            # calculate and set params of calculating real coord between cube and mecharm270
            # 计算和设置计算立方体和mycobot之间真实坐标的参数
            detect.set_params(
                (detect.sum_x1 + detect.sum_x2) / 20.0,
                (detect.sum_y1 + detect.sum_y2) / 20.0,
                abs(detect.sum_x1 - detect.sum_x2) / 10.0 +
                abs(detect.sum_y1 - detect.sum_y2) / 10.0
            )
            print("ok")
            continue

        # get detect result 获取检测结果
        detect_result = detect.color_detect(frame)
        if detect_result is None:
            cv2.imshow("figure", frame)
            continue
        else:
            x, y = detect_result
            # calculate real coord between cube and mecharm270 计算立方体和 mycobot 之间的真实坐标
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
