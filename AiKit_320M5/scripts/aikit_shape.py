import math
import sys
import time

import cv2
import numpy as np
import serial
import serial.tools.list_ports
from pymycobot.mycobot320 import MyCobot320
from common import limit_coords

IS_CV_4 = cv2.__version__[0] == '4'
__version__ = "1.0"


class Object_detect():

    def __init__(self, camera_x=260, camera_y=10):
        # inherit the parent class
        super(Object_detect, self).__init__()
        # declare mycobot320
        self.mc = None
        self.plist = [
            str(x).split(" - ")[0].strip() for x in serial.tools.list_ports.comports()
        ]

        # 移动角度
        self.move_angles = [
            [0.61, 45.87, -92.37, -41.3, 89.56, 9.58],  # init the point
            [18.8, -7.91, -54.49, -23.02, 89.56, -14.76],  # point to grab
            [17.22, -5.27, -52.47, -25.75, 89.73, -0.26],
        ]

        # 移动坐标
        # self.move_coords = [
        #     [32, -228.3, 201.6, -168.07, -7.17, -92.56],  # D Sorting area
        #     [266.5, -219.7, 209.3, -170, -3.64, -94.62],  # C Sorting area
        #     [253.8, 236.8, 224.6, -170, 6.87, -77.91],  # A Sorting area
        #     [35.9, 235.4, 211.8, -169.33, -9.27, 88.3],  # B Sorting area
        # ]
        self.move_coords_to_angles = [
            [-61.61, 3.6, -100.63, 12.91, 95.44, -59.06],  # D Sorting area
            [-25.22, -43.94, -39.9, 9.22, 90.43, -21.18],  # C Sorting area
            [58.18, -42.89, -32.69, -1.31, 89.38, 45.52],  # A Sorting area
            [100.1, -0.17, -95.0, 11.77, 97.64, -77.87],  # B Sorting area
        ]

        # choose place to set cube
        self.color = 0
        # parameters to calculate camera clipping parameters
        self.x1 = self.x2 = self.y1 = self.y2 = 0
        # set cache of real coord
        self.cache_x = self.cache_y = 0

        # use to calculate coord between cube and mycobot
        self.sum_x1 = self.sum_x2 = self.sum_y2 = self.sum_y1 = 0
        # The coordinates of the grab center point relative to the mycobot
        self.camera_x, self.camera_y = camera_x, camera_y
        # The coordinates of the cube relative to the mycobot
        self.c_x, self.c_y = 0, 0
        # The ratio of pixels to actual values
        self.ratio = 0
        # Get ArUco marker dict that can be detected.
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        # Get ArUco marker params.
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # 初始化背景减法器
        self.mog = cv2.bgsegm.createBackgroundSubtractorMOG()

    def pump_on(self):
        """Start the suction pump"""
        self.mc.set_basic_output(1, 0)
        self.mc.set_basic_output(2, 1)

    def pump_off(self):
        """stop suction pump m5"""
        self.mc.set_basic_output(1, 1)
        self.mc.set_basic_output(2, 0)
        time.sleep(1)
        self.mc.set_basic_output(2, 1)

    # Grasping motion
    def move(self, x, y, color):
        """
        Functions that control a series of movements of the robotic arm and grab blocks
        :param x: The x-axis coordinate of the block relative to the robot arm
        :param y: The y-axis coordinate of the block relative to the robot arm
        :param color: The index of where the block is placed(0-C,1-D,2-A,3-B)
        :return: None
        """
        print(color)
        print('x,y:', round(x, 2), round(y, 2))
        # send Angle to move mycobot320
        self.mc.send_angles(self.move_angles[2], 50)
        time.sleep(3)

        # send coordinates to move mycobot
        # self.mc.send_coords([x, y, 230, -173.84, -0.14, -74.37], 100, 1)
        target1 = [x, y, 230, -173.84, -0.14, -74.37]
        target1 = limit_coords(target1)  # <-- 自动限位
        self.mc.send_coords(target1, 100, 1)
        time.sleep(2.5)
        # self.mc.send_coords([x, y, 100, -173.84, -0.14, -74.37], 100, 1)  #
        target2 = [x, y, 95, -173.84, -0.14, -74.37]
        target2 = limit_coords(target2)  # <-- 自动限位
        self.mc.send_coords(target2, 100, 1)
        time.sleep(3)

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
        self.mc.send_angles([tmp[0], -0.71, -54.49, -23.02, 89.56, tmp[5]],
                            25)  # [18.8, -7.91, -54.49, -23.02, -0.79, -14.76]
        time.sleep(3)
        self.mc.send_angles(self.move_coords_to_angles[color], 50)
        time.sleep(2.5)
        # close pump
        self.pump_off()
        time.sleep(2)

        self.mc.send_angles(self.move_angles[0], 25)
        time.sleep(2.5)

    # decide whether grab cube
    def decide_move(self, x, y, color):
        print(x, y, self.cache_x, self.cache_y)
        # detect the cube status move or run
        if (abs(x - self.cache_x) + abs(y - self.cache_y)) / 2 > 5:  # mm
            self.cache_x, self.cache_y = x, y
            return
        else:
            self.cache_x = self.cache_y = 0
            # 调整吸泵吸取位置，y增大,向左移动;y减小,向右移动;x增大,前方移动;x减小,向后方移动
            self.move(x, y, color)

    # init mycobot320
    def run(self):
        self.mc = MyCobot320(self.plist[0], 115200)
        self.mc.send_angles([0.61, 45.87, -92.37, -41.3, 89.56, 9.58], 50)
        time.sleep(2.5)

    # draw aruco
    def draw_marker(self, img, x, y):
        # draw rectangle on img
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

    # get points of two aruco
    def get_calculate_params(self, img):
        """
        Get the center coordinates of two ArUco codes in the image
        :param img: Image, in color image format.
        :return: If two ArUco codes are detected, returns the coordinates of the centers of the two codes; otherwise returns None.
        """
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
            return center_x1, center_x2, center_y1, center_y2
        else:
            return None

    # set camera clipping parameters
    def set_cut_params(self, x1, y1, x2, y2):
        self.x1 = int(x1)
        self.y1 = int(y1)
        self.x2 = int(x2)
        self.y2 = int(y2)
        print(self.x1, self.y1, self.x2, self.y2)

    # set parameters to calculate the coords between cube and mycobot320
    def set_params(self, c_x, c_y, ratio):
        self.c_x = c_x
        self.c_y = c_y
        self.ratio = 320.0 / ratio

    # calculate the coords between cube and mycobot320
    def get_position(self, x, y):
        # 二维码板子摆放方向改变之前
        # pot_x = ((y - self.c_y) * self.ratio + self.camera_x)
        # pot_y = ((x - self.c_x) * self.ratio + self.camera_y)

        # 二维码板子摆放方向改变之后
        pot_x = ((y - self.c_y) * (-self.ratio) + self.camera_x)
        pot_y = -((x - self.c_x) * self.ratio + self.camera_y)
        return pot_x, pot_y
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
                    int(self.x1 * 0.88):int(self.x2 * 1.06)]
        return frame

    # 检测物体的形状
    def shape_detect(self, img):
        """
        Color recognition detection function
        :param img: Image information read under the camera
        :return: image coordinates
        """
        x = 0
        y = 0
        Alpha = 65.6
        Gamma = -8191.5
        cal = cv2.addWeighted(img, Alpha, img, 0, Gamma)
        gray = cv2.cvtColor(cal, cv2.COLOR_BGR2GRAY)

        # 转换为灰度图片
        # ray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        # a etching operation on a picture to remove edge roughness
        erosion = cv2.erode(gray, np.ones((2, 2), np.uint8), iterations=2)

        # the image for expansion operation, its role is to deepen the color depth in the picture
        dilation = cv2.dilate(erosion, np.ones(
            (1, 1), np.uint8), iterations=2)

        # 设定灰度图的阈值 175, 255
        _, threshold = cv2.threshold(dilation, 175, 255, cv2.THRESH_BINARY)
        # 边缘检测
        edges = cv2.Canny(threshold, 50, 100)
        # 检测物体边框
        contours, _ = cv2.findContours(
            edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        if len(contours) > 0:
            for cnt in contours:
                # if 6000>cv2.contourArea(cnt) and cv2.contourArea(cnt)>4500:
                if cv2.contourArea(cnt) > 5500:
                    objectType = None
                    peri = cv2.arcLength(cnt, True)
                    approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
                    objCor = len(approx)
                    x, y, w, h = cv2.boundingRect(approx)
                    print('objCor:', objCor)

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
                        # find the largest object that fits the requirements
                        c = max(contours, key=cv2.contourArea)
                        rect = cv2.minAreaRect(c)
                        box = cv2.boxPoints(rect)
                        box = np.int0(box)
                        cv2.drawContours(img, [box], 0, (153, 153, 0), 2)
                        x = int(rect[0][0])
                        y = int(rect[0][1])

                    if objCor == 3:
                        objectType = "Triangle(三角形)"
                        cv2.drawContours(img, [cnt], 0, (0, 0, 255), 3)
                        self.color = 3
                    elif objCor == 4:
                        box = cv2.boxPoints(rect)
                        box = np.int0(box)
                        _W = math.sqrt(math.pow((box[0][0] - box[1][0]), 2) + math.pow((box[0][1] - box[1][1]), 2))
                        _H = math.sqrt(math.pow((box[0][0] - box[3][0]), 2) + math.pow((box[0][1] - box[3][1]), 2))
                        aspRatio = _W / float(_H)
                        if 0.98 < aspRatio < 1.03:
                            objectType = "Square(正方形)"
                            cv2.drawContours(img, [cnt], 0, (0, 0, 255), 3)
                            self.color = 1
                        else:
                            objectType = "Rectangle(长方形)"
                            cv2.drawContours(img, [cnt], 0, (0, 0, 255), 3)
                            self.color = 2
                    elif objCor >= 8:
                        objectType = "Circle(圆形)"
                        self.color = 0
                        cv2.drawContours(img, [cnt], 0, (0, 0, 255), 3)
                    else:
                        print('not recognized!')
                    print(objectType)

        if abs(x) + abs(y) > 0:
            return x, y
        else:
            return None


def runs():
    """
    Turn on the camera and call the detection function to get the detection result and move the robot arm
    :return:
    """
    # open the camera
    import platform

    if platform.system() == "Windows":
        cap_num = 1
        # cap = cv2.VideoCapture(cap_num, cv2.CAP_V4L)
        cap = cv2.VideoCapture(cap_num, cv2.CAP_DSHOW)
        cap.set(3, 640)
        cap.set(4, 480)
        if not cap.isOpened():
            cap.open(1)
    elif platform.system() == "Linux":
        cap_num = 0
        cap = cv2.VideoCapture(cap_num, cv2.CAP_V4L)
        cap.set(3, 640)
        cap.set(4, 480)
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
        # 旋转180度
        frame = cv2.rotate(frame, cv2.ROTATE_180)
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
            # get detect result 获取检测结果z
            detect_result = detect.shape_detect(frame)
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
    runs()
