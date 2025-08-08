import traceback
import RPi.GPIO as GPIO
import cv2
import numpy as np
import time
import os, sys
import math

from pymycobot.mycobot280 import MyCobot280
from offset_utils import load_offset_from_txt

IS_CV_4 = cv2.__version__[0] == '4'
__version__ = "1.0"


offset_path = '/home/er/AiKit_UI/libraries/offset/myCobot 280 for Pi_shape.txt'

camera_x, camera_y, camera_z = load_offset_from_txt(offset_path)


class Object_detect():

    def __init__(self, camera_x=camera_x, camera_y=camera_y):
        # inherit the parent class
        super(Object_detect, self).__init__()
        # declare mycobot280
        self.mc = None

        # 移动角度
        self.move_angles = [
            [0.61, 45.87, -92.37, -41.3, 2.02, 9.58],  # init the point
            [18.8, -7.91, -54.49, -23.02, -0.79, -14.76],  # point to grab
        ]

        self.new_move_coords_to_angles = [
            [-33.22, -10.28, -84.99, 4.83, 0.08, -7.99],  # D Sorting area
            [-21.79, -52.82, -26.45, -5.53, 0.08, -7.91],  # C Sorting area
            [47.81, -53.61, -27.15, -6.41, 0.08, -7.73],  # A Sorting area
            [72.42, -6.06, -98.43, 14.23, -0.87, -8.96],  # B Sorting area
        ]

        # self.z_down_values = [138, 145, 147, 135]  # D, C, A, B
        self.z_down_values = [113, 120, 122, 110]  # D, C, A, B


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

        self.camera_z = camera_z

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(20, GPIO.OUT)
        GPIO.setup(21, GPIO.OUT)

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
        # send Angle to move mycobot280
        print(color)
        self.mc.send_angles(self.move_angles[1], 50)
        self.check_position(self.move_angles[1], 0)

        # send coordinates to move mycobot
        self.mc.send_coords([x, y, 170.6, 179.87, -3.78, -62.75], 70, 1)  # usb :rx,ry,rz -173.3, -5.48, -57.9
        self.mc.send_coords([x, y, self.camera_z, 179.87, -3.78, -62.75], 70, 1)
        data = [x, y, self.camera_z, 179.87, -3.78, -62.75]
        self.check_position(data, 1)

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
        self.mc.send_angles([tmp[0], -0.71, -54.49, -23.02, -0.79, tmp[5]],
                            50)  # [18.8, -7.91, -54.49, -23.02, -0.79, -14.76]
        self.check_position([tmp[0], -0.71, -54.49, -23.02, -0.79, tmp[5]], 0)

        self.mc.send_angles(self.new_move_coords_to_angles[color], 50)
        self.check_position(self.new_move_coords_to_angles[color], 0)

        self.mc.send_coord(3, self.z_down_values[color], 50)
        time.sleep(2)

        # close pump
        self.pump_off()
        time.sleep(0.5)

        self.mc.send_angles(self.move_angles[0], 50)
        self.check_position(self.move_angles[0], 0)

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

    # init mycobot280
    def run(self):
        self.mc = MyCobot280('/dev/ttyAMA0', 1000000)
        if self.mc.get_fresh_mode() != 0:
            self.mc.set_fresh_mode(0)
        self.pump_off()
        self.mc.send_angles([0.61, 45.87, -92.37, -41.3, 2.02, 9.58], 50)
        self.check_position([0.61, 45.87, -92.37, -41.3, 2.02, 9.58], 0)

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
        if len(corners) > 0:
            if ids is not None:
                if len(corners) <= 1 or ids[0] == 1:
                    return None
                x1 = x2 = y1 = y2 = 0
                point_11, point_21, point_31, point_41 = corners[0][0]
                x1, y1 = int((point_11[0] + point_21[0] + point_31[0] + point_41[0]) / 4.0), int(
                    (point_11[1] + point_21[1] + point_31[1] + point_41[1]) / 4.0)
                point_1, point_2, point_3, point_4 = corners[1][0]
                x2, y2 = int((point_1[0] + point_2[0] + point_3[0] + point_4[0]) / 4.0), int(
                    (point_1[1] + point_2[1] + point_3[1] + point_4[1]) / 4.0)
                return x1, x2, y1, y2
        return None

    # set camera clipping parameters
    def set_cut_params(self, x1, y1, x2, y2):
        self.x1 = int(x1)
        self.y1 = int(y1)
        self.x2 = int(x2)
        self.y2 = int(y2)
        print(self.x1, self.y1, self.x2, self.y2)

    # set parameters to calculate the coords between cube and mycobot280
    def set_params(self, c_x, c_y, ratio):
        self.c_x = c_x
        self.c_y = c_y
        self.ratio = 235.0 / ratio

    # calculate the coords between cube and mycobot280
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

    # 检测物体的形状
    def shape_detect(self, img):
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
                if cv2.contourArea(cnt) > 6900:
                    objectType = None
                    peri = cv2.arcLength(cnt, True)
                    approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
                    objCor = len(approx)
                    x, y, w, h = cv2.boundingRect(approx)

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
                    elif objCor >= 5:
                        objectType = "Circle(圆形)"
                        self.color = 0
                        cv2.drawContours(img, [cnt], 0, (0, 0, 255), 3)
                    else:
                        pass
                    print(objectType)

        if abs(x) + abs(y) > 0:
            return x, y
        else:
            return None


if __name__ == "__main__":

    # open the camera
    cap_num = 0
    cap = cv2.VideoCapture(cap_num, cv2.CAP_V4L)
    cap.set(3, 640)
    cap.set(4, 480)
    if not cap.isOpened():
        cap.open()
    # init a class of Object_detect
    detect = Object_detect()
    # init mycobot280
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

        # calculate the parameters of camera clipping
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

        # calculate params of the coords between cube and mycobot280
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
            # calculate and set params of calculating real coord between cube and mycobot280
            detect.set_params(
                (detect.sum_x1 + detect.sum_x2) / 20.0,
                (detect.sum_y1 + detect.sum_y2) / 20.0,
                abs(detect.sum_x1 - detect.sum_x2) / 10.0 +
                abs(detect.sum_y1 - detect.sum_y2) / 10.0
            )
            print("ok")
            continue

        # get detect result
        # detect_result = detect.color_detect(frame)
        # print('调用检测')
        detect_result = detect.shape_detect(frame)
        # print("完成检测")
        if detect_result is None:
            cv2.imshow("figure", frame)
            continue
        else:
            x, y = detect_result
            # calculate real coord between cube and mycobot280
            real_x, real_y = detect.get_position(x, y)
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
