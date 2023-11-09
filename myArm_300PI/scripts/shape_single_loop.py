import cv2
import numpy as np
import time
import os, sys
import math

from pymycobot.myarm import MyArm

IS_CV_4 = cv2.__version__[0] == '4'
__version__ = "1.0"


# Adaptive seeed


class Object_detect():

    def __init__(self, camera_x=160, camera_y=0):
        # inherit the parent class
        super(Object_detect, self).__init__()
        # declare myarm 300
        self.ma = None

        # 移动角度
        self.move_angles = [
            [-60, 0, 0, -90, 0, -83, 0],  # init the point
            [0, 0, 0, -90, 0, -83, 0],  # point to grab
        ]

        # 移动坐标
        self.move_coords = [
            [122.4, -143.5, 181.8, 179.92, 6.5, 130.23], # D Sorting area [-49.74, 28.74, 0.26, -71.8, 0.0, -72.94, 0.26]
            [209.4, -127.7, 186.1, 179.49, 25.39, 148.18], # C Sorting area [-31.64, 50.62, 0.43, -38.05, 0.0, -65.91, 0.26]
            [116.6, 154.6, 177.8, 179.51, 11.59, -127.46], # A Sorting area [52.47, 28.82, 0.52, -73.91, 0.17, -65.65, 0.26]
            [6.6, 164.3, 179.7, 179.69, 8.43, -92.74],  # B Sorting area [87.27, 16.08, 0.35, -90.0, 0.17, -65.47, 0.26]
        ]

        # which robot: USB* is m5; ACM* is wio; AMA* is raspi
        self.robot_raspi = os.popen("ls /dev/ttyAMA*").readline()[:-1]
        self.raspi = False

        if "dev" in self.robot_raspi:
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

        # choose place to set cube
        self.color = 0
        # parameters to calculate camera clipping parameters
        self.x1 = self.x2 = self.y1 = self.y2 = 0
        # set cache of real coord
        self.cache_x = self.cache_y = 0

        # use to calculate coord between cube and myarm
        self.sum_x1 = self.sum_x2 = self.sum_y2 = self.sum_y1 = 0
        # The coordinates of the grab center point relative to the myarm
        self.camera_x, self.camera_y = camera_x, camera_y
        # The coordinates of the cube relative to the myarm
        self.c_x, self.c_y = 0, 0
        # The ratio of pixels to actual values
        self.ratio = 0
        # Get ArUco marker dict that can be detected.
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        # Get ArUco marker params.
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # 初始化背景减法器
        self.mog = cv2.bgsegm.createBackgroundSubtractorMOG()

        # pump_control pi

    def gpio_status(self, flag):
        if flag:
            self.GPIO.output(20, 0)
            self.GPIO.output(21, 0)
        else:
            self.GPIO.output(20, 1)
            self.GPIO.output(21, 1)

        # Grasping motion

    def move(self, x, y, color):
        # send Angle to move myArm 300
        print(color)
        self.ma.send_angles(self.move_angles[1], 35)  # [126.1, 0.7, 217.0, 179.64, -1.14, -179.64]
        time.sleep(3)

        # send coordinates to move myArm
        self.ma.send_coords([x, y, 190.5, -179.72, 6.5, -179.43], 40, 1)  # [164.2, 0.9, 190.5, 179.65, -1.14, 179.94]
        time.sleep(3)

        # self.ma.send_coords([x, y, 150, 179.87, -3.78, -62.75], 25, 0)
        # time.sleep(3)

        self.ma.send_coords([x, y, 70, -179.72, 6.5, -179.43], 40, 1)  # [165.0, 0.9, 109.6, 179.69, 0.08, 179.78]
        time.sleep(3)

        # open pump
        self.gpio_status(True)
        time.sleep(1.5)

        tmp = []
        while True:
            if not tmp:
                tmp = self.ma.get_angles()
            else:
                break
        time.sleep(0.5)

        # print(tmp)
        self.ma.send_angles([tmp[0], 0, 0, -90, -0.79, -83, tmp[6]], 35)  # [0, 0, 0, -90, 0, -83, 0]
        time.sleep(3)

        self.ma.send_coords(self.move_coords[color], 40, 1)

        time.sleep(3)

        # close pump
        self.gpio_status(False)
        time.sleep(5)

        self.ma.send_angles(self.move_angles[0], 50)
        time.sleep(3)

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

        # init myArm 300

    def run(self):
        if "dev" in self.robot_raspi:
            self.ma = MyArm(self.robot_raspi, 115200)
        self.gpio_status(False)
        self.ma.send_angles([-60, 0, 0, -90, 0, -83, 0], 40)
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

    # set parameters to calculate the coords between cube and myarm 300
    def set_params(self, c_x, c_y, ratio):
        self.c_x = c_x
        self.c_y = c_y
        self.ratio = 220.0 / ratio

    # calculate the coords between cube and myarm 300
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
                    int(self.x1 * 0.88):int(self.x2 * 1.06)]
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
                if cv2.contourArea(cnt) > 5500:
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


def shape_single():
    # open the camera
    cap_num = 0
    cap = cv2.VideoCapture(cap_num, cv2.CAP_V4L)
    cap.set(3, 640)
    cap.set(4, 480)
    if not cap.isOpened():
        cap.open()
    # init a class of Object_detect
    detect = Object_detect()
    # init myarm 300
    detect.run()
    # Control the number of crawls 控制抓取次数
    count = 0

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

        # calculate params of the coords between cube and myarm 300
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
            # calculate and set params of calculating real coord between cube and myarm 300
            detect.set_params(
                (detect.sum_x1 + detect.sum_x2) / 20.0,
                (detect.sum_y1 + detect.sum_y2) / 20.0,
                abs(detect.sum_x1 - detect.sum_x2) / 10.0 +
                abs(detect.sum_y1 - detect.sum_y2) / 10.0
            )
            print("ok")
            continue

        if count < 2:
            # get detect result
            # print('调用检测')
            detect_result = detect.shape_detect(frame)
            # print("完成检测")
            if detect_result is None:
                cv2.imshow("figure", frame)
                continue
            else:
                x, y = detect_result
                # calculate real coord between cube and myarm 300
                real_x, real_y = detect.get_position(x, y)
                if num == 20:

                    detect.decide_move(real_sx / 20.0, real_sy / 20.0, detect.color)
                    num = real_sx = real_sy = 0
                    count += 1

                else:
                    num += 1
                    real_sy += real_y
                    real_sx += real_x
        else:
            break

        cv2.imshow("figure", frame)

        # close the window
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            sys.exit()


def shape_loop():
    # open the camera
    cap_num = 0
    cap = cv2.VideoCapture(cap_num, cv2.CAP_V4L)
    cap.set(3, 640)
    cap.set(4, 480)
    if not cap.isOpened():
        cap.open()
    # init a class of Object_detect
    detect = Object_detect()
    # init myarm 300
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

        # calculate params of the coords between cube and myarm 300
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
            # calculate and set params of calculating real coord between cube and myarm 300
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
            # calculate real coord between cube and myarm 300
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


if __name__ == '__main__':
    # 提醒用户操作字典
    print("********************************************************")
    print("*  请输入数字选择模式(Please enter number selection mode)：*")
    print("*  1: 单次模式(single mode)                              *")
    print("*  2: 循环模式(loop mode)                                *")
    print("*  3: 退出(quit)                                        *")
    print("********************************************************")
    mode = int(input('请选择模式(please select mode):'))
    if mode == 1:
        shape_single()
    elif mode == 2:
        shape_loop()
    elif mode == 3:
        exit(0)