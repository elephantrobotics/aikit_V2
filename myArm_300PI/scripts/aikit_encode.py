# encoding: UTF-8
import cv2
import numpy as np
from pymycobot.myarm import MyArm
import time
import os

# y轴偏移量
pump_y = -55
# x轴偏移量
pump_x = 15


class Detect_marker():
    def __init__(self):

        # set cache of real coord
        self.cache_x = self.cache_y = 0

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
            self.pub_pump(False)

        # Creating a Camera Object
        cap_num = 0
        self.cap = cv2.VideoCapture(cap_num, cv2.CAP_V4L)
        self.cap.set(3, 640)
        self.cap.set(4, 480)

        # choose place to set cube
        self.color = 0

        # Get ArUco marker dict that can be detected.
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        # Get ArUco marker params.
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        # 摄像头的内参矩阵
        self.camera_matrix = np.array([
            [781.33379113, 0., 347.53500524],
            [0., 783.79074192, 246.67627253],
            [0., 0., 1.]])

        # 摄像头的畸变系数
        self.dist_coeffs = np.array(([[3.41360787e-01, -2.52114260e+00, -1.28012469e-03, 6.70503562e-03,
                                       2.57018000e+00]]))

    # 控制吸泵      
    def pub_pump(self, flag):
        if flag:
            self.GPIO.output(20, 0)
            self.GPIO.output(21, 0)
        else:
            self.GPIO.output(20, 1)
            self.GPIO.output(21, 1)

    # Grasping motion
    def move(self, x, y, color):
        print(color)

        angles = [
            [-60, 0, 0, -90, 0, -90, 0],  # init the point
            [0, 0, 0, -90, 0, -90, 0],  # point to grab
        ]

        coords = [
            [145.0, -65.5, 280.1, 178.99, 7.67, -179.9],  # 初始化点 init point
            [52.99, -18.36, -1.4, -86.57, -0.17, -55.89, -0.08],  # A分拣区 A sorting area
            [87.97, -18.28, -1.4, -86.57, -0.35, -71.19, -0.08],  # B分拣区  B sorting area
            [-27.59, -44.82, -1.75, -48.95, 0.0, -55.89, -0.08],  # C分拣区 C sorting area
            [-47.9, -17.31, 0.17, -89.91, -0.17, -56.07, 0.0],  # D分拣区 D sorting area

        ]
        print('real_x, real_y:', round(coords[0][0] + x, 2), round(coords[0][1] + y, 2))
        # send coordinates to move mycobot
        self.ma.send_angles(angles[1], 25)
        time.sleep(3)

        # self.ma.send_coords([coords[0][0]+x, coords[0][1]+y, 240, 178.99, 5.38, -179.9], 20, 0)
        # time.sleep(2)
        self.ma.send_coords([coords[0][0] + x, coords[0][1] + y, 200, 178.99, -3.78, -62.9], 40, 1)
        time.sleep(2)
        # self.ma.send_coords([coords[0][0]+x, coords[0][1]+y, 105, 178.99, -3.78, -62.9], 25, 0)
        # time.sleep(2)
        self.ma.send_coords([coords[0][0] + x, coords[0][1] + y, 65.5, 178.99, -3.78, -62.9], 40, 1)
        time.sleep(3.5)

        # open pump
        if "dev" in self.robot_raspi:
            self.pub_pump(True)
        time.sleep(1.5)

        tmp = []
        while True:
            if not tmp:
                tmp = self.ma.get_angles()
            else:
                break
        time.sleep(0.5)

        # print(tmp)
        self.ma.send_angles([tmp[0], 0, 0, -90, -0.79, -90, tmp[6]],
                            50)  # [18.8, -7.91, -54.49, -23.02, -0.79, -14.76]
        time.sleep(3)
        # 抓取后放置区域
        self.ma.send_coords(coords[color], 40, 1)  # coords[1] 为A分拣区，coords[2] 为B分拣区, coords[3] 为C分拣区，coords[4] 为D分拣区
        time.sleep(4)

        # close pump
        if "dev" in self.robot_raspi:
            self.pub_pump(False)
        time.sleep(5)

        self.ma.send_angles(angles[0], 50)
        time.sleep(2)

    # decide whether grab cube
    def decide_move(self, x, y, color):

        # print(x,y)
        # detect the cube status move or run
        if (abs(x - self.cache_x) + abs(y - self.cache_y)) / 2 > 5:  # mm
            self.cache_x, self.cache_y = x, y
            return
        else:
            self.cache_x = self.cache_y = 0
            # 调整吸泵吸取位置，y增大,向左移动;y减小,向右移动;x增大,前方移动;x减小,向后方移动
            self.move(x - 15, y + 145, color)

    # init mycobot
    def init_mycobot(self):
        if "dev" in self.robot_raspi:
            self.ma = MyArm(self.robot_raspi, 1000000)
        self.pub_pump(False)
        self.ma.send_angles([-60, 0, 0, -90, 0, -90, 0], 20)
        # self.ma.send_coords([135.0, -65.5, 280.1, 178.99, 5.38, -179.9], 20, 1)
        time.sleep(2.5)

    def run(self):
        global pump_y, pump_x
        self.init_mycobot()
        print('ok')
        num = sum_x = sum_y = 0
        while cv2.waitKey(1) < 0:
            success, img = self.cap.read()
            if not success:
                print("It seems that the image cannot be acquired correctly.")
                break

            # transfrom the img to model of gray
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # Detect ArUco marker.
            corners, ids, rejectImaPoint = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )

            # Determine the placement point of the QR code
            if ids == np.array([[1]]):
                self.color = 1
            elif ids == np.array([[2]]):
                self.color = 2
            elif ids == np.array([[3]]):
                self.color = 3
            elif ids == np.array([[4]]):
                self.color = 4

            if len(corners) > 0:
                if ids is not None:
                    # get informations of aruco
                    ret = cv2.aruco.estimatePoseSingleMarkers(
                        corners, 0.03, self.camera_matrix, self.dist_coeffs
                    )
                    # rvec:rotation offset,tvec:translation deviator
                    (rvec, tvec) = (ret[0], ret[1])
                    (rvec - tvec).any()
                    xyz = tvec[0, 0, :]
                    # calculate the coordinates of the aruco relative to the pump
                    xyz = [round(xyz[0] * 1000 + pump_y, 2), round(xyz[1] * 1000 + pump_x, 2), round(xyz[2] * 1000, 2)]

                    # cv2.putText(img, str(xyz[:2]), (0, 64), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    for i in range(rvec.shape[0]):
                        # draw the aruco on img
                        cv2.aruco.drawDetectedMarkers(img, corners)

                        if num < 40:
                            sum_x += xyz[1]
                            sum_y += xyz[0]
                            num += 1
                        elif num == 40:
                            self.decide_move(sum_x / 40.0, sum_y / 40.0, self.color)
                            num = sum_x = sum_y = 0

            cv2.imshow("encode_image", img)


if __name__ == "__main__":
    detect = Detect_marker()
    detect.run()
