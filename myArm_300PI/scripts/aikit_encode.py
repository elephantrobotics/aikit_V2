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
        self.ma = None
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
            [-60, 0, 0, -90, 0, -83, 0],  # init the point
            [0, 0, 0, -90, 0, -83, 0],  # point to grab
        ]

        coords = [
            [126.1, 0.7, 217.0, 179.64, -1.14, -179.64],  # 初始化点 init point
            [116.6, 154.6, 177.8, 179.51, 11.59, -127.46],  # A Sorting area [52.47, 28.82, 0.52, -73.91, 0.17, -65.65, 0.26]
            [6.6, 164.3, 179.7, 179.69, 8.43, -92.74],  # B Sorting area [87.27, 16.08, 0.35, -90.0, 0.17, -65.47, 0.26]
            [209.4, -127.7, 186.1, 179.49, 25.39, 148.18],  # C Sorting area [-31.64, 50.62, 0.43, -38.05, 0.0, -65.91, 0.26]
            [122.4, -143.5, 181.8, 179.92, 6.5, 130.23],  # D Sorting area [-49.74, 28.74, 0.26, -71.8, 0.0, -72.94, 0.26]

        ]
        print('real_x, real_y:', round(coords[0][0] + x, 2), round(coords[0][1] + y, 2))
        # send coordinates to move myarm 300
        self.ma.send_angles(angles[1], 35) # [126.1, 0.7, 217.0, 179.64, -1.14, -179.64]
        time.sleep(3)

        self.ma.send_coords([coords[0][0] + x, coords[0][1] + y, 190.5, -179.72, 6.5, -179.43], 40, 1)  # [164.2, 0.9, 190.5, 179.65, -1.14, 179.94])
        time.sleep(3)
        self.ma.send_coords([coords[0][0] + x, coords[0][1] + y, 70, -179.72, 6.5, -179.43], 40, 1)  # [165.0, 0.9, 109.6, 179.69, 0.08, 179.78]
        time.sleep(3)

        # open pump
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
        self.ma.send_angles([tmp[0], 0, 0, -90, -0.79, -83, tmp[6]],
                            35)  # [18.8, -7.91, -54.49, -23.02, -0.79, -14.76]
        time.sleep(3)
        # 抓取后放置区域
        self.ma.send_coords(coords[color], 40, 1)  # coords[1] 为A分拣区，coords[2] 为B分拣区, coords[3] 为C分拣区，coords[4] 为D分拣区
        time.sleep(3)

        # close pump
        self.pub_pump(False)
        time.sleep(5)

        self.ma.send_angles(angles[0], 50)
        time.sleep(3)

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
            self.move(x + 30, y + 70, color)

    # init myarm 300
    def init_mycobot(self):
        if "dev" in self.robot_raspi:
            self.ma = MyArm(self.robot_raspi, 115200)
        self.pub_pump(False)
        self.ma.send_angles([-60, 0, 0, -90, 0, -83, 0], 20)
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
