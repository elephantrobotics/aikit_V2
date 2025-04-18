# encoding: UTF-8
import os
import platform
import time
import traceback

import cv2
import numpy as np
from pymycobot.mycobot320 import MyCobot320
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device
from gpiozero import LED

# y轴偏移量
pump_y = -30
# x轴偏移量
pump_x = 240


class Detect_marker():
    def __init__(self):

        # set cache of real coord
        self.cache_x = self.cache_y = 0

        self.robot_riscv = os.popen("ls /dev/ttyAMA*").readline()[:-1]

        # Creating a Camera Object
        cap_num = 20
        self.cap = cv2.VideoCapture(cap_num, cv2.CAP_V4L)
        self.cap.set(3, 640)
        self.cap.set(4, 480)

        Device.pin_factory = LGPIOFactory(chip=0) # 显式指定/dev/gpiochip0
        # 初始化 GPIO 控制的设备
        self.pump = LED(71)   # 气泵
        self.valve = LED(72)  # 阀门
        self.pump.on()
        time.sleep(0.05)
        self.valve.on()

        self.pub_pump(False)

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
            """start the suction pump"""
            # self.mc.set_basic_output(1, 0)
            # self.mc.set_basic_output(2, 1)
            self.pump.on()
            self.valve.off()
        else:
            """stop suction pump"""
            # self.mc.set_basic_output(1, 1)
            # self.mc.set_basic_output(2, 0)
            # time.sleep(1)
            # self.mc.set_basic_output(2, 1)
            self.pump.off()
            self.valve.on()

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
            while True:
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
        """
        Functions that control a series of movements of the robotic arm and grab blocks
        :param x: The x-axis coordinate of the block relative to the robot arm
        :param y: The y-axis coordinate of the block relative to the robot arm
        :param color: The index of where the block is placed(0-C,1-D,2-A,3-B)
        :return: None
        """
        print(color)

        angles = [
            [0.61, 45.87, -92.37, -41.3, 89.56, 9.58],  # init to point
            [18.8, -7.91, -54.49, -23.02, 89.56, -14.76],
            [17.22, -5.27, -52.47, -25.75, 89.73, -0.26],
        ]

        coords_to_angles = [
            [58.178, -55.45, -28.74, 3.51, 87.8, 46.14],  # A分拣区 A sorting area
            [99.58, -5.0, -92.9, 6.32, 87.89, -77.78],  # B分拣区  B sorting area
            [-24.69, -54.58, -36.65, 9.31, 90.35, -20.74],  # C分拣区 C sorting area
            [-60.9, 1.75, -98.45, 24.69, 90.17, -58.62],  # D分拣区 D sorting area
        ]
        print('real_x, real_y:', round(x, 2), round(y, 2))
        # send coordinates to move mycobot
        self.mc.send_angles(angles[2], 50)
        self.check_position(angles[2], 0)
        self.mc.send_coords([x, y, 94, -176, -0.32, -72.79], 100, 1)
        self.check_position([x, y, 94, -176, -0.32, -72.79], 1, max_same_data_count=30)

        # open pump
        self.pub_pump(True)
        time.sleep(1.5)
        tmp = []
        while True:
            if not tmp:
                tmp = self.mc.get_angles()
            else:
                break
        time.sleep(0.05)
        print(self.mc.get_coords())
        # print(tmp)
        self.mc.send_angles([tmp[0], -0.71, -54.49, -23.02, 89.56, tmp[5]],
                            50)  # [18.8, -7.91, -54.49, -23.02, -0.79, -14.76]
        self.check_position([tmp[0], -0.71, -54.49, -23.02, 89.56, tmp[5]], 0)
        # 抓取后放置区域
        self.mc.send_angles(coords_to_angles[color - 1],
                            50)  # coords[1] 为A分拣区，coords[2] 为B分拣区, coords[3] 为C分拣区，coords[4] 为D分拣区
        self.check_position(coords_to_angles[color - 1], 0)
        # close pump
        self.pub_pump(False)
        time.sleep(1.5)

        self.mc.send_angles(angles[0], 50)
        self.check_position(angles[0], 0)

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
            self.move(round(-x, 2), round(-y, 2), color)

    # init mycobot
    def init_mycobot(self):
        self.mc = MyCobot320(self.robot_riscv, 115200)
        if self.mc.get_fresh_mode() != 0:
            self.mc.set_fresh_mode(0)
        self.pub_pump(False)
        self.mc.send_angles([0.61, 45.87, -92.37, -41.3, 89.56, 9.58], 50)
        self.check_position([0.61, 45.87, -92.37, -41.3, 89.56, 9.58], 0)

    def encode_single(self):
        global pump_y, pump_x
        self.init_mycobot()
        print('ok')
        # control the number of crawls 控制抓取次数
        count = 0

        num = sum_x = sum_y = 0
        while cv2.waitKey(1) < 0:
            success, img = self.cap.read()
            # 旋转180度
            img = cv2.rotate(img, cv2.ROTATE_180)
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
            if ids is not None and len(ids) > 0:
                if np.array_equal(ids, np.array([[1]])):
                    self.color = 1
                elif np.array_equal(ids, np.array([[2]])):
                    self.color = 2
                elif np.array_equal(ids, np.array([[3]])):
                    self.color = 3
                elif np.array_equal(ids, np.array([[4]])):
                    self.color = 4
            if count < 2:
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
                        xyz = [round(xyz[0] * 1000 - pump_y, 2), round(xyz[1] * 1000 - pump_x, 2),
                               round(xyz[2] * 1000, 2)]

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
                                count += 1
            else:
                break

            cv2.imshow("encode_image", img)

    def encode_loop(self):
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
            if ids is not None and len(ids) > 0:
                if np.array_equal(ids, np.array([[1]])):
                    self.color = 1
                elif np.array_equal(ids, np.array([[2]])):
                    self.color = 2
                elif np.array_equal(ids, np.array([[3]])):
                    self.color = 3
                elif np.array_equal(ids, np.array([[4]])):
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
    # 提醒用户操作字典
    print("********************************************************")
    print("*  请输入数字选择模式(Please enter number selection mode)：*")
    print("*  1: 单次模式(single mode)                              *")
    print("*  2: 循环模式(loop mode)                                *")
    print("*  3: 退出(quit)                                        *")
    print("********************************************************")
    mode = int(input('请选择模式(please select mode):'))
    if mode == 1:
        detect.encode_single()
    elif mode == 2:
        detect.encode_loop()
    elif mode == 3:
        exit(0)
