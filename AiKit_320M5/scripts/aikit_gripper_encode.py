# encoding: UTF-8
import platform
import time

import cv2
import numpy as np
import serial
import serial.tools.list_ports
from pymycobot.mycobot320 import MyCobot320

# y轴偏移量
pump_y = -55
# x轴偏移量
pump_x = 15


class Detect_marker():
    def __init__(self):

        # set cache of real coord
        self.cache_x = self.cache_y = 0

        # get real serial
        self.plist = [
            str(x).split(" - ")[0].strip() for x in serial.tools.list_ports.comports()
        ]

        # Creating a Camera Object
        if platform.system() == "Windows":
            cap_num = 1
            self.cap = cv2.VideoCapture(cap_num, cv2.CAP_DSHOW)
            self.cap.set(3, 640)
            self.cap.set(4, 480)
        elif platform.system() == "Linux":
            cap_num = 0
            self.cap = cv2.VideoCapture(cap_num, cv2.CAP_V4L)
            self.cap.set(3, 640)
            self.cap.set(4, 480)

        # choose place to set cube
        self.color = 0
        # aruco code rotation angle
        self.yaw_degrees = 0

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
            self.mc.set_basic_output(1, 0)
            self.mc.set_basic_output(2, 1)
        else:
            """stop suction pump"""
            self.mc.set_basic_output(1, 1)
            self.mc.set_basic_output(2, 0)
            time.sleep(1)
            self.mc.set_basic_output(2, 1)

    def gripper_on(self):
        """start gripper"""
        self.mc.set_gripper_state(0, 100)
        time.sleep(1.5)

    def gripper_off(self):
        """stop gripper"""
        self.mc.set_gripper_state(1, 100)
        time.sleep(1.5)

    # Grasping motion
    def move(self, x, y, color, yaw_degrees):

        print(color, yaw_degrees)

        angles = [
            [0.61, 45.87, -92.37, -32.16, 89.56, 1.66],  # init to point
            [18.8, -7.91, -54.49, -23.02, 89.56, -14.76],
            [16.96, -5.27, -52.38, -17.66, 89.82, 0.0],
        ]

        coords = [
            [145.0, -65.5, 280.1, 178.99, 7.67, -179.9],  # 初始化点 init point
            [244.5, 193.2, 330.3, -160.54, 17.35, -74.59],  # A分拣区 A sorting area
            [33.2, 205.3, 322.5, -170.22, -13.93, 92.28],  # B分拣区  B sorting area
            [240.3, -202.2, 317.1, -152.12, -10.15, -95.73],  # C分拣区 C sorting area
            [30.3, -214.9, 302.3, -169.77, -8.64, -91.55],  # D分拣区 D sorting area
        ]

        if yaw_degrees > 169:
            yaw_degrees = 169
        elif yaw_degrees < -169:
            yaw_degrees = -169
        else:
            yaw_degrees = yaw_degrees

        yaw_degrees_opt = yaw_degrees + 5

        if yaw_degrees_opt > 170:
            yaw_degrees_opt = 170
        elif yaw_degrees_opt < -173:
            yaw_degrees_opt = -173
        else:
            yaw_degrees_opt = yaw_degrees_opt
        print('yaw_degrees_opt:', yaw_degrees_opt)
        print('real_x, real_y:', round(coords[0][0] + x, 2), round(coords[0][1] + y, 2))
        # send coordinates to move mycobot
        self.mc.send_angles(angles[2], 50)
        time.sleep(3)
        self.mc.send_angle(6, yaw_degrees_opt, 80)
        time.sleep(3)
        self.gripper_on()

        time.sleep(2.5)
        tmp_coords = []
        while True:
            if not tmp_coords:
                tmp_coords = self.mc.get_coords()
            else:
                break
        time.sleep(0.5)

        self.mc.send_coords([coords[0][0] + x, coords[0][1] + y, 250, tmp_coords[3], tmp_coords[4], tmp_coords[5]], 100,
                            1)
        time.sleep(2)
        self.mc.send_coords([coords[0][0] + x, coords[0][1] + y, 203, tmp_coords[3], tmp_coords[4], tmp_coords[5]], 100,
                            1)
        time.sleep(3)

        # close gripper
        self.gripper_off()
        tmp = []
        while True:
            if not tmp:
                tmp = self.mc.get_angles()
            else:
                break
        time.sleep(0.5)

        # print(tmp)
        self.mc.send_angles([tmp[0], -0.71, -54.49, -23.02, 89.56, tmp[5]], 50)
        time.sleep(3)
        # 抓取后放置区域
        self.mc.send_coords(coords[color], 100, 1)  # coords[1] 为A分拣区，coords[2] 为B分拣区, coords[3] 为C分拣区，coords[4] 为D分拣区
        time.sleep(6.5)

        # open gripper
        self.gripper_on()
        time.sleep(6.5)

        self.gripper_off()
        time.sleep(1)
        self.mc.send_angles(angles[0], 50)
        time.sleep(4.5)

    # decide whether grab cube
    def decide_move(self, x, y, color, yaw_degrees):

        # print(x,y)
        # detect the cube status move or run
        if (abs(x - self.cache_x) + abs(y - self.cache_y)) / 2 > 5:  # mm
            self.cache_x, self.cache_y = x, y
            return
        else:
            self.cache_x = self.cache_y = 0
            # 调整吸泵吸取位置，y增大,向左移动;y减小,向右移动;x增大,前方移动;x减小,向后方移动
            self.move(x + 100, y + 140, color, yaw_degrees)

    # init mycobot
    def init_mycobot(self):
        self.mc = MyCobot320(self.plist[0], 115200)
        self.mc.send_angles([0.61, 45.87, -92.37, -32.16, 89.56, 1.66], 50)
        time.sleep(2.5)
        # 设置夹爪为透传模式
        self.mc.set_gripper_mode(0)
        time.sleep(0.5)
        self.gripper_off()

    def run(self):
        global pump_y, pump_x
        self.init_mycobot()
        print('ok')
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
                    # 从旋转向量（rvec）计算旋转矩阵
                    rotation_matrix, _ = cv2.Rodrigues(rvec)

                    # 从旋转矩阵提取欧拉角（yaw、pitch、roll）
                    euler_angles = cv2.decomposeProjectionMatrix(np.hstack((rotation_matrix, tvec.reshape(3, 1))))[6]

                    # 提取yaw角度（绕Z轴旋转角度）
                    yaw_degrees = euler_angles[2]

                    # 输出ArUco码的旋转角
                    # print("Rotation (Yaw):", yaw_degrees)
                    self.yaw_degrees = round(yaw_degrees[0], 2)
                    # cv2.putText(img, str(xyz[:2]), (0, 64), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    for i in range(rvec.shape[0]):
                        # draw the aruco on img
                        cv2.aruco.drawDetectedMarkers(img, corners)

                        if num < 40:
                            sum_x += xyz[1]
                            sum_y += xyz[0]
                            num += 1
                        elif num == 40:
                            self.decide_move(sum_x / 40.0, sum_y / 40.0, self.color, self.yaw_degrees)
                            num = sum_x = sum_y = 0

            cv2.imshow("encode_image", img)


if __name__ == "__main__":
    detect = Detect_marker()
    detect.run()
