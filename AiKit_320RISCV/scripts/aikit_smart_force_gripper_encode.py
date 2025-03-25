# encoding: UTF-8
import traceback

import time

import cv2
import numpy as np
from pymycobot.mycobot320 import MyCobot320

# y轴偏移量
pump_y = -30
# x轴偏移量
pump_x = 235


class Detect_marker():
    def __init__(self):

        # set cache of real coord
        self.cache_x = self.cache_y = 0

        # Creating a Camera Object
        cap_num = 20
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

    def gripper_on(self):
        """start gripper"""
        self.mc.set_pro_gripper_open(14)
        time.sleep(1.5)

    def gripper_off(self):
        """stop gripper"""
        self.mc.set_pro_gripper_close(14)
        time.sleep(1.5)

    def calculate_j6_angle(self, qr_angle):
        """
        根据二维码旋转角计算机械臂J6关节的角度。（仅适用于320 力控夹爪）

        :param qr_angle: 二维码旋转角（yaw角），单位：度
        :return: J6目标角度，单位：度
        """
        # 偏移量（二维码0度对应J6关节-120度）
        offset = -120

        # 计算J6目标角度
        j6_angle = qr_angle + offset

        # 限制J6角度在[-180, 180]范围内
        if j6_angle > 180:
            j6_angle -= 360
        elif j6_angle < -180:
            j6_angle += 360

        return j6_angle

    # Grasping motion
    def move(self, x, y, color, yaw_degrees):

        print(color, yaw_degrees)
        print('x, y:', x, y)

        angles = [
            [0.61, 45.87, -92.37, -41.3, 89.56, -127],  # init to point
            [16.96, -6.85, -54.93, -19.68, 89.47, -127],  # point to grab
            [16.96, -6.85, -54.93, -19.68, 89.47, -127],
        ]

        coords_to_angels = [
            [54.58, -42.89, -11.16, -12.3, 90.61, -80],  # A分拣区 A sorting area
            [103.18, 9.75, -75.32, -11.16, 90.76, -30],  # B分拣区  B sorting area
            [-26, -33.92, -30.75, 0.66, 90.08, -155],  # C分拣区 C sorting area
            [-65.15, 8.17, -75.56, -8, 93.86, -10],  # D分拣区 D sorting area
        ]
        self.mc.send_angles(angles[2], 30)
        self.check_position(angles[2], 0)

        print('force rotation angle:', yaw_degrees)
        j6_angle = self.calculate_j6_angle(yaw_degrees)
        self.mc.send_angle(6, j6_angle, 80)
        self.check_position([16.96, -6.85, -54.93, -19.68, 89.47, j6_angle], 0)
        self.gripper_on()

        tmp_coords = []
        while True:
            if not tmp_coords:
                tmp_coords = self.mc.get_coords()
            else:
                break
        time.sleep(0.1)

        self.mc.send_coords(
            [x, y, 250, tmp_coords[3], tmp_coords[4], tmp_coords[5]], 100,
            1)
        self.mc.send_coords(
            [ x, y, 185, tmp_coords[3], tmp_coords[4], tmp_coords[5]], 100,
            1)
        self.check_position(
            [x, y, 185, tmp_coords[3], tmp_coords[4], tmp_coords[5]], 1, max_same_data_count=20)

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
        self.check_position([tmp[0], -0.71, -54.49, -23.02, 89.56, tmp[5]], 0)

        self.mc.send_angle(6, -127, 80)
        self.check_position([16.96, -6.85, -54.93, -19.68, 89.47, -127], 0)
        # 抓取后放置区域
        self.mc.send_angles(coords_to_angels[color-1],50)  # coords_to_angels[1] 为A分拣区，coords_to_angels[2] 为B分拣区, coords_to_angels[3] 为C分拣区，coords_to_angels[4] 为D分拣区
        self.check_position(coords_to_angels[color-1], 0)

        # open gripper
        self.gripper_on()

        self.mc.send_angles(angles[0], 50)
        self.check_position(angles[0], 0)
        # close gripper
        self.gripper_off()
        time.sleep(1)

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
                if (time.time() - start_time) >= 3:
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
            self.move(round(-x, 2), round(-y, 2), color, yaw_degrees)

    # init mycobot
    def init_mycobot(self):
        self.mc = MyCobot320('/dev/ttyAMA0', 115200)
        if self.mc.get_fresh_mode() != 0:
            self.mc.set_fresh_mode(0)
        self.mc.send_angles([0.61, 45.87, -92.37, -41.3, 89.56, -127], 40)
        self.check_position([0.61, 45.87, -92.37, -41.3, 89.56, -127], 0)
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
                    xyz = [round(xyz[0] * 1000 - pump_y, 2),
                           round(xyz[1] * 1000 - pump_x, 2), round(xyz[2] * 1000, 2)]
                    # 从旋转向量（rvec）计算旋转矩阵
                    rotation_matrix, _ = cv2.Rodrigues(rvec)

                    # 从旋转矩阵提取欧拉角（yaw、pitch、roll）
                    euler_angles = cv2.decomposeProjectionMatrix(np.hstack((rotation_matrix, tvec.reshape(3, 1))))[6]

                    # 提取yaw角度（绕Z轴旋转角度）
                    yaw_degrees = euler_angles[2]

                    # 输出ArUco码的旋转角
                    self.yaw_degrees = round(yaw_degrees[0], 2)
                    # print("Rotation (Yaw):", yaw_degrees)
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
