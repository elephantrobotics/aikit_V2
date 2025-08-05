# encoding: UTF-8
import platform
import time
import traceback

import RPi.GPIO as GPIO
import cv2
import numpy as np
from pymycobot.mycobot280 import MyCobot280

from offset_utils import load_offset_from_txt

# y轴偏移量
pump_y = -55
# x轴偏移量
pump_x = 15

offset_path = '/home/er/AiKit_UI/libraries/offset/myCobot 280 for Pi_encode.txt'

camera_x, camera_y, camera_z = load_offset_from_txt(offset_path)

class Detect_marker():
    def __init__(self, x_offset=camera_x, y_offset=camera_y):

        # set cache of real coord
        self.cache_x = self.cache_y = 0

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

        self.x_offset = x_offset
        self.y_offset = y_offset
        self.camera_z = camera_z

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(20, GPIO.OUT)
        GPIO.setup(21, GPIO.OUT)

    # 控制吸泵
    def pub_pump(self, flag):
        if flag:
            GPIO.output(20, 0)
            time.sleep(0.05)
        else:
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

        print(color)

        angles = [
            [0.61, 45.87, -92.37, -41.3, 2.02, 9.58],  # init to point
            [18.8, -7.91, -54.49, -23.02, -0.79, -14.76],
        ]

        new_move_coords_to_angles = [
            [-33.22, -10.28, -84.99, 4.83, 0.08, -7.99],  # D Sorting area
            [-21.79, -52.82, -26.45, -5.53, 0.08, -7.91],  # C Sorting area
            [47.81, -53.61, -27.15, -6.41, 0.08, -7.73],  # A Sorting area
            [72.42, -6.06, -98.43, 14.23, -0.87, -8.96],  # B Sorting area
        ]
        z_down_values = [138, 145, 147, 135]  # D, C, A, B

        print('real_x, real_y:', round(x, 2), round(y, 2))
        # send coordinates to move mycobot
        self.mc.send_angles(angles[1], 50)
        self.check_position(angles[1], 0)

        self.mc.send_coords([x, y, 170, 178.99, -3.78, -62.9], 70, 1)

        self.mc.send_coords([x, y, self.camera_z, 178.99, -3.78, -62.9], 70, 1)
        data = [x, y, self.camera_z, 178.99, -3.78, -62.9]
        self.check_position(data, 1)

        # open pump
        self.pub_pump(True)
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
        # 抓取后放置区域
        self.mc.send_angles(new_move_coords_to_angles[color], 50)
        self.check_position(new_move_coords_to_angles[color], 0)

        self.mc.send_coord(3, z_down_values[color], 50)
        time.sleep(1.5)

        # close pump
        self.pub_pump(False)
        time.sleep(0.5)

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
            self.move(round(x, 2), round(y, 2), color)

    # init mycobot
    def init_mycobot(self):
        self.mc = MyCobot280('/dev/ttyAMA0', 1000000)
        if self.mc.get_fresh_mode() != 0:
            self.mc.set_fresh_mode(0)
        self.pub_pump(False)
        self.mc.send_angles([0.61, 45.87, -92.37, -41.3, 2.02, 9.58], 50)
        self.check_position([0.61, 45.87, -92.37, -41.3, 2.02, 9.58], 0)

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

            # 只处理目标ID
            target_ids = [3, 4, 5, 6]
            filtered_corners = []
            filtered_ids = []
            if ids is not None:
                for i, id_val in enumerate(ids.flatten()):
                    if id_val in target_ids:
                        filtered_corners.append(corners[i])
                        filtered_ids.append(id_val)
            if len(filtered_corners) > 0:
                filtered_corners = np.array(filtered_corners)
                filtered_ids = np.array(filtered_ids).reshape(-1, 1)
                # 根据 ArUco ID 获取 color（ID 3-6 → color 0-3）
                colors = np.array([id_val - 3 for id_val in filtered_ids.flatten()]).reshape(-1, 1)
                self.color = colors[0][0]
                id_val = filtered_ids[0][0]
                # get informations of aruco
                ret = cv2.aruco.estimatePoseSingleMarkers(
                    filtered_corners, 0.03, self.camera_matrix, self.dist_coeffs
                )
                # rvec:rotation offset,tvec:translation deviator
                (rvec, tvec) = (ret[0], ret[1])
                (rvec - tvec).any()
                xyz = tvec[0, 0, :]
                # calculate the coordinates of the aruco relative to the pump
                xyz = [round(xyz[0] * 1000 + self.y_offset, 2),
                       round(xyz[1] * 1000 + self.x_offset, 2),
                       round(xyz[2] * 1000, 2)]

                # cv2.putText(img, str(xyz[:2]), (0, 64), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                for i in range(rvec.shape[0]):
                    # draw the aruco on img
                    cv2.aruco.drawDetectedMarkers(img, filtered_corners)

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

