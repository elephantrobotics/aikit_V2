#encoding: UTF-8
import traceback

import cv2
import numpy as np
from pymycobot.mycobot280 import MyCobot280
# import RPi.GPIO as GPIO
import time
import os
import serial
import serial.tools.list_ports
import platform


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
        
        # 相机延后到机械臂回初始位后再打开，避免挡住 ArUco
        self.cap = None
            
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
        self.dist_coeffs = np.array(([[3.41360787e-01, -2.52114260e+00, -1.28012469e-03,  6.70503562e-03,
             2.57018000e+00]]))
    
    # 控制吸泵      
    def pub_pump(self, flag):
        if flag:
            self.mc.set_basic_output(2, 0)
            self.mc.set_basic_output(5, 0)
        else:
            self.mc.set_basic_output(2, 1)
            self.mc.set_basic_output(5, 1)

    @staticmethod
    def _angle_diff(a, b):
        """最短角差（度），兼容姿态角在 ±180 附近跳变。"""
        return abs((float(a) - float(b) + 180.0) % 360.0 - 180.0)

    def _is_reached(self, current, target, ids, angle_tol=2.0, coord_tol=5.0, orient_tol=5.0):
        """
        根据反馈自行判断是否到位（不依赖 is_in_position）。
        关节有虚位时，2°/数毫米容差比固件到位判定更符合真机。
        :param ids: 0-角度，1-坐标
        """
        # get_angles/get_coords 失败时可能返回 -1（int），不能直接 len()
        if not isinstance(current, (list, tuple)) or len(current) < len(target):
            return False
        if ids == 0:
            return all(
                self._angle_diff(current[i], target[i]) <= angle_tol
                for i in range(len(target))
            )
        # 坐标：XYZ 用毫米，rx/ry/rz 用角度
        for i, value in enumerate(target):
            cur = current[i]
            if i < 3:
                if abs(float(cur) - float(value)) > coord_tol:
                    return False
            else:
                if self._angle_diff(cur, value) > orient_tol:
                    return False
        return True

    def check_position(
        self,
        data,
        ids,
        timeout=5.0,
        angle_tol=2.0,
        coord_tol=5.0,
        orient_tol=5.0,
        stable_count=2,
    ):
        """
        循环检测是否到位某个位置（自判反馈 + 短超时保底）
        :param data: 角度或者坐标
        :param ids: 角度-0，坐标-1
        :param timeout: 超时时间（秒）
        :param angle_tol: 关节角到位容差（度），默认 2°，用于吸收虚位
        :param coord_tol: 坐标 XYZ 到位容差（mm）
        :param orient_tol: 坐标姿态到位容差（度）
        :param stable_count: 连续满足容差的次数，避免刚进入误差带就判定
        :return: True 确认到位；False 超时或查询失败后继续后续流程
        """
        try:
            start_time = time.time()
            ok_count = 0
            error_count = 0
            while time.time() - start_time < timeout:
                current = self.mc.get_angles() if ids == 0 else self.mc.get_coords()
                if not isinstance(current, (list, tuple)):
                    error_count += 1
                    if error_count >= 5:
                        print("check_position feedback query failed repeatedly, target=", data)
                        return False
                    time.sleep(0.1)
                    continue
                error_count = 0
                if self._is_reached(
                    current, data, ids,
                    angle_tol=angle_tol,
                    coord_tol=coord_tol,
                    orient_tol=orient_tol,
                ):
                    ok_count += 1
                    if ok_count >= stable_count:
                        time.sleep(0.15)  # 短暂停稳，消化虚位抖动
                        return True
                else:
                    ok_count = 0
                time.sleep(0.1)
            print("check_position timeout, target=", data)
            return False
        except Exception as e:
            e = traceback.format_exc()
            print(e)
            return False

            time.sleep(0.1)
            print("check_position timeout, target=", data)
            return False
        except Exception as e:
            e = traceback.format_exc()
            print(e)
            return False
    # Grasping motion
    def move(self, x, y, color):
        
        print(color)
        
        angles = [
            [0.61, 45.87, -92.37, -41.3, 2.02, 9.58], # init to point
            [18.8, -7.91, -54.49, -23.02, -0.79, -14.76],
        ]

        coords = [
            [145.0, -65.5, 280.1, 178.99, 7.67, -179.9], # 初始化点 init point
            [115.8, 177.3, 210.6, 178.06, -0.92, -6.11], # A分拣区 A sorting area
            [-6.9, 173.2, 201.5, 179.93, 0.63, 33.83], # B分拣区  B sorting area
            [238.8, -124.1, 204.3, -169.69, -5.52, -96.52], # C分拣区 C sorting area
            [132.2, -136.9, 200.8, -178.24, -3.72, -107.17],  # D分拣区 D sorting area   
            
        ]
        print('real_x, real_y:', round(coords[0][0]+x, 2), round(coords[0][1]+y, 2))
        # send coordinates to move mycobot
        self.mc.send_angles(angles[1], 25)
        self.check_position(angles[1], 0)
        
        # self.mc.send_coords([coords[0][0]+x, coords[0][1]+y, 240, 178.99, 5.38, -179.9], 20, 0)
        # time.sleep(2)
        self.mc.send_coords([coords[0][0]+x, coords[0][1]+y, 200, 178.99, -3.78, -62.9], 40, 1)

        # self.mc.send_coords([coords[0][0]+x, coords[0][1]+y, 105, 178.99, -3.78, -62.9], 40, 1)
        # time.sleep(2)
        self.mc.send_coords([coords[0][0]+x, coords[0][1]+y, 65.5, 178.99, -3.78, -62.9], 40, 1)
        data = [coords[0][0]+x, coords[0][1]+y, 65.5, 178.99, -3.78, -62.9]
        self.check_position(data, 1)
        
        # open pump
        self.pub_pump(True)
        time.sleep(1.5)
        
        tmp = []
        start_time = time.time()
        while time.time() - start_time < 3.0:
            cur_angles = self.mc.get_angles()
            if isinstance(cur_angles, (list, tuple)) and cur_angles:
                tmp = cur_angles
                break
            time.sleep(0.1)
        if not tmp:
            print("get_angles timeout, use fallback angles")
            tmp = angles[1]
        time.sleep(0.5)
        
        # print(tmp)
        self.mc.send_angles([tmp[0], -0.71, -54.49, -23.02, -0.79, tmp[5]], 25) # [18.8, -7.91, -54.49, -23.02, -0.79, -14.76]
        self.check_position([tmp[0], -0.71, -54.49, -23.02, -0.79, tmp[5]], 0)
        # 抓取后放置区域
        self.mc.send_coords(coords[color], 40, 1) # coords[1] 为A分拣区，coords[2] 为B分拣区, coords[3] 为C分拣区，coords[4] 为D分拣区
        self.check_position(coords[color], 1)
        
        # close pump
        self.pub_pump(False)
        time.sleep(0.5)

        self.mc.send_angles(angles[0], 25)
        self.check_position(angles[0], 0)

    # decide whether grab cube
    def decide_move(self, x, y, color):

        # print(x,y)
        # detect the cube status move or run
        if (abs(x - self.cache_x) + abs(y - self.cache_y)) / 2 > 5: # mm
            self.cache_x, self.cache_y = x, y
            return
        else:
            self.cache_x = self.cache_y = 0
            # 调整吸泵吸取位置，y增大,向左移动;y减小,向右移动;x增大,前方移动;x减小,向后方移动
            self.move(x-15, y+145, color)

    # init mycobot
    def init_mycobot(self):
        self.mc = MyCobot280(self.plist[0], 115200)
        self.pub_pump(False)
        self.mc.send_angles([0.61, 45.87, -92.37, -41.3, 2.02, 9.58], 20)
        self.check_position([0.61, 45.87, -92.37, -41.3, 2.02, 9.58], 0)

    def open_camera(self):
        """机械臂回到初始位后再开相机，避免挡住 ArUco 底板。"""
        if platform.system() == "Windows":
            cap_num = 1
        else:
            cap_num = 0
        self.cap = cv2.VideoCapture(cap_num)
        self.cap.set(3, 640)
        self.cap.set(4, 480)

    def run(self):
        global pump_y, pump_x
        self.init_mycobot()
        self.open_camera()
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
                    xyz = [round(xyz[0]*1000+pump_y, 2), round(xyz[1]*1000+pump_x, 2), round(xyz[2]*1000, 2)]

                    # cv2.putText(img, str(xyz[:2]), (0, 64), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    for i in range(rvec.shape[0]):
			# draw the aruco on img
                        cv2.aruco.drawDetectedMarkers(img, corners)
                        
                        if num < 40 :
                            sum_x += xyz[1]
                            sum_y += xyz[0]
                            num += 1
                        elif num ==40 :
                            self.decide_move(sum_x/40.0, sum_y/40.0, self.color)
                            num = sum_x = sum_y = 0

            cv2.imshow("encode_image", img)

if __name__ == "__main__":
    detect = Detect_marker()
    detect.run()

