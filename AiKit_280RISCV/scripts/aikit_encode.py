#encoding: UTF-8
import traceback

import cv2
import numpy as np
from pymycobot.mycobot280 import MyCobot280
import time
import os
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device, LED


# y轴偏移量
pump_y = 20
# x轴偏移量
pump_x = 170

class Detect_marker():
    def __init__(self):
        
        # set cache of real coord
        self.cache_x = self.cache_y = 0
        
        self.robot_riscv = os.popen("ls /dev/ttyAMA*").readline()[:-1]

        Device.pin_factory = LGPIOFactory(chip=0) # 显式指定/dev/gpiochip0
        
        # 初始化 GPIO 控制的设备
        self.valve = LED(72)  # 阀门

        self.pub_pump(False)
        
        # Creating a Camera Object
        cap_num = 20
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
        self.dist_coeffs = np.array(([[3.41360787e-01, -2.52114260e+00, -1.28012469e-03,  6.70503562e-03,
             2.57018000e+00]]))
    
    # 控制吸泵      
    def pub_pump(self, flag):
        if flag:
            self.valve.off()
        else:
            self.valve.on()

    def check_position(self, data, ids):
        """
        循环检测是否到位某个位置
        :param data: 角度或者坐标
        :param ids: 角度-0，坐标-1
        :return:
        """
        try:
            start_time = time.time()
            while True:
                # 超时检测
                if (time.time() - start_time) >= 5:
                    break
                res = self.mc.is_in_position(data, ids)
                # print('res', res)
                if res == 1:
                    time.sleep(0.1)
                    break
                time.sleep(0.1)
        except Exception as e:
            e = traceback.format_exc()
            print(e)

    # Grasping motion
    def move(self, x, y, color):
        
        print(color)
        
        angles = [
            [0.61, 45.87, -92.37, -41.3, 2.02, 9.58], # init to point
            [18.8, -7.91, -54.49, -23.02, -0.79, -14.76],
        ]

        # 移动目标放置点角度
        move_target_angles = [
            [74.0, -18.1, -64.24, -9.84, -0.79, -9.49],  # A Sorting area
            [112.93, 3.16, -96.32, 0.87, 0.26, -9.75],  # B Sorting area
            [-13.71, -52.11, -25.4, -4.57, -3.86, -7.73],  # C Sorting area
            [-24.87, -2.98, -92.46, 5.88, -3.07, -8.34],  # D Sorting area
        ]
        print('real_x, real_y:', round(x, 2), round(y, 2))
        # 过渡动作
        # send coordinates to move mycobot
        self.mc.send_angles(angles[1], 50)
        self.check_position(angles[1], 0)
        
        self.mc.send_coords([x, y, 65.5, 178.99, -3.78, -62.9], 60, 1)
        data = [x, y, 65.5, 178.99, -3.78, -62.9]
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
        self.mc.send_angles([tmp[0], -0.71, -74.49, -23.02, -0.79, tmp[5]],50) # [18.8, -7.91, -54.49, -23.02, -0.79, -14.76]
        self.check_position([tmp[0], -0.71, -74.49 - 20, -23.02, -0.79, tmp[5]], 0)

        # 抓取后放置区域
        self.mc.send_angles(move_target_angles[color-1], 50) # coords[1] 为A分拣区，coords[2] 为B分拣区, coords[3] 为C分拣区，coords[4] 为D分拣区
        self.check_position(move_target_angles[color-1], 0)
        
        # close pump
        self.pub_pump(False)
        time.sleep(0.5)

        self.mc.send_angles(angles[0], 50)
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
            self.move(round(x, 2), round(y, 2), color)

    # init mycobot
    def init_mycobot(self):
        
        self.mc = MyCobot280(self.robot_riscv, 1000000)
        self.pub_pump(False)
        if self.mc.get_fresh_mode() != 0:
            self.mc.set_fresh_mode(0)
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
            
            # Determine the placement point of the QR code
            #if ids == np.array([[1]]):
            if np.array_equal(ids, np.array([[1]])):
                self.color = 1
            #elif ids == np.array([[2]]):
            elif np.array_equal(ids, np.array([[2]])):
                self.color = 2
            #elif ids == np.array([[3]]):
            elif np.array_equal(ids, np.array([[3]])):
                self.color = 3
            #elif ids == np.array([[4]]):
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

