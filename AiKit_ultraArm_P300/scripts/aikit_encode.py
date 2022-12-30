import cv2 as cv
import numpy as np
from pymycobot.ultraArm import ultraArm
import time
import serial
import serial.tools.list_ports


# y轴偏移量
pump_y = -45
# x轴偏移量
pump_x = -30

class Detect_marker():
    def __init__(self):
        #initialize MyCobot
        self.ua = None
        # get real serial
        self.plist = [
        str(x).split(" - ")[0].strip() for x in serial.tools.list_ports.comports()
    ]
        # set cache of real coord
        self.cache_x = self.cache_y = 0
        # Creating a Camera Object
        cap_num = 1
        self.cap = cv.VideoCapture(cap_num)
        
        # choose place to set cube
        self.color = 0
        
        # Get ArUco marker dict that can be detected.
        self.aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)
        # Get ArUco marker params.
        self.aruco_params = cv.aruco.DetectorParameters_create()
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
            self.ua.set_gpio_state(0)
        else:
            self.ua.set_gpio_state(1)

    # Grasping motion
    def move(self, x, y, color):
        
        print(color)
        
        angles = [
            [0.0, 0.0, 0.0],  # init the point
            # [19.48, 0.0, 0.0],  # point to grab
            [25.55, 0.0, 15.24],
            [0.0, 14.32, 0.0],  # point to grab
        ]

        coords = [
            [267.15, 0.0, 125.96], # 初始化点
            [269.02, -161.65, 51.42], # A分拣区
            [146.8, -159.53, 50.44], # B分拣区  
            [248.52, 152.35, 53.45], # C分拣区
            [141.53, 148.67, 43.73], # D分拣区
            [141.53, 148.67, 43.73], # D分拣区
            
        ]

        # send coordinates to move mycobot
        self.ua.set_angles(angles[2], 30)
        time.sleep(3)
        
        self.ua.set_coords([coords[0][0]+x, coords[0][1]-y, 65.51], 50)
        time.sleep(2)
        self.ua.set_coords([coords[0][0]+x, coords[0][1]-y, -70], 50)
        time.sleep(2)
        
        # open pump
        self.pub_pump(True)
        
        self.ua.set_angles(angles[0], 50)
        # self.ua.set_angle(2, 0, 50)
        # time.sleep(0.02)
        # self.ua.set_angle(3, 0, 50)
        time.sleep(0.5)
        
        # 抓取后放置区域
        self.ua.set_coords(coords[color], 50) # coords[1] 为A分拣区，coords[2] 为B分拣区, coords[3] 为C分拣区，coords[4] 为D分拣区
        time.sleep(7)
        
        # close pump
        self.pub_pump(False)  
        time.sleep(8)
        
        self.ua.set_angles(angles[1], 50)
        time.sleep(1.5)

    # decide whether grab cube
    def decide_move(self, x, y, color):

        print(x,y)
        # detect the cube status move or run
        if (abs(x - self.cache_x) + abs(y - self.cache_y)) / 2 > 5: # mm
            self.cache_x, self.cache_y = x, y
            return
        else:
            self.cache_x = self.cache_y = 0
            # 调整吸泵吸取位置，y增大,向左移动;y减小,向右移动;x增大,前方移动;x减小,向后方移动
            self.move(x+50, y+60, color)

    # init mycobot
    def init_mycobot(self):
        self.ua = ultraArm(self.plist[0])
        self.ua.go_zero()
        self.pub_pump(False)
        self.ua.set_angles([25.55, 0.0, 15.24], 50)
        time.sleep(2)
        
    def run(self):
        global pump_y, pump_x
        self.init_mycobot()
        print('ok')
        num = sum_x = sum_y = 0 
        while cv.waitKey(1) < 0:
            success, img = self.cap.read()
            if not success:
                print("It seems that the image cannot be acquired correctly.")
                break
            
            # img = cv.resize(img, None, fx=1.5, fy=1.5, interpolation=cv.INTER_CUBIC)
            # img = img[140:630, 240:730]

            # transfrom the img to model of gray
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            # Detect ArUco marker.
            corners, ids, rejectImaPoint = cv.aruco.detectMarkers(
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
            else:
                self.color = 5

            if len(corners) > 0:
                if ids is not None:
                    # get informations of aruco
                    ret = cv.aruco.estimatePoseSingleMarkers(
                        corners, 0.03, self.camera_matrix, self.dist_coeffs
                    )
                    # rvec:rotation offset,tvec:translation deviator
                    (rvec, tvec) = (ret[0], ret[1])
                    (rvec - tvec).any()
                    xyz = tvec[0, 0, :]
                    # calculate the coordinates of the aruco relative to the pump
                    xyz = [round(xyz[0]*1000+pump_y, 2), round(xyz[1]*1000+pump_x, 2), round(xyz[2]*1000, 2)]

                    # cv.putText(img, 'coords' + str(xyz), (0, 64), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv.LINE_AA)
                    for i in range(rvec.shape[0]):
			# draw the aruco on img
                        cv.aruco.drawDetectedMarkers(img, corners)
     
                        if num < 40 :
                            sum_x += xyz[1]
                            sum_y += xyz[0]
                            num += 1
                        elif num ==40 :
                            self.decide_move(sum_x/40.0, sum_y/40.0, self.color)
                            num = sum_x = sum_y = 0

            cv.imshow("encode_image", img)

if __name__ == "__main__":
    detect = Detect_marker()
    detect.run()
