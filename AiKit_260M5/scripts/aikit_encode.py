import cv2 as cv
import numpy as np
from pymycobot.mypalletizer import MyPalletizer
import time
import serial
import serial.tools.list_ports


# y轴偏移量
pump_y = -45
# x轴偏移量
pump_x = -30

class Detect_marker():
    def __init__(self):
        
        # get real serial
        self.plist = [
            str(x).split(" - ")[0].strip() for x in serial.tools.list_ports.comports()
        ]
        
        #initialize MyCobot
        self.mc = None
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
            self.mc.set_basic_output(2, 0)
            self.mc.set_basic_output(5, 0)
        else:
            self.mc.set_basic_output(2, 1)
            self.mc.set_basic_output(5, 1)

    # Grasping motion
    def move(self, x, y, color):
        
        print(color)
        
        angles = [
            [0, 0, 0, 0],  # init the point
            [-29.0, 5.88, -4.92, -76.28],  # point to grab
            [17.4, -10.1, -87.27, 5.8],  # point to grab
        ]

        coords = [
            [166.4, -21.8, 219, 0.96], # 初始化点
            [111.6, 159, 221.5, -120], # A分拣区
            [-15.9, 164.6, 217.5, -119.35], # B分拣区  
            [232.5, -134.1, 197.7, -45.26], # C分拣区
            [132.6, -155.6, 211.8, -20.9], # D分拣区
            
        ]

        # send coordinates to move mycobot
        self.mc.send_angles(angles[0], 30)
        time.sleep(3)
        
        self.mc.send_coords([coords[0][0]+x, coords[0][1]+y, 160, 85], 20, 0)
        time.sleep(2)
        # self.mc.send_coords([coords[0][0]+x, coords[0][1]+y, 108, 85], 20, 0)
        self.mc.send_coords([coords[0][0]+x, coords[0][1]+y, 63, 85], 20, 0)
        time.sleep(2)
        
        # open pump
        self.pub_pump(True)
        
        self.mc.send_angle(2, 0, 20)
        time.sleep(0.3)
        self.mc.send_angle(3, -20, 20)
        time.sleep(2)
        
        # 抓取后放置区域
        self.mc.send_coords(coords[color], 20, 1) # coords[1] 为A分拣区，coords[2] 为B分拣区, coords[3] 为C分拣区，coords[4] 为D分拣区
        time.sleep(4)
        
        # close pump
        self.pub_pump(False)  
        time.sleep(5)
        
        self.mc.send_angles(angles[1], 20)
        time.sleep(1.5)
        self.mc.send_angles([-30, 0, 0, 0], 20)
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
            self.move(x+40, y+88, color)

    # init mycobot
    def init_mycobot(self):
        self.mc = MyPalletizer(self.plist[0], 115200)
        self.pub_pump(False)
        self.mc.send_angles([-29.0, 5.88, -4.92, -76.28], 30)
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
