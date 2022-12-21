import cv2
import numpy as np
import time
import os,sys
import serial
import serial.tools.list_ports
import math
from pymycobot.ultraArm import ultraArm


IS_CV_4 = cv2.__version__[0] == '4'
__version__ = "1.0"
# Adaptive seeed


class Object_detect():

    def __init__(self, camera_x = 255, camera_y = -10):
        # inherit the parent class
        super(Object_detect, self).__init__()
        # declare ultraArm
        self.ua = None
        
        # get real serial
        self.plist = [
        str(x).split(" - ")[0].strip() for x in serial.tools.list_ports.comports()
    ]

        # 移动角度
        self.move_angles = [
            [0.0, 0.0, 0.0],  # init the point
            # [19.48, 0.0, 0.0],  # point to grab
            [25.55, 0.0, 15.24],
            [0.0, 14.32, 0.0],  # point to grab
        ]

        # 移动坐标
        self.move_coords = [
            [141.53, 148.67, 43.73], # D Sorting area
            [248.52, 152.35, 53.45],    # C Sorting area
            [269.02, -161.65, 51.42],   # A Sorting area
            [146.8, -159.53, 50.44],     # B Sorting area
        ]
        # choose place to set cube
        self.color = 0
        # parameters to calculate camera clipping parameters
        self.x1 = self.x2 = self.y1 = self.y2 = 0
        # set cache of real coord
        self.cache_x = self.cache_y = 0

        # use to calculate coord between cube and ultraArm P300
        self.sum_x1 = self.sum_x2 = self.sum_y2 = self.sum_y1 = 0
        # The coordinates of the grab center point relative to the ultraArm P300
        self.camera_x, self.camera_y = camera_x, camera_y
        # The coordinates of the cube relative to the ultraArm P300
        self.c_x, self.c_y = 0, 0
        # The ratio of pixels to actual values
        self.ratio = 0
        # Get ArUco marker dict that can be detected.
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        # Get ArUco marker params.
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        # 初始化背景减法器
        self.mog =cv2.bgsegm.createBackgroundSubtractorMOG() 
 
    # 开启吸泵
    def pump_on(self):
        self.ua.set_gpio_state(0)

    # 停止吸泵
    def pump_off(self):
         self.ua.set_gpio_state(1)

    # Grasping motion
    def move(self, x, y, color):
        # send Angle to move ultraArm P300
        print(color)
        self.ua.set_angles(self.move_angles[2], 50)
        time.sleep(3)

        # send coordinates to move ultraArm
        self.ua.set_coords([x, -y, 65.51], 50)
        time.sleep(1.5)
        self.ua.set_coords([x, -y, -70], 50)
        time.sleep(2)


        # open pump
        self.pump_on()
        time.sleep(1.5)
        
        self.ua.set_angles(self.move_angles[0], 50)
        # self.ua.set_angle(2, 0, 50)
        # time.sleep(0.02)
        # self.ua.set_angle(3, 0, 50)
        time.sleep(0.5)

        self.ua.set_coords(self.move_coords[color], 50)
        # self.pub_marker(self.move_coords[color][0]/1000.0, self.move_coords[color]
        #                 [1]/1000.0, self.move_coords[color][2]/1000.0)
        time.sleep(7)
       
        # close pump
        self.pump_off()

        time.sleep(8)

        self.ua.set_angles(self.move_angles[1],50)
        time.sleep(1.5)
       

    # decide whether grab cube
    def decide_move(self, x, y, color):
        print(x, y, self.cache_x, self.cache_y)
        # detect the cube status move or run
        if (abs(x - self.cache_x) + abs(y - self.cache_y)) / 2 > 5:  # mm
            self.cache_x, self.cache_y = x, y
            return
        else:
            self.cache_x = self.cache_y = 0
            # 调整吸泵吸取位置，y增大,向左移动;y减小,向右移动;x增大,前方移动;x减小,向后方移动
            self.move(x, y, color)

    # init ultraArm P300
    def run(self):
     
        self.ua = ultraArm(self.plist[0], 115200)
        self.ua.go_zero()
        self.ua.set_angles([25.55, 0.0, 15.24], 50)
        time.sleep(3)

    # draw aruco
    def draw_marker(self, img, x, y):
        # draw rectangle on img
        cv2.rectangle( 
            img,
            (x - 20, y - 20),
            (x + 20, y + 20),
            (0, 255, 0),
            thickness=2,
            lineType=cv2.FONT_HERSHEY_COMPLEX,
        )
        # add text on rectangle
        cv2.putText(img, "({},{})".format(x, y), (x, y),
                    cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (243, 0, 0), 2,)

    # get points of two aruco
    def get_calculate_params(self, img):
        # Convert the image to a gray image
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Detect ArUco marker.
        corners, ids, rejectImaPoint = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )

        """
        Two Arucos must be present in the picture and in the same order.
        There are two Arucos in the Corners, and each aruco contains the pixels of its four corners.
        Determine the center of the aruco by the four corners of the aruco.
        """
        if len(corners) > 0:
            if ids is not None:
                if len(corners) <= 1 or ids[0] == 1:
                    return None
                x1 = x2 = y1 = y2 = 0
                point_11, point_21, point_31, point_41 = corners[0][0]
                x1, y1 = int((point_11[0] + point_21[0] + point_31[0] + point_41[0]) / 4.0), int(
                    (point_11[1] + point_21[1] + point_31[1] + point_41[1]) / 4.0)
                point_1, point_2, point_3, point_4 = corners[1][0]
                x2, y2 = int((point_1[0] + point_2[0] + point_3[0] + point_4[0]) / 4.0), int(
                    (point_1[1] + point_2[1] + point_3[1] + point_4[1]) / 4.0)
                return x1, x2, y1, y2
        return None

    # set camera clipping parameters
    def set_cut_params(self, x1, y1, x2, y2):
        self.x1 = int(x1)
        self.y1 = int(y1)
        self.x2 = int(x2)
        self.y2 = int(y2)
        print(self.x1, self.y1, self.x2, self.y2)

    # set parameters to calculate the coords between cube and ultraArm P300
    def set_params(self, c_x, c_y, ratio):
        self.c_x = c_x
        self.c_y = c_y
        self.ratio = 220.0/ratio

    # calculate the coords between cube and ultraArm P300
    def get_position(self, x, y):
        return ((y - self.c_y)*self.ratio + self.camera_x), ((x - self.c_x)*self.ratio + self.camera_y)

    """
    Calibrate the camera according to the calibration parameters.
    Enlarge the video pixel by 1.5 times, which means enlarge the video size by 1.5 times.
    If two ARuco values have been calculated, clip the video.
    """
    def transform_frame(self, frame):
        # enlarge the image by 1.5 times
        fx = 1.5
        fy = 1.5
        frame = cv2.resize(frame, (0, 0), fx=fx, fy=fy,
                           interpolation=cv2.INTER_CUBIC)
        if self.x1 != self.x2:
            # the cutting ratio here is adjusted according to the actual situation
            frame = frame[int(self.y2*0.6):int(self.y1*1.1),
                          int(self.x1*0.82):int(self.x2*1.08)]
        return frame
    
    # 检测物体的形状
    def shape_detect(self,img):
        x = 0
        y = 0
        
        Alpha = 65.6
        Gamma=-8191.5
        cal = cv2.addWeighted(img, Alpha,img, 0, Gamma)
        # 转换为灰度图片
        gray = cv2.cvtColor(cal, cv2.COLOR_BGR2GRAY)

        # a etching operation on a picture to remove edge roughness
        erosion = cv2.erode(gray, np.ones((2, 2), np.uint8), iterations=2)

        # the image for expansion operation, its role is to deepen the color depth in the picture
        dilation = cv2.dilate(erosion, np.ones(
            (1, 1), np.uint8), iterations=2)


        # 设定灰度图的阈值 175, 255
        _, threshold = cv2.threshold(dilation, 175, 255, cv2.THRESH_BINARY)
        # 边缘检测
        edges = cv2.Canny(threshold,50,100)
        # 检测物体边框
        contours,_ = cv2.findContours(
            edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        if len(contours)>0:
            for cnt in contours:
                # if 6000>cv2.contourArea(cnt) and cv2.contourArea(cnt)>4500:
                if cv2.contourArea(cnt)>5500:
                    objectType = None
                    peri = cv2.arcLength(cnt,True)
                    approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
                    objCor = len(approx)

                    boxes = [
                        box
                        for box in [cv2.boundingRect(c) for c in contours]
                        if min(img.shape[0], img.shape[1]) / 10
                        < min(box[2], box[3])
                        < min(img.shape[0], img.shape[1]) / 1
                    ]
                    if boxes:
                        for box in boxes:
                            x, y, w, h = box
                        # find the largest object that fits the requirements
                        c = max(contours, key=cv2.contourArea)
                        rect = cv2.minAreaRect(c)
                        box = cv2.boxPoints(rect)
                        box = np.int0(box)
                        cv2.drawContours(img, [box], 0, (153, 153, 0), 2)
                        x = int(rect[0][0])
                        y = int(rect[0][1])

                    if objCor==3:
                        objectType = ["Triangle","三角形"]
                        self.color = 3
                        cv2.drawContours(img, [cnt], 0, (0, 0, 255), 3)
                        
                    elif objCor==4:
                        box = cv2.boxPoints(rect)
                        box = np.int0(box)
                        _W = math.sqrt(math.pow((box[0][0] - box[1][0]), 2) + math.pow((box[0][1] - box[1][1]), 2))
                        _H = math.sqrt(math.pow((box[0][0] - box[3][0]), 2) + math.pow((box[0][1] - box[3][1]), 2))
                        aspRatio = _W/float(_H)
                        if 0.98 < aspRatio < 1.03:
                            objectType = ["Square","正方形"]
                            cv2.drawContours(img, [cnt], 0, (0, 0, 255), 3)
                            self.color=1
                        else:
                            objectType = ["Rectangle","长方形"]
                            cv2.drawContours(img, [cnt], 0, (0, 0, 255), 3)
                            self.color=2
                    elif objCor>=5:
                        objectType = ["Circle", "圆形"]
                        self.color=0
                        cv2.drawContours(img, [cnt], 0, (0, 0, 255), 3)
                    else:
                        pass
                    print(f"shape is {objectType[0]}(形状为{objectType[1]})")

        if abs(x) + abs(y) > 0:
            return x, y
        else:
            return None

        
if __name__ == "__main__":

    # open the camera
    cap_num = 1
    # cap = cv2.VideoCapture(cap_num, cv2.CAP_V4L)
    cap = cv2.VideoCapture(cap_num)
    if not cap.isOpened():
        cap.open(1)
    # init a class of Object_detect
    detect = Object_detect()
    # init ultraArm P300
    detect.run()

    _init_ = 20  
    init_num = 0
    nparams = 0
    num = 0
    real_sx = real_sy = 0
    while cv2.waitKey(1) < 0:
       # read camera
        _, frame = cap.read()
        # deal img
        frame = detect.transform_frame(frame)
        if _init_ > 0:
            _init_ -= 1
            continue

        # calculate the parameters of camera clipping
        if init_num < 20:
            if detect.get_calculate_params(frame) is None:
                cv2.imshow("figure", frame)
                continue
            else:
                x1, x2, y1, y2 = detect.get_calculate_params(frame)
                detect.draw_marker(frame, x1, y1)
                detect.draw_marker(frame, x2, y2)
                detect.sum_x1 += x1
                detect.sum_x2 += x2
                detect.sum_y1 += y1
                detect.sum_y2 += y2
                init_num += 1
                continue
        elif init_num == 20:
            detect.set_cut_params(
                (detect.sum_x1)/20.0,
                (detect.sum_y1)/20.0,
                (detect.sum_x2)/20.0,
                (detect.sum_y2)/20.0,
            )
            detect.sum_x1 = detect.sum_x2 = detect.sum_y1 = detect.sum_y2 = 0
            init_num += 1
            continue

        # calculate params of the coords between cube and ultraArm P300
        if nparams < 10:
            if detect.get_calculate_params(frame) is None:
                cv2.imshow("figure", frame)
                continue
            else:
                x1, x2, y1, y2 = detect.get_calculate_params(frame)
                detect.draw_marker(frame, x1, y1)
                detect.draw_marker(frame, x2, y2)
                detect.sum_x1 += x1
                detect.sum_x2 += x2
                detect.sum_y1 += y1
                detect.sum_y2 += y2
                nparams += 1
                continue
        elif nparams == 10:
            nparams += 1
            # calculate and set params of calculating real coord between cube and ultraArm P300
            detect.set_params(
                (detect.sum_x1+detect.sum_x2)/20.0,
                (detect.sum_y1+detect.sum_y2)/20.0,
                abs(detect.sum_x1-detect.sum_x2)/10.0 +
                abs(detect.sum_y1-detect.sum_y2)/10.0
            )
            print("ok")
            continue

        # get detect result
        # detect_result = detect.color_detect(frame)
        # print('调用检测')
        detect_result = detect.shape_detect(frame)
        # print("完成检测")
        if detect_result is None:
            cv2.imshow("figure", frame)
            continue
        else:
            x, y = detect_result
            # calculate real coord between cube and ultraArm P300
            real_x, real_y = detect.get_position(x, y)
            if num == 20:
                
                detect.decide_move(real_sx/20.0, real_sy/20.0, detect.color)
                num = real_sx = real_sy = 0

            else:
                num += 1
                real_sy += real_y
                real_sx += real_x

        cv2.imshow("figure", frame)

        # close the window
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            sys.exit()