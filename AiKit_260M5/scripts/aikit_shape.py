import cv2
import numpy as np
import time
import os,sys
import serial
import serial.tools.list_ports
import math
import platform
from pymycobot.mypalletizer260 import MyPalletizer260


from offset_utils import load_offset_from_txt

IS_CV_4 = cv2.__version__[0] == '4'
__version__ = "1.0"


offset_path = os.path.expanduser('~/AiKit_UI/libraries/offset/myPalletizer 260 for M5_shape.txt')

camera_x, camera_y, camera_z = load_offset_from_txt(offset_path)


class Object_detect():

    def __init__(self, camera_x=camera_x, camera_y=camera_y):
        # inherit the parent class
        super(Object_detect, self).__init__()
        
        # get real serial
        self.plist = [
            str(x).split(" - ")[0].strip() for x in serial.tools.list_ports.comports()
        ]
        
        # declare mypal260
        self.mc = None

        # 移动角度
        self.move_angles = [
            [-30.0, 0, 0, 7.28],  # point to grab
            [0, 0, 0, 0],  # point to grab
        ]

        self.new_move_coords_to_angles = [
            [-54, 14.15, 16.34, 0],  # D Sorting area
            [-35, 53.61, -44.64, 0],  # C Sorting area
            [34, 51.15, -40.34, 0],  # A Sorting area
            [52.38, 14.67, 12.83, -0.43],  # B Sorting area
        ]
        self.z_down_values = [95, 105, 105, 95]  # D, C, A, B
        # choose place to set cube
        self.color = 0
        # parameters to calculate camera clipping parameters
        self.x1 = self.x2 = self.y1 = self.y2 = 0
        # set cache of real coord
        self.cache_x = self.cache_y = 0

        # use to calculate coord between cube and mypal260
        self.sum_x1 = self.sum_x2 = self.sum_y2 = self.sum_y1 = 0
        # The coordinates of the grab center point relative to the mypal260
        self.camera_x, self.camera_y = camera_x, camera_y
        # The coordinates of the cube relative to the mypal260
        self.c_x, self.c_y = 0, 0
        # The ratio of pixels to actual values
        self.ratio = 0
        # Get ArUco marker dict that can be detected.
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        # Get ArUco marker params.
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        # 初始化背景减法器
        self.mog =cv2.bgsegm.createBackgroundSubtractorMOG()
        self.camera_z = camera_z
 
    # 开启吸泵 m5
    def pump_on(self):
        # 让5号位工作
        self.mc.set_basic_output(5, 0)
        time.sleep(0.05)

    # 停止吸泵 m5
    def pump_off(self):

        # 让5号位停止工作
        self.mc.set_basic_output(5, 1)
        time.sleep(0.05)
        self.mc.set_basic_output(2, 0)
        time.sleep(0.05)
        self.mc.set_basic_output(2, 1)
        time.sleep(0.05)

    # Grasping motion
    def move(self, x, y, color):
        # send Angle to move mypal260
        print(color)
        self.mc.send_angles(self.move_angles[1], 40)
        time.sleep(3)

        # send coordinates to move mypal260
        self.mc.send_coords([x, y, 100, 0], 60)
        time.sleep(1.5)
        self.mc.send_coords([x, y, self.camera_z, 0], 60)
        time.sleep(2.5)
        # open pump
        self.pump_on()
        time.sleep(1)

        self.mc.send_angle(2, 0, 20)
        time.sleep(0.3)
        self.mc.send_angle(3, -20, 20)
        time.sleep(2)

        self.mc.send_angles(self.new_move_coords_to_angles[color], 20)
        time.sleep(4)

        self.mc.send_coord(3, self.z_down_values[color], 25)
        time.sleep(2.5)
       
        # close pump
        self.pump_off()
        time.sleep(2)

    
        self.mc.send_angles(self.move_angles[0], 20)
        time.sleep(3)

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
            self.move(round(x, 2), round(y, 2), color)

    # init mypal260
    def run(self):
     
        self.mc = MyPalletizer260(self.plist[0], 115200)
        self.pump_off()
        self.mc.send_angles(self.move_angles[0], 40)
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

    # set parameters to calculate the coords between cube and mypal260
    def set_params(self, c_x, c_y, ratio):
        self.c_x = c_x
        self.c_y = c_y
        self.ratio = 235.0/ratio

    # calculate the coords between cube and mypal260
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
            frame = frame[int(self.y2*0.66):int(self.y1*1.1),
                          int(self.x1*0.86):int(self.x2*1.08)]
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
                if cv2.contourArea(cnt)>6900:
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
    if platform.system() == "Windows":
        cap_num = 1
        cap = cv2.VideoCapture(cap_num, cv2.CAP_DSHOW)
        if not cap.isOpened():
            cap.open(1)
    elif platform.system() == "Linux":
        cap_num = 0
        cap = cv2.VideoCapture(cap_num, cv2.CAP_V4L)
        if not cap.isOpened():
            cap.open()
    # init a class of Object_detect
    detect = Object_detect()
    # init mypal260
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

        # calculate params of the coords between cube and mypal260
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
            # calculate and set params of calculating real coord between cube and mypal260
            detect.set_params(
                (detect.sum_x1+detect.sum_x2)/20.0,
                (detect.sum_y1+detect.sum_y2)/20.0,
                abs(detect.sum_x1-detect.sum_x2)/10.0 +
                abs(detect.sum_y1-detect.sum_y2)/10.0
            )
            print("ok")
            continue

        # get detect result
        detect_result = detect.shape_detect(frame)
   
        if detect_result is None:
            cv2.imshow("figure", frame)
            continue
        else:
            x, y = detect_result
            # calculate real coord between cube and mypal260
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