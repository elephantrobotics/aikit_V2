from multiprocessing import Process, Pipe
import cv2
import numpy as np
import time
import datetime
import threading
import os,sys
import matplotlib.pyplot as plt 

from pymycobot.mycobot import MyCobot

IS_CV_4 = cv2.__version__[0] == '4'
__version__ = "1.0"  # Adaptive seeed


class Object_detect():
    global move_finsh

    def __init__(self, camera_x = 150, camera_y = 10):
        # inherit the parent class
        super(Object_detect, self).__init__()

        # declare mycobot 280pi
        self.mc = None
        # 移动角度
        self.move_angles = [
            [0.61, 45.87, -92.37, -41.3, 2.02, 9.58],  # init the point
            [18.8, -7.91, -54.49, -23.02, -0.79, -14.76],  # point to grab
        ]

        # 移动坐标
        self.move_coords = [
            [132.2, -136.9, 200.8, -178.24, -3.72, -107.17],  # D Sorting area
            [238.8, -124.1, 204.3, -169.69, -5.52, -96.52], # C Sorting area
            [115.8, 177.3, 210.6, 178.06, -0.92, -6.11], # A Sorting area
            [-6.9, 173.2, 201.5, 179.93, 0.63, 33.83], # B Sorting area
        ]
        
        # which robot: USB* is m5; ACM* is wio; AMA* is raspi
        self.robot_m5 = os.popen("ls /dev/ttyUSB*").readline()[:-1]
        self.robot_wio = os.popen("ls /dev/ttyACM*").readline()[:-1]
        self.robot_raspi = os.popen("ls /dev/ttyAMA*").readline()[:-1]
        self.robot_jes = os.popen("ls /dev/ttyTHS1").readline()[:-1]
        self.raspi = False
        if "dev" in self.robot_m5:
            self.Pin = [2, 5]
        elif "dev" in self.robot_wio:
            # self.Pin = [20, 21]
            self.Pin = [2, 5]

            # for i in self.move_coords:
            #     i[2] -= 20
        elif "dev" in self.robot_raspi or "dev" in self.robot_jes:
            import RPi.GPIO as GPIO
            GPIO.setwarnings(False)
            self.GPIO = GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(20, GPIO.OUT)
            GPIO.setup(21, GPIO.OUT)
            GPIO.output(20, 1)
            GPIO.output(21, 1)
            self.raspi = True
        if self.raspi:
            self.gpio_status(False)
   
        # choose place to set cube
        self.color = 0
        # parameters to calculate camera clipping parameters
        self.x1 = self.x2 = self.y1 = self.y2 = 0
        # set cache of real coord
        self.cache_x = self.cache_y = 0

        # use to calculate coord between cube and mycobot
        self.sum_x1 = self.sum_x2 = self.sum_y2 = self.sum_y1 = 0
        # The coordinates of the grab center point relative to the mycobot
        self.camera_x, self.camera_y = camera_x, camera_y
        # The coordinates of the cube relative to the mycobot
        self.c_x, self.c_y = 0, 0
        # The ratio of pixels to actual values
        self.ratio = 0
        # Get ArUco marker dict that can be detected.
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        # Get ArUco marker params.
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        # yolov5 model file path
        self.path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.modelWeights = self.path + "/scripts/yolov5s.onnx"
        if IS_CV_4:
            self.net = cv2.dnn.readNet(self.modelWeights)
        else:
            print('Load yolov5 model need the version of opencv is 4.')
            exit(0)
            
        # Constants.
        self.INPUT_WIDTH = 640   # 640
        self.INPUT_HEIGHT = 640  # 640
        self.SCORE_THRESHOLD = 0.5
        self.NMS_THRESHOLD = 0.45 
        self.CONFIDENCE_THRESHOLD = 0.45
        
        # Text parameters.
        self.FONT_FACE = cv2.FONT_HERSHEY_SIMPLEX
        self.FONT_SCALE = 0.7
        self.THICKNESS = 1
        
        # Colors.
        self.BLACK  = (0,0,0)
        self.BLUE   = (255,178,50)
        self.YELLOW = (0,255,255)
        
        '''加载类别名'''
        classesFile = self.path + "/scripts/coco.names"
        self.classes = None
        with open(classesFile, 'rt') as f:
            self.classes = f.read().rstrip('\n').split('\n')
       
        self.change_flag = False 

    # pump_control pi
    def gpio_status(self, flag):
        if flag:
            self.GPIO.output(20, 0)
            self.GPIO.output(21, 0)
        else:
            self.GPIO.output(20, 1)
            self.GPIO.output(21, 1)

    # 开启吸泵 m5
    def pump_on(self):
        # 让2号位工作
        self.mc.set_basic_output(2, 0)
        # 让5号位工作
        self.mc.set_basic_output(5, 0)

    # 停止吸泵 m5
    def pump_off(self):
        # 让2号位停止工作
        self.mc.set_basic_output(2, 1)
        # 让5号位停止工作
        self.mc.set_basic_output(5, 1)

    # Grasping motion
    def move(self, x, y, color):
        print(color)
        # send Angle to move mypal260
        self.mc.send_angles(self.move_angles[1], 25)
        time.sleep(3)

        # send coordinates to move mycobot
        self.mc.send_coords([x, y,  170.6, 179.87, -3.78, -62.75], 25, 1) # usb :rx,ry,rz -173.3, -5.48, -57.9
        time.sleep(3)
        
        # self.mc.send_coords([x, y, 150, 179.87, -3.78, -62.75], 25, 0)
        # time.sleep(3)

        self.mc.send_coords([x, y, 65, 179.87, -3.78, -62.75], 25, 0)
        # self.mc.send_coords([x, y, 103, 179.87, -3.78, -62.75], 25, 0)
        
        time.sleep(4)

        # open pump
        if "dev" in self.robot_m5 or "dev" in self.robot_wio:
            self.pump_on()
        elif "dev" in self.robot_raspi or "dev" in self.robot_jes:
            self.gpio_status(True)
        time.sleep(1.5)

        tmp = []
        while True:
            if not tmp: 
                tmp = self.mc.get_angles()    
            else:
                break
        time.sleep(0.5)

         # print(tmp)
        self.mc.send_angles([tmp[0], -0.71, -54.49, -23.02, -0.79, tmp[5]],25) # [18.8, -7.91, -54.49, -23.02, -0.79, -14.76]
        time.sleep(3)



        self.mc.send_coords(self.move_coords[color], 25, 1)
        time.sleep(4)

        # close pump
        if "dev" in self.robot_m5 or "dev" in self.robot_wio:
            self.pump_off()
        elif "dev" in self.robot_raspi or "dev" in self.robot_jes:
            self.gpio_status(False)
        time.sleep(5)

        self.mc.send_angles(self.move_angles[0], 25)
        time.sleep(4.5)
        print('请按空格键打开摄像头进行下一次图像存储和识别')
        print('Please press the space bar to open the camera for the next image storage and recognition')

    # decide whether grab cube
    def decide_move(self, x, y, color):
        # print(x, y, self.cache_x, self.cache_y)
        # detect the cube status move or run
        #if (abs(x - self.cache_x) + abs(y - self.cache_y)) / 2 > 5:  # mm
            #self.cache_x, self.cache_y = x, y
            #return
        #else:
        self.cache_x = self.cache_y = 0
        # 调整吸泵吸取位置，y增大,向左移动;y减小,向右移动;x增大,前方移动;x减小,向后方移动
   
        self.move(x, y, color)
      

    # init mycobot
    def run(self):
    
        if "dev" in self.robot_wio :
            self.mc = MyCobot(self.robot_wio, 115200) 
        elif "dev" in self.robot_m5:
            self.mc = MyCobot(self.robot_m5, 115200) 
        elif "dev" in self.robot_raspi:
            self.mc = MyCobot(self.robot_raspi, 1000000)
        if not self.raspi:
            self.pub_pump(False, self.Pin)
        self.mc.send_angles([0.61, 45.87, -92.37, -41.3, 2.02, 9.58], 20)
        time.sleep(2.5)


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
        cv2.putText(
            img,
            "({},{})".format(x, y),
            (x, y),
            cv2.FONT_HERSHEY_COMPLEX_SMALL,
            1,
            (243, 0, 0),
            2,
        )

    # get points of two aruco
    def get_calculate_params(self, img):
        # Convert the image to a gray image
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Detect ArUco marker.
        corners, ids, rejectImaPoint = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params)

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
                x1, y1 = int(
                    (point_11[0] + point_21[0] + point_31[0] + point_41[0]) /
                    4.0), int(
                        (point_11[1] + point_21[1] + point_31[1] + point_41[1])
                        / 4.0)
                point_1, point_2, point_3, point_4 = corners[1][0]
                x2, y2 = int(
                    (point_1[0] + point_2[0] + point_3[0] + point_4[0]) /
                    4.0), int(
                        (point_1[1] + point_2[1] + point_3[1] + point_4[1]) /
                        4.0)
                #print(x1,x2,y1,y2)
                return x1, x2, y1, y2
        return None

    # set camera clipping parameters
    def set_cut_params(self, x1, y1, x2, y2):
        self.x1 = int(x1)
        self.y1 = int(y1)
        self.x2 = int(x2)
        self.y2 = int(y2)
        print(self.x1, self.y1, self.x2, self.y2)

    # set parameters to calculate the coords between cube and mycobot
    def set_params(self, c_x, c_y, ratio):
        self.c_x = c_x
        self.c_y = c_y
        self.ratio = 220.0 / ratio

    # calculate the coords between cube and mycobot
    def get_position(self, x, y):
        return ((y - self.c_y) * self.ratio +
                self.camera_x), ((x - self.c_x) * self.ratio + self.camera_y)

    """
    Calibrate the camera according to the calibration parameters.
    Enlarge the video pixel by 1.5 times, which means enlarge the video size by 1.5 times.
    If two ARuco values have been calculated, clip the video.
    """

    def transform_frame(self, frame):
        # enlarge the image by 1.5 times
        fx = 1.5
        fy = 1.5
        frame = cv2.resize(frame, (0, 0),
                           fx=fx,
                           fy=fy,
                           interpolation=cv2.INTER_CUBIC)
        if self.x1 != self.x2:
            # the cutting ratio here is adjusted according to the actual situation
            frame = frame[int(self.y2 * 0.2):int(self.y1 * 1.15),
                          int(self.x1 * 0.4):int(self.x2 * 1.15)]
        return frame

        '''绘制类别'''
    def draw_label(self,img,label,x,y):
        text_size = cv2.getTextSize(label,self.FONT_FACE,self.FONT_SCALE,self.THICKNESS)
        dim,baseline = text_size[0],text_size[1]
        cv2.rectangle(img,(x,y),(x+dim[0],y+dim[1]+baseline),(0,0,0),cv2.FILLED)
        cv2.putText(img,label,(x,y+dim[1]),self.FONT_FACE,self.FONT_SCALE,self.YELLOW,self.THICKNESS)

    '''
    预处理
    将图像和网络作为参数。
    - 首先，图像被转换为​​ blob。然后它被设置为网络的输入。
    - 该函数getUnconnectedOutLayerNames()提供输出层的名称。
    - 它具有所有层的特征，图像通过这些层向前传播以获取检测。处理后返回检测结果。
    '''
    def pre_process(self,input_image,net):
        blob = cv2.dnn.blobFromImage(input_image,1/255,(self.INPUT_HEIGHT,self.INPUT_WIDTH),[0,0,0], 1, crop=False)
        # Sets the input to the network.
        net.setInput(blob)
        # Run the forward pass to get output of the output layers.
        outputs = net.forward(net.getUnconnectedOutLayersNames())
        return outputs
    '''后处理
    过滤 YOLOv5 模型给出的良好检测
    步骤
    - 循环检测。
    - 过滤掉好的检测。
    - 获取最佳班级分数的索引。
    - 丢弃类别分数低于阈值的检测。
    '''
    
    # detect object
    def post_process(self,input_image):
        class_ids = []
        confidences = []
        boxes = []
        blob = cv2.dnn.blobFromImage(input_image,1/255,(self.INPUT_HEIGHT,self.INPUT_WIDTH),[0,0,0], 1, crop=False)
        # Sets the input to the network.
        self.net.setInput(blob)
        # Run the forward pass to get output of the output layers.
        outputs = self.net.forward(self.net.getUnconnectedOutLayersNames())
        
        rows = outputs[0].shape[1]
        image_height ,image_width = input_image.shape[:2]
        
        x_factor = image_width/self.INPUT_WIDTH
        y_factor = image_height/self.INPUT_HEIGHT
        # 像素中心点
        cx = 0 
        cy = 0 
        # 循环检测
        try:
            for r in range(rows):
                row = outputs[0][0][r]
                confidence = row[4]
                if confidence>self.CONFIDENCE_THRESHOLD:
                    classes_scores = row[5:]
                    class_id = np.argmax(classes_scores)
                    if (classes_scores[class_id]>self.SCORE_THRESHOLD):
                        confidences.append(confidence)
                        class_ids.append(class_id)
                        cx,cy,w,h = row[0],row[1],row[2],row[3]
                        left = int((cx-w/2)*x_factor)
                        top = int((cy - h/2) * y_factor)
                        width = int(w * x_factor)
                        height = int(h * y_factor)
                        box = np.array([left, top, width, height])
                        boxes.append(box)
                        
                        

                        '''非极大值抑制来获取一个标准框'''
                        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.CONFIDENCE_THRESHOLD, self.NMS_THRESHOLD)
                       
                        for i in indices:
                            box = boxes[i]
                            left = box[0]
                            top = box[1]
                            width = box[2]
                            height = box[3]
                                    
                            # 描绘标准框
                            cv2.rectangle(input_image, (left, top), (left + width, top + height),self.BLUE, 3*self.THICKNESS)
                           
                            # 像素中心点
                            cx = left+(width)//2 
                            cy = top +(height)//2
                           
                            cv2.circle(input_image, (cx,cy),  5,self.BLUE, 10)
                          
                            

                            # 检测到的类别                      
                            label = "{}:{:.2f}".format(self.classes[class_ids[i]], confidences[i])             
                            # 绘制类real_sx, real_sy, detect.color)
                             
                            self.draw_label(input_image, label, left, top)
                            
                #cv2.imshow("nput_frame",input_image)
           # return input_image
        except Exception as e:
            print(e)
            exit(0)

        if cx + cy > 0:
            return cx, cy, input_image
        else:
            return None


status = True
def camera_status():
    global status
    status = True
    cap_num = 0
    cap = cv2.VideoCapture(cap_num)
    
    
    
def runs():
    global status

    detect = Object_detect()

    # init mycobot
    detect.run()
    flag = True
    count = 0
    _init_ = 20  # 
    init_num = 0
    nparams = 0
    num = 0
    real_sx = real_sy = 0
        
    # flag = False 
    import time
    import glob
    
    # yolov5 img path
    path_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    path_img = path_dir + '/res/yolov5_detect.png'
    # open the camera
    cap_num = 0
    cap = cv2.VideoCapture(cap_num)
    print("*  热键(请在摄像头的窗口使用):                   *")
    print("*  hotkey(please use it in the camera window): *")
    print("*  z: 拍摄图片(take picture)                    *")
    print("*  q: 退出(quit)                                *")

    while cv2.waitKey(1)<0:
        if not status:
            cap = cv2.VideoCapture(cap_num)
            status = True
            print("请将可识别物体放置摄像头窗口进行拍摄")
            print("Please place an identifiable object in the camera window for shooting")
            print("*  热键(请在摄像头的窗口使用):                   *")
            print("*  hotkey(please use it in the camera window): *")
            print("*  z: 拍摄图片(take picture)                    *")
            print("*  q: 退出(quit)                                *")
        # 读入每一帧
        ret, frame = cap.read()

        cv2.imshow("capture", frame)
                                      

            # 存储
        input = cv2.waitKey(1) & 0xFF
        if input == ord('q'):
            print('quit')
            break
        elif input == ord('z'):
            print("请截取白色识别区域的部分")
            print("Please capture the part of the white recognition area")
            # 选择ROI
            roi = cv2.selectROI(windowName="capture",
                        img=frame,
                        showCrosshair=False,
                        fromCenter=False)
            x, y, w, h = roi
            print(roi)
            if roi != (0, 0, 0, 0):
                crop = frame[y:y+h, x:x+w]
                cv2.imwrite(path_img, crop)
                cap.release()
                cv2.destroyAllWindows()
                status=False
            
            
            while True:
                frame = cv2.imread(path_img)
                
                #frame = frame[170:700, 230:720]
                frame = detect.transform_frame(frame)
                
                cv2.imshow('oringal',frame)
                
                if _init_ > 0:
                    _init_-=1
                    continue
                # calculate the parameters of camera clipping
                if init_num < 20:
                    if detect.get_calculate_params(frame) is None:
                        cv2.imshow("figure",frame)
                        continue
                    else:
                        x1,x2,y1,y2 = detect.get_calculate_params(frame)
                        detect.draw_marker(frame,x1,y1)
                        detect.draw_marker(frame,x2,y2)
                        detect.sum_x1+=x1
                        detect.sum_x2+=x2
                        detect.sum_y1+=y1
                        detect.sum_y2+=y2
                        init_num+=1
                        continue
                elif init_num==20:
                    detect.set_cut_params(
                        (detect.sum_x1)/20.0, 
                        (detect.sum_y1)/20.0, 
                        (detect.sum_x2)/20.0, 
                        (detect.sum_y2)/20.0, 
                    )
                    detect.sum_x1 = detect.sum_x2 = detect.sum_y1 = detect.sum_y2 = 0
                    init_num+=1
                    continue

                # calculate params of the coords between cube and mycobot
                if nparams < 10:
                    if detect.get_calculate_params(frame) is None:
                        cv2.imshow("figure",frame)
                        continue
                    else:
                        x1,x2,y1,y2 = detect.get_calculate_params(frame)
                        detect.draw_marker(frame,x1,y1)
                        detect.draw_marker(frame,x2,y2)
                        detect.sum_x1+=x1
                        detect.sum_x2+=x2
                        detect.sum_y1+=y1
                        detect.sum_y2+=y2
                        nparams+=1
                        continue
                elif nparams==10:
                    nparams+=1
                    # calculate and set params of calculating real coord between cube and mycobot
                    detect.set_params(
                        (detect.sum_x1+detect.sum_x2)/20.0, 
                        (detect.sum_y1+detect.sum_y2)/20.0, 
                        abs(detect.sum_x1-detect.sum_x2)/10.0+abs(detect.sum_y1-detect.sum_y2)/10.0
                    )
                    print('start yolov5 recognition.....')
                    print("ok") 
                    continue
                # yolov5 detect result        
                detect_result = detect.post_process(frame)
                print('pick...')
                if detect_result:
                    x, y, input_img = detect_result
                    
                    real_x, real_y = detect.get_position(x, y)
                    print("real_x,real_y:", (round(real_x, 2), round(real_y, 2)))
                  
                    a = threading.Thread(target=lambda:detect.decide_move(real_x, real_y, detect.color))
                    a.start()
                  
                    cv2.imshow("detect_done",input_img)
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()
                break
                

if __name__ == "__main__":

    runs()
    

































