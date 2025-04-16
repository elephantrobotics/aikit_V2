import traceback
import cv2
from collections import deque
import time
import threading
import os, sys
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device, LED

from pymycobot.mycobot280 import MyCobot280
from yolov8_detect import YOLODetection


class Object_detect():

    def __init__(self, camera_x=170, camera_y=20):
        # inherit the parent class
        super(Object_detect, self).__init__()

        # declare mycobot 280 riscv
        self.mc = None
        # 移动角度
        self.move_angles = [
            [0.61, 45.87, -92.37, -41.3, 2.02, 9.58],  # init the point
            [18.8, -7.91, -54.49, -23.02, -0.79, -14.76],  # point to grab
        ]

        # 移动目标放置点角度
        self.move_target_angles = [
            [-24.87, -2.98, -92.46, 5.88, -3.07, -8.34],  # D Sorting area
            [-13.71, -52.11, -25.4, -4.57, -3.86, -7.73],  # C Sorting area
            [74.0, -18.1, -64.24, -9.84, -0.79, -9.49],  # A Sorting area
            [112.93, 3.16, -96.32, 0.87, 0.26, -9.75],  # B Sorting area
        ]

        self.robot_riscv = os.popen("ls /dev/ttyAMA*").readline()[:-1]

        Device.pin_factory = LGPIOFactory(chip=0)  # 显式指定/dev/gpiochip0
        # 初始化 GPIO 控制的设备
        self.valve = LED(72)  # 阀门

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

    # pump_control riscv
    def gpio_status(self, flag):
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
                if (time.time() - start_time) >= 3:
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
        # global is_picking
        print(color)
        self.mc.send_angles(self.move_angles[1], 50)
        self.check_position(self.move_angles[1], 0)

        self.mc.send_coords([x, y, 65, 179.87, -3.78, -62.75], 90, 1)
        data = [x, y, 65, 179.87, -3.78, -62.75]
        self.check_position(data, 1)

        # open pump
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
        self.mc.send_angles([tmp[0], -0.71, -74.49, -23.02, -0.79, tmp[5]],
                            50)  # [18.8, -7.91, -54.49, -23.02, -0.79, -14.76]
        self.check_position([tmp[0], -0.71, -74.49 - 20, -23.02, -0.79, tmp[5]], 0)

        self.mc.send_angles(self.move_target_angles[color], 50)
        self.check_position(self.move_target_angles[color], 0)

        # close pump
        self.gpio_status(False)
        time.sleep(0.5)

        self.mc.send_angles(self.move_angles[0], 50)
        self.check_position(self.move_angles[0], 0)

    # decide whether grab cube
    def decide_move(self, x, y, color):
        print('real_x_y:', round(x, 2), round(y, 2))
        self.cache_x = self.cache_y = 0
        # 调整吸泵吸取位置，y增大,向左移动;y减小,向右移动;x增大,前方移动;x减小,向后方移动
        self.move(round(x, 2), round(y, 2), color)

    # init mycobot
    def run(self):

        self.mc = MyCobot280(self.robot_riscv, 1000000)
        self.gpio_status(False)
        if self.mc.get_fresh_mode() != 0:
            self.mc.set_fresh_mode(0)
        self.mc.send_angles([0.61, 45.87, -92.37, -41.3, 2.02, 9.58], 50)
        self.check_position([0.61, 45.87, -92.37, -41.3, 2.02, 9.58], 0)

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
                # print(x1,x2,y1,y2)
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
            frame = frame[int(self.y2 * 0.78):int(self.y1 * 1.1),
                    int(self.x1 * 0.86):int(self.x2 * 1.08)]
        return frame


# 初始化是否正在抓取标志
is_picking = False
cooldown_counter = 0  # 新增冷却计数器（单位：帧）
detect_history = deque(maxlen=5)  # 存放最近5帧识别结果


def runs():
    global is_picking, cooldown_counter
    # open the camera
    cap_num = 20
    cap = cv2.VideoCapture(cap_num, cv2.CAP_V4L)

    if not cap.isOpened():
        cap.open()

    detect = Object_detect()
    yolo_detect = YOLODetection()

    # init mycobot
    detect.run()

    _init_ = 20  #
    init_num = 0
    nparams = 0

    while cv2.waitKey(1) < 0:
        # 读入每一帧
        ret, frame = cap.read()
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
                (detect.sum_x1) / 20.0,
                (detect.sum_y1) / 20.0,
                (detect.sum_x2) / 20.0,
                (detect.sum_y2) / 20.0,
            )
            detect.sum_x1 = detect.sum_x2 = detect.sum_y1 = detect.sum_y2 = 0
            init_num += 1
            continue

        # calculate params of the coords between cube and mycobot
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
            # calculate and set params of calculating real coord between cube and mycobot
            detect.set_params(
                (detect.sum_x1 + detect.sum_x2) / 20.0,
                (detect.sum_y1 + detect.sum_y2) / 20.0,
                abs(detect.sum_x1 - detect.sum_x2) / 10.0 + abs(detect.sum_y1 - detect.sum_y2) / 10.0
            )
            print('start yolov8 recognition.....')
            print("ok")
            continue
        if cooldown_counter > 0:
            cooldown_counter -= 1  # 冷却中，等待帧数减少
            cv2.imshow("figure", frame)
            continue
        # yolov8 detect result
        detect_result = yolo_detect.infer(frame)
        if detect_result is not None:
            x, y, class_ids, input_img = detect_result
            # 根据类别索引范围设置 detect.color
            for class_id in class_ids:
                if 0 <= class_id <= 19:
                    detect.color = 1
                elif 20 <= class_id <= 39:
                    detect.color = 2
                elif 40 <= class_id <= 59:
                    detect.color = 3
                elif 60 <= class_id <= 79:
                    detect.color = 4

            detect_history.append((x, y))

            if len(detect_history) == 5:
                dx = max([abs(detect_history[i][0] - detect_history[i - 1][0]) for i in range(1, 5)])
                dy = max([abs(detect_history[i][1] - detect_history[i - 1][1]) for i in range(1, 5)])

                if dx < 5 and dy < 5:  # 坐标变化小，认为物体静止
                    if not is_picking and cooldown_counter == 0:
                        print("物体稳定，准备抓取")
                        real_x, real_y = detect.get_position(x, y)
                        is_picking = True

                        def pick_task():
                            detect.decide_move(real_x, real_y, detect.color)
                            global is_picking, cooldown_counter
                            is_picking = False
                            cooldown_counter = 20  # 设置冷却帧数，防止连续触发

                        threading.Thread(target=pick_task).start()
                else:
                    # print("物体未稳定，等待...")
                    pass
            else:
                pass
                # print("帧数不足，继续观察中...")

            cv2.imshow("figure", input_img)
        else:
            cv2.imshow("figure", frame)

        if cooldown_counter > 0:
            cooldown_counter -= 1

        # close the window
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            sys.exit()


if __name__ == "__main__":
    runs()
