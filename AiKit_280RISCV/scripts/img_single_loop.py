import traceback
from multiprocessing import Process, Pipe
import cv2
import numpy as np
import time
import os, sys

from pymycobot.mycobot280 import MyCobot280
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device, LED

IS_CV_4 = cv2.__version__[0] == '4'
__version__ = "1.0"  # Adaptive seeed


class Object_detect():

    def __init__(self, camera_x=170, camera_y=10):
        # inherit the parent class
        super(Object_detect, self).__init__()

        # declare mycobot 280 musepi
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

        Device.pin_factory = LGPIOFactory(chip=0) # 显式指定/dev/gpiochip0
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

    # pump_control musepi
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

        self.mc.send_angles(self.move_angles[1], 50)
        self.check_position(self.move_angles[1], 0)

        # send coordinates to move mycobot
        self.mc.send_coords([x, y, 65, 179.87, -3.78, -62.75], 60, 1)
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
        self.mc.send_angles([tmp[0], -0.71, -74.49, -23.02, -0.79, tmp[5]], 50) # [18.8, -7.91, -54.49, -23.02, -0.79, -14.76]
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
        print(x, y, self.cache_x, self.cache_y)
        # detect the cube status move or run
        if (abs(x - self.cache_x) + abs(y - self.cache_y)) / 2 > 5:  # mm
            self.cache_x, self.cache_y = x, y
            return
        else:
            self.cache_x = self.cache_y = 0
            # 调整吸泵吸取位置，y增大,向左移动;y减小,向右移动;x增大,前方移动;x减小,向后方移动

            self.move(x - 5, y - 5, color)

    # init mycobot
    def run(self):

        self.mc = MyCobot280(self.robot_riscv, 1000000)
        self.gpio_status(False)
        if self.mc.get_fresh_mode() != 0:
            self.mc.set_fresh_mode(0)
        self.mc.send_angles([0.61, 45.87, -92.37, -41.3, 2.02, 9.58], 20)
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
                    int(self.x1 * 0.7):int(self.x2 * 1.15)]
        return frame

    # according the class_id to get object name
    def id_class_name(self, class_id):
        for key, value in self.labels.items():
            if class_id == int(key):
                return value

    # detect object
    def obj_detect(self, img, goal, kp_img, desc_img, kp_list, desc_list, connection):
        i = 0
        MIN_MATCH_COUNT = 5
        # sift = cv2.xfeatures2d.SIFT_create()

        # find the keypoints and descriptors with SIFT
        # kp = []
        # des = []
        kp = kp_list
        des = desc_list

        kp2, des2 = kp_img, desc_img

        # FLANN parameters
        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)  # or pass empty dictionary
        flann = cv2.FlannBasedMatcher(index_params, search_params)

        x, y = 0, 0
        try:
            for i in range(len(des)):
                matches = flann.knnMatch(des[i], des2, k=2)
                # store all the good matches as per Lowe's ratio test.  根据Lowe比率测试存储所有良好匹配项。
                good = []
                for m, n in matches:
                    if m.distance < 0.7 * n.distance:
                        good.append(m)

                # When there are enough robust matching point pairs 当有足够的健壮匹配点对（至少个MIN_MATCH_COUNT）时
                if len(good) > MIN_MATCH_COUNT:
                    # extract corresponding point pairs from matching 从匹配中提取出对应点对
                    # query index of small objects, training index of scenarios 小对象的查询索引，场景的训练索引
                    src_pts = np.float32([kp[i][m.queryIdx].pt
                                          for m in good]).reshape(-1, 1, 2)
                    dst_pts = np.float32([kp2[m.trainIdx].pt
                                          for m in good]).reshape(-1, 1, 2)

                    # Using matching points to find homography matrix in cv2.ransac 利用匹配点找到CV2.RANSAC中的单应矩阵
                    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,
                                                 5.0)
                    matchesMask = mask.ravel().tolist()
                    # Calculate the distortion of image, that is the corresponding position in frame 计算图1的畸变，也就是在图2中的对应的位置
                    h, w, d = goal[i].shape
                    pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1],
                                      [w - 1, 0]]).reshape(-1, 1, 2)
                    dst = cv2.perspectiveTransform(pts, M)
                    coord = (dst[0][0] + dst[1][0] + dst[2][0] +
                             dst[3][0]) / 4.0
                    connection.send((DRAW_COORDS, coord))
                    # cv2.putText(img, "{}".format(coord), (50, 60),
                    #         fontFace=None, fontScale=1,
                    #         color=(0, 255, 0), lineType=1)
                    print(format(dst[0][0][0]))
                    x = (dst[0][0][0] + dst[1][0][0] + dst[2][0][0] +
                         dst[3][0][0]) / 4.0
                    y = (dst[0][0][1] + dst[1][0][1] + dst[2][0][1] +
                         dst[3][0][1]) / 4.0

                    # bound box  绘制边框
                    # img = cv2.polylines(img, [np.int32(dst)], True, 244, 3, cv2.LINE_AA)
                    connection.send((DRAW_RECT, dst))
                    # cv2.polylines(mixture, [np.int32(dst)], True, (0, 255, 0), 2, cv2.LINE_AA)
        except Exception as e:
            pass

        if x + y > 0:
            return x, y
        else:
            return None


# The path to save the image folder
def parse_folder(folder):
    restore = []
    path1 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    path = path1 + '/' + folder

    for i, j, k in os.walk(path):
        for l in k:
            restore.append(cv2.imread(path + '/{}'.format(l)))
    # print(restore)
    return restore


def compute_keypoints_and_descriptors(sift, images_lists):
    kp_list = []
    desc_list = []
    for images in images_lists:
        kp_tmp = []
        desc_tmp = []
        for img in images:
            kp, desc = sift.detectAndCompute(img, None)
            kp_tmp.append(kp)
            desc_tmp.append(desc)
        kp_list.append(kp_tmp)
        desc_list.append(desc_tmp)

    return kp_list, desc_list


GET_FRAME = 1
STOP_PROCESSING = 2
DRAW_COORDS = 3
DRAW_RECT = 4
CLEAR_DRAW = 5
CROP_FRAME = 6


def get_frame(connection):
    connection.send(GET_FRAME)
    frame = connection.recv()
    return frame


def process_transform_frame(frame, x1, y1, x2, y2):
    # enlarge the image by 1.5 times
    fx = 1.5
    fy = 1.5
    frame = cv2.resize(frame, (0, 0),
                       fx=fx,
                       fy=fy,
                       interpolation=cv2.INTER_CUBIC)
    if x1 != x2:
        # the cutting ratio here is adjusted according to the actual situation
        frame = frame[int(y2 * 0.7):int(y1 * 1.15),
                int(x1 * 0.7):int(x2 * 1.15)]
    return frame


def process_display_frame(connection):
    cap_num = 20
    coord = None
    dst = None
    x1 = 0
    y1 = 0
    x2 = 0
    y2 = 0
    cap = cv2.VideoCapture(cap_num, cv2.CAP_V4L)
    if not cap.isOpened():
        cap.open()
    while cv2.waitKey(1) < 0:
        _, frame = cap.read()
        frame = process_transform_frame(frame, x1, y1, x2, y2)
        if connection.poll():
            request = connection.recv()
            if request == GET_FRAME:
                connection.send(frame)
            elif request == CLEAR_DRAW:
                coord = None
                dst = None
            elif type(request) is tuple:
                if request[0] == DRAW_COORDS:
                    coord = request[1]
                elif request[0] == DRAW_RECT:
                    dst = request[1]
                elif request[0] == CROP_FRAME:
                    x1 = request[1]
                    y1 = request[2]
                    x2 = request[3]
                    y2 = request[4]

        if not coord is None:
            cv2.putText(frame, "{}".format(coord), (50, 60), fontFace=None,
                        fontScale=1, color=(0, 255, 0), lineType=1)
        if not dst is None:
            frame = cv2.polylines(frame, [np.int32(dst)], True, 244, 3, cv2.LINE_AA)
        cv2.imshow("figure", frame)
        time.sleep(0.04)
    connection.send(STOP_PROCESSING)


def img_single():
    parent_conn, child_conn = Pipe()
    child = Process(target=process_display_frame, args=(child_conn,))
    child.start()

    res_queue = [[], [], [], []]
    res_queue[0] = parse_folder('res/D')
    res_queue[1] = parse_folder('res/C')
    res_queue[2] = parse_folder('res/A')
    res_queue[3] = parse_folder('res/B')

    sift = cv2.xfeatures2d.SIFT_create()
    # sift = cv2.SIFT_create()
    kp_list, desc_list = compute_keypoints_and_descriptors(sift, res_queue)

    # init a class of Object_detect
    detect = Object_detect()

    # init mycobot
    detect.run()

    # control the number of crawls 控制抓取次数
    count = 0

    # _init_ = 20  #
    init_num = 0
    nparams = 0
    # num = 0
    # real_sx = real_sy = 0
    while True:
        start_time = time.time()
        if parent_conn.poll():
            data = parent_conn.recv()
            if data == STOP_PROCESSING:
                break
        # read camera
        frame = get_frame(parent_conn)

        # calculate the parameters of camera clipping
        if init_num < 20:
            if detect.get_calculate_params(frame) is None:
                # cv2.imshow("figure", frame)
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
            parent_conn.send((CROP_FRAME,
                              (detect.sum_x1) / 20.0,
                              (detect.sum_y1) / 20.0,
                              (detect.sum_x2) / 20.0,
                              (detect.sum_y2) / 20.0))
            detect.sum_x1 = detect.sum_x2 = detect.sum_y1 = detect.sum_y2 = 0
            init_num += 1
            continue

        # calculate params of the coords between cube and mycobot
        if nparams < 10:
            if detect.get_calculate_params(frame) is None:
                # cv2.imshow("figure", frame)
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
                print("ok")
                continue
        elif nparams == 10:
            nparams += 1
            # calculate and set params of calculating real coord between cube and mycobot
            detect.set_params((detect.sum_x1 + detect.sum_x2) / 20.0,
                              (detect.sum_y1 + detect.sum_y2) / 20.0,
                              abs(detect.sum_x1 - detect.sum_x2) / 10.0 +
                              abs(detect.sum_y1 - detect.sum_y2) / 10.0)
            print("ok")
            continue
        if count < 2:
            # get detect result
            kp_img, desc_img = sift.detectAndCompute(frame, None)
            frame = get_frame(parent_conn)
            for i, v in enumerate(res_queue):
                # HACK: to update frame every time
                detect_result = detect.obj_detect(frame, v, kp_img, desc_img, kp_list[i], desc_list[i], parent_conn)
                if detect_result:
                    x, y = detect_result
                    # calculate real coord between cube and mycobot
                    real_x, real_y = detect.get_position(x, y)
                    detect.color = i
                    # detect.pub_marker(real_x / 1000.0, real_y / 1000.0)
                    detect.decide_move(real_x, real_y, detect.color)
                    parent_conn.send(CLEAR_DRAW)
                    count += 1

            # cv2.imshow("figure", frame)
            time.sleep(0.05)
            end_time = time.time()
        else:
            break
        # close the window
        if cv2.waitKey(1) & 0xFF == ord('q'):
            # cap.release()
            cv2.destroyAllWindows()
            sys.exit()

    child.join()


def img_loop():
    parent_conn, child_conn = Pipe()
    child = Process(target=process_display_frame, args=(child_conn,))
    child.start()

    res_queue = [[], [], [], []]
    res_queue[0] = parse_folder('res/D')
    res_queue[1] = parse_folder('res/C')
    res_queue[2] = parse_folder('res/A')
    res_queue[3] = parse_folder('res/B')

    sift = cv2.xfeatures2d.SIFT_create()
    # sift = cv2.SIFT_create()
    kp_list, desc_list = compute_keypoints_and_descriptors(sift, res_queue)

    # init a class of Object_detect
    detect = Object_detect()

    # init mycobot
    detect.run()

    # _init_ = 20  #
    init_num = 0
    nparams = 0
    # num = 0
    # real_sx = real_sy = 0
    while True:
        start_time = time.time()
        if parent_conn.poll():
            data = parent_conn.recv()
            if data == STOP_PROCESSING:
                break
        # read camera
        frame = get_frame(parent_conn)
        # deal img
        # frame = detect.transform_frame(frame)

        # calculate the parameters of camera clipping
        if init_num < 20:
            if detect.get_calculate_params(frame) is None:
                # cv2.imshow("figure", frame)
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
            parent_conn.send((CROP_FRAME,
                              (detect.sum_x1) / 20.0,
                              (detect.sum_y1) / 20.0,
                              (detect.sum_x2) / 20.0,
                              (detect.sum_y2) / 20.0))
            detect.sum_x1 = detect.sum_x2 = detect.sum_y1 = detect.sum_y2 = 0
            init_num += 1
            continue

        # calculate params of the coords between cube and mycobot
        if nparams < 10:
            if detect.get_calculate_params(frame) is None:
                # cv2.imshow("figure", frame)
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
                print("ok")
                continue
        elif nparams == 10:
            nparams += 1
            # calculate and set params of calculating real coord between cube and mycobot
            detect.set_params((detect.sum_x1 + detect.sum_x2) / 20.0,
                              (detect.sum_y1 + detect.sum_y2) / 20.0,
                              abs(detect.sum_x1 - detect.sum_x2) / 10.0 +
                              abs(detect.sum_y1 - detect.sum_y2) / 10.0)
            print("ok")
            continue

        # get detect result
        kp_img, desc_img = sift.detectAndCompute(frame, None)
        frame = get_frame(parent_conn)
        for i, v in enumerate(res_queue):
            # HACK: to update frame every time
            detect_result = detect.obj_detect(frame, v, kp_img, desc_img, kp_list[i], desc_list[i], parent_conn)
            if detect_result:
                x, y = detect_result
                # calculate real coord between cube and mycobot
                real_x, real_y = detect.get_position(x, y)
                detect.color = i
                # detect.pub_marker(real_x / 1000.0, real_y / 1000.0)
                detect.decide_move(real_x, real_y, detect.color)
                parent_conn.send(CLEAR_DRAW)

        # cv2.imshow("figure", frame)
        time.sleep(0.05)
        end_time = time.time()
        # close the window
        if cv2.waitKey(1) & 0xFF == ord('q'):
            # cap.release()
            cv2.destroyAllWindows()
            sys.exit()

    child.join()


if __name__ == "__main__":
    # 提醒用户操作字典
    print("********************************************************")
    print("*  请输入数字选择模式(Please enter number selection mode)：*")
    print("*  1: 单次模式(single mode)                              *")
    print("*  2: 循环模式(loop mode)                                *")
    print("*  3: 退出(quit)                                        *")
    print("********************************************************")
    mode = int(input('请选择模式(please select mode):'))
    if mode == 1:
        img_single()
    elif mode == 2:
        img_loop()
    elif mode == 3:
        exit(0)
