'''
Aruco标记的检测和定位
'''

import numpy as np
import time
import cv2
import math
import os 

# 摄像头的内参数矩阵
load = np.load(os.path.join(os.path.dirname(__file__), "mtx_dist.npz"))
mtx = load['mtx']
dist = load['dist']

# mtx = np.array(([
#     [781.33379113, 0., 347.53500524],
#     [0., 783.79074192, 246.67627253],
#     [0., 0., 1.]]))

# # 摄像头的畸变系数
# dist = np.array([[3.41360787e-01, -2.52114260e+00, -1.28012469e-03,  6.70503562e-03,
#         2.57018000e+00]])


# 调用摄像头
# cap = cv2.VideoCapture(2, cv2.CAP_V4L)
cap = cv2.VideoCapture(1)
font = cv2.FONT_HERSHEY_SIMPLEX

while True:
    # 获取视频帧
    ret, frame = cap.read()
    # 获取视频帧的宽高
    h1, w1 = frame.shape[:2]
    # 纠正畸变
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (h1, w1), 0, (h1, w1))

    dst1 = cv2.undistort(frame, mtx, dist, None, newcameramtx)
    x, y, w1, h1 = roi
    dst1 = dst1[y:y + h1, x:x + w1]
    frame = dst1

    # 灰度化
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 设置预定义的字典
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

    # 使用默认值初始化检测器参数
    parameters = cv2.aruco.DetectorParameters_create()
    dst1 = cv2.undistort(frame, mtx, dist, None, newcameramtx)


    # 使用aruco.detectMarkers()函数可以检测到marker，返回ID和标志版的4个角点坐标
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    # 如果找到ids
    if ids is not None:

        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist)
        (rvec - tvec).any()  # 避免numpy混乱数据
        # 获取旋转坐标值和平移坐标值
        xyz = tvec[0, 0, :]
        print(xyz)
        tsvec = tvec
        for i in range(3):
            tsvec[0][0][i] = round(tvec[0][0][i], 3)
        tsvec = np.squeeze(tsvec)
        for i in range(rvec.shape[0]):
            # 绘制轴
            cv2.aruco.drawAxis(frame, mtx, dist, rvec[i, :, :, ], tvec[i, :, :, ], 0.03)
            # 在标记周围画一个正方形
            cv2.aruco.drawDetectedMarkers(frame, corners)
        # 显示于画面中
        cv2.putText(frame, 'id=' + str(ids), (0, 64), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(frame,str(tsvec) , (0,90),font, 1, (0, 255, 0), 2, cv2.LINE_AA)
    else:

        cv2.putText(frame, 'not found,no ids', (0, 64), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.imshow('frame', frame)
    key = cv2.waitKey(1)
    if key == 27:
        print("ese break")
        cap.release()
        cv2.destroyAllWindows()