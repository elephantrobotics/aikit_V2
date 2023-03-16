import time

from megaAiKit import megaAikit

# kit = megaAikit('COM24')

# print('one.....')
# kit.write_distance(2, 10)
# time.sleep(2)
# kit.write_distance(7, 10)
# print('one end...')


# print('two.....')
# kit.write_distance(7, 50)
# time.sleep(4)
# kit.write_distance(4, 50)
# print('two end..')
# time.sleep(4)
# kit.write_distance(7, 50)
# time.sleep(4)
# kit.write_distance(4, 50)

# while True:
#     kit.write_distance(2, 10)
#
#     time.sleep(0.1)
#     kit.write_distance(7, 10)
#     time.sleep(0.1)

import cv2
import numpy as np
import cv2
# 定义颜色范围
# lower_yellow = np.array([20, 100, 100])
# upper_yellow = np.array([30, 255, 255])
# lower_blue = np.array([110, 50, 50])
# upper_blue = np.array([130, 255, 255])

lower_blue = np.array([100, 80, 80])
upper_blue = np.array([130, 255, 255])

cap = cv2.VideoCapture(1)

while True:
    _, frame = cap.read()

    # 转换为HSV颜色空间
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 设定阈值并进行颜色分割
    yellow_mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # 边缘检测
    edges = cv2.Canny(yellow_mask, 100, 200)

    # 轮廓检测
    contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 过滤轮廓
    valid_contours = []
    for contour in contours:
        if cv2.contourArea(contour) > 1000:  # 过滤太小的轮廓
            valid_contours.append(contour)

    # 绘制轮廓
    cv2.drawContours(frame, valid_contours, -1, (0, 255, 0), 3)

    cv2.imshow("frame", frame)
    cv2.imshow("yellow_mask", yellow_mask)
    cv2.imshow("edges", edges)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()


