import platform

import cv2


cap_num = 20
cap = cv2.VideoCapture(cap_num, cv2.CAP_V4L)
cap.set(3, 640)
cap.set(4, 480)

while cv2.waitKey(1) < 0:
    ret, frame = cap.read()
    # 旋转180度
    frame = cv2.rotate(frame, cv2.ROTATE_180)
    if not ret:
        break
    cv2.imshow('', frame)
