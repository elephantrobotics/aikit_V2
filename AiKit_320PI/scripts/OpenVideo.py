import cv2
import platform

if platform.system() == "Windows":
    cap_num = 1
    cap = cv2.VideoCapture(cap_num, cv2.CAP_DSHOW)
    cap.set(3, 640)
    cap.set(4, 480)
elif platform.system() == "Linux":
    cap_num = 0
    cap = cv2.VideoCapture(cap_num, cv2.CAP_V4L)
    cap.set(3, 640)
    cap.set(4, 480)

while cv2.waitKey(1) < 0:
    ret, frame = cap.read()
    if not ret:
        break
    cv2.imshow('', frame)
