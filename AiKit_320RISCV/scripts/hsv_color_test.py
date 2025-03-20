import cv2
import platform

import numpy as np

cap = cv2.VideoCapture(20, cv2.CAP_V4L)

for i in range(0, 19):
    print(cap.get(i))

HSV = {
    "yellow": [np.array([11, 85, 70]), np.array([59, 255, 245])],
    "red": [np.array([0, 43, 46]), np.array([8, 255, 255])],
    "green": [np.array([35, 43, 35]), np.array([90, 255, 255])],
    "blue": [np.array([78, 43, 46]), np.array([110, 255, 255])],
    "cyan": [np.array([78, 43, 46]), np.array([99, 255, 255])],
}

while 1:

    ret, frame = cap.read()

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red = np.array([0, 43, 46])

    upper_red = np.array([8, 255, 255])

    mask = cv2.inRange(hsv, lower_red, upper_red)  # 红色掩模

    res = cv2.bitwise_and(frame, frame, mask=mask)

    cv2.imshow(u"Capture", frame)

    cv2.imshow(u"mask", mask)

    cv2.imshow(u"res", res)

    key = cv2.waitKey(1)

    if key & 0xff == ord('q') or key == 27:
        print(frame.shape, ret)

        break

cap.release()

cv2.destroyAllWindows()