import cv2


cap  = cv2.VideoCapture(1)

while cv2.waitKey(1)<0:
    ret, frame = cap.read()
    if not ret:
        break
    cv2.imshow('', frame)



