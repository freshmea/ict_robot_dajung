import cv2, sys
import numpy as np 

cap = cv2.VideoCapture('opencv/data/vtest.avi')
if (not cap.isOpened()):
    sys.exit()
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))

acc_gray = np.zeros(shape=(height, width), dtype=np.float32)
acc_bgr = np.zeros(shape=(height, width, 3), dtype=np.float32)


t = int()
while True:
    ret, frame = cap.read()
    if not ret:
        break
    t += 1
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    cv2.accumulate(frame, acc_bgr)
    avg_bgr = acc_bgr/t
    dst_gray = cv2.convertScaleAbs(avg_bgr)
    
    
    cv2.imshow('frame', frame)
    cv2.imshow('dst_gray', dst_gray)
    key = cv2.waitKey(20)
    if key == 27:
        break

cv2.destroyAllWindows()
