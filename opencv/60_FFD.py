import cv2 
import numpy as np 

src = cv2.imread('opencv/data/chessBoard.jpg')
gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
fastF = cv2.FastFeatureDetector.create(threshold=80)

kp = fastF.detect(gray)
dst = cv2.drawKeypoints(src, kp, None, (0,0,255))

points = cv2.KeyPoint_convert(kp)

print(type(points))
print(points)
print(len(kp))

cv2.imshow('src', dst)
cv2.waitKey()
cv2.destroyAllWindows()