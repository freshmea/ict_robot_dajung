import cv2 
import numpy as np 

src = cv2.imread('opencv/data/chessBoard.jpg')
gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)

mserF = cv2.MSER_create(10)
kp = mserF.detect(gray)

print(len(kp))
points = cv2.KeyPoint_convert(kp)
print(points)

dst = cv2.drawKeypoints(src, kp, None, (255,0,0))

cv2.imshow('dst', dst)
cv2.waitKey()
cv2.destroyAllWindows()