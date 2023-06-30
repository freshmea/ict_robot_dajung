import cv2 
import numpy as np 

src = cv2.imread('opencv/data/chessBoard.jpg')
gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)

#1
# goodF = cv2.GFTTDetector_create()

#2
goodF = cv2.GFTTDetector_create(40, 0.1, 10, useHarrisDetector=True)
kp = goodF.detect(gray)
print(len(kp))
points = cv2.KeyPoint_convert(kp)
print(points)

dst = cv2.drawKeypoints(src, kp, None, (255,0,0))

cv2.imshow('dst', dst)
cv2.waitKey()
cv2.destroyAllWindows()