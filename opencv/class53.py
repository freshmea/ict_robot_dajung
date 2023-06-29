import cv2 
import numpy as np 

src = cv2.imread('opencv/data/circles.jpg')
gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT,dp=1, minDist=50, param2=15)
circles = np.int32(circles)

print(type(circles))
print('circles.shape', circles.shape)
for circle in circles[0,:]:
    cx, cy, r = circle
    cv2.circle(src, (cx,cy), r, (0,0,255), 2)

cv2.imshow('src', src)
cv2.waitKey()
cv2.destroyAllWindows()
