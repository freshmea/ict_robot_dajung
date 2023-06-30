import cv2 
import numpy as np 

src = cv2.imread('opencv/data/CornerTest.jpg')
gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)

K = 5
corners = cv2.goodFeaturesToTrack(gray, K, 0.05, 10)
corners2 = cv2.goodFeaturesToTrack(gray, K, 0.05, 10, useHarrisDetector=True)

corners = corners.reshape(-1, 2)
corners2 = corners2.reshape(-1, 2)
for x, y in corners:
    print(type(x))
    cv2.circle(src, (int(x),int(y)), 5, (0,0,255), -1)
for x, y in corners2:
    cv2.circle(src, (int(x),int(y)), 5, (255,0,0), 1)
    
cv2.imshow('src', src)
cv2.waitKey()
cv2.destroyAllWindows()