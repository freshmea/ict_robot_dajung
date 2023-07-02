import cv2 
import numpy as np 

src = cv2.imread('opencv/data/fruits.jpg')
hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
h,s,v = cv2.split(hsv)

roi = cv2.selectROI('src', src)

roi_h = h[roi[1]:roi[1]+roi[3], roi[0]:roi[0]+roi[2]]
hist = cv2.calcHist([roi_h], [0], None, [64],[0,256])
backP = cv2.calcBackProject([h.astype(np.float32)], [0], hist, [0,256], scale = 1.0)
hist = cv2.sort(hist, cv2.SORT_EVERY_COLUMN + cv2.SORT_DESCENDING)
k = 1
T = hist[k][0]-1
ret, dst = cv2.threshold(backP, T, 255, cv2.THRESH_BINARY)

cv2.imshow('dst', dst)
cv2.waitKey()
cv2.destroyAllWindows()


