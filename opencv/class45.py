import cv2 
import numpy as np 

src = cv2.imread('opencv/data/rect.jpg', cv2.IMREAD_GRAYSCALE)

gx = cv2.Sobel(src, cv2.CV_32F, 1, 0,ksize= 3)
gy = cv2.Sobel(src, cv2.CV_32F, 0, 1,ksize= 3)

dstX = cv2.sqrt(np.abs(gx))
dstX = cv2.normalize(dstX, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

dstY = cv2.sqrt(np.abs(gy))
dstY = cv2.normalize(dstY, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

mag = cv2.magnitude(gx, gy)
minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(mag)
print('mag :', minVal, maxVal, minLoc, maxLoc)

dstM = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

cv2.imshow('src', src)
cv2.imshow('dstX', dstX)
cv2.imshow('dstY', dstY)
cv2.imshow('dstM', dstM)
cv2.waitKey()
cv2.destroyAllWindows()
