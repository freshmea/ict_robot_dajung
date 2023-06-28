import cv2 
import numpy as np 

src = cv2.imread('opencv/data/rect.jpg', cv2.IMREAD_GRAYSCALE)

gx = cv2.Sobel(src, cv2.CV_32F, 1, 0,ksize= 3)
gy = cv2.Sobel(src, cv2.CV_32F, 0, 1,ksize= 3)

mag, angle = cv2.cartToPolar(gx, gy, angleInDegrees=True)
minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(angle)
print('angle =', minVal, maxVal, minLoc, maxLoc)

ret, edge = cv2.threshold(mag, 100, 255, cv2.THRESH_BINARY)
edge = edge.astype(np.uint8)
cv2.imshow('edge', edge)

cv2.imshow('src', src)
cv2.waitKey()
cv2.destroyAllWindows()
