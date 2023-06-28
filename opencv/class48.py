import cv2 
import numpy as np 

src = cv2.imread('opencv/data/rect.jpg', cv2.IMREAD_GRAYSCALE)

kx, ky = cv2.getDerivKernels(1, 0, 3)
sobelX = ky.dot(kx.T)
# [-1, 0, 1]
# [-2, 0, 2]
# [-1, 0, 1] np.float32

gx = cv2.filter2D(src, cv2.CV_32F, sobelX)
cv2.imshow('gx', gx)
cv2.waitKey()
cv2.destroyAllWindows()