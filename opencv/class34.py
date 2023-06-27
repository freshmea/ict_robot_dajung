import cv2 
import numpy as np 

src1 = cv2.imread('opencv/data/lena.jpg', cv2.IMREAD_GRAYSCALE)
src2 = np.zeros_like(src1) + 100

dst1 = src1 + src2
dst2 = cv2.add(src1, src2)
cv2.imshow('src1', src1)
cv2.imshow('dst1', dst1)
cv2.imshow('dst2', dst2)
cv2.waitKey()
cv2.destroyAllWindows()
