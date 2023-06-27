import cv2 
import numpy as np 

src = cv2.imread('opencv/data/lena.jpg', cv2.IMREAD_GRAYSCALE)

dst1 = cv2.boxFilter(src, ddepth= -1, ksize=(10, 10))
dst2 = cv2.boxFilter(src, ddepth= -1, ksize=(20, 20))

dst3 = cv2.bilateralFilter(src, d=-1, sigmaColor=200, sigmaSpace=10)
dst4 = cv2.bilateralFilter(src, d=-1, sigmaColor=100, sigmaSpace=10)


cv2.imshow('dst1', dst1)
cv2.imshow('dst2', dst2)
cv2.imshow('dst3', dst3)
cv2.imshow('dst4', dst4)
cv2.waitKey()
cv2.destroyAllWindows()