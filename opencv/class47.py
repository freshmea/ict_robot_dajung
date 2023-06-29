import cv2 
import numpy as np 

src = cv2.imread('opencv/data/lena.jpg', cv2.IMREAD_GRAYSCALE)
blur = cv2.GaussianBlur(src, (7,7), 0.0)

lap = cv2.Laplacian(src, cv2.CV_32F)
dst = cv2.convertScaleAbs(lap)
dst = cv2.normalize(dst, None, 0, 255, cv2.NORM_MINMAX)

lap2 = cv2.Laplacian(blur, cv2.CV_32F)
dst2 = cv2.convertScaleAbs(lap2)
dst2 = cv2.normalize(dst2, None, 0, 255, cv2.NORM_MINMAX)

cv2.imshow('src', src)
cv2.imshow('src', src)
cv2.imshow('lap', lap)
cv2.imshow('dst', dst)
cv2.imshow('dst2', dst2)
cv2.waitKey()
cv2.destroyAllWindows()