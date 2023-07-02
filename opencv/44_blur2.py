import cv2 
import numpy as np 

src = cv2.imread('opencv/data/lena.jpg', cv2.IMREAD_GRAYSCALE)

dst1 = cv2.medianBlur(src, 15)
dst2 = cv2.blur(src, (15,15))
dst3 = cv2.GaussianBlur(src,(7,7), 0.0)
dst4 = cv2.GaussianBlur(src,(31,31), 100.0)


cv2.imshow('dst1', dst1)
cv2.imshow('dst2', dst2)
cv2.imshow('dst3', dst3)
cv2.imshow('dst4', dst4)
cv2.waitKey()
cv2.destroyAllWindows()