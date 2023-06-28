import cv2
import numpy as np 

src = cv2.imread('opencv/data/Heart10.jpg', cv2.IMREAD_GRAYSCALE)
cv2.imshow('src', src)

ret, dst = cv2.threshold(src, 120, 255, cv2.THRESH_BINARY)
print('ret= ', ret)
cv2.imshow('dst', dst)

ret2, dst2 = cv2.threshold(src, 120, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
print('ret= ', ret2)
cv2.imshow('dst2', dst2)


cv2.waitKey()
cv2.destroyAllWindows()