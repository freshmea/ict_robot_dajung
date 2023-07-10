import cv2 
import numpy as np

img = cv2.imread('opencv/data/lena.jpg')

dst = cv2.resize(img, (320, 240))
dst2 = cv2.resize(img, (0,0), fx=1.5 ,fy=1.2)
dst3 =cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
cv2.imshow('img', img)
cv2.imshow('dst', dst)
cv2.imshow('dst2', dst2)
cv2.imshow('dst3', dst3)
cv2.waitKey()
cv2.destroyAllWindows()