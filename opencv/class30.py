import cv2
import numpy as np

img = cv2.imread('opencv/data/lena.jpg')

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
yCrCb = cv2.cvtColor(img, cv2.COLOR_BGR2YCrCb)
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
print(type(gray), type(yCrCb), type(hsv))
print(gray.dtype, yCrCb.dtype, hsv.dtype)
print(hsv.shape)

cv2.imshow('img', img)
cv2.imshow('gray', gray)
cv2.imshow('yCrCb', yCrCb)
cv2.imshow('hsv', hsv)
cv2.waitKey()
cv2.destroyAllWindows()