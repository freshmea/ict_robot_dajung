import cv2 
import numpy as np

src1 = cv2.imread('opencv/data/lena.jpg')
src2 = cv2.imread('opencv/data/opencv_logo.png')
cv2.imshow('src2', src2)

rows, cols, channels = src2.shape
roi = src1[0:rows, 0:cols]

gray = cv2.cvtColor(src2, cv2.COLOR_BGR2GRAY)
ret, mask = cv2.threshold(gray, 160, 255, cv2.THRESH_BINARY)
mask_inv = cv2.bitwise_not(mask)
cv2.imshow('mask', mask)
cv2.imshow('mask_inv', mask_inv)


src1_bg = cv2.bitwise_and(roi, roi, mask=mask)
cv2.imshow('src1_gb', src1_bg)

src2_fg = cv2.bitwise_and(src2, src2, mask = mask_inv)
cv2.imshow('src2_fg', src2_fg)


dst = cv2.bitwise_or(src1_bg, src2_fg)
cv2.imshow('dst', dst)

src1[0:rows, 0:cols] = dst

cv2.imshow('result', src1)

cv2.waitKey()
cv2.destroyAllWindows()