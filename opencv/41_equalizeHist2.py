import cv2
import numpy as np 

src = cv2.imread('opencv/data/lena.jpg')
hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
h, s, v = cv2.split(hsv)
v = cv2.equalizeHist(v)
dst = cv2.merge([h,s,v])
dst_1 = cv2.cvtColor(dst, cv2.COLOR_HSV2BGR)

yCrCb = cv2.cvtColor(src, cv2.COLOR_BGR2YCrCb)
y, cr, cv = cv2.split(yCrCb)
y = cv2.equalizeHist(y)
dst2 = cv2.merge([y, cr, cv])
dst2_1 = cv2.cvtColor(dst2, cv2.COLOR_YCrCb2BGR)


cv2.imshow('src', src)
cv2.imshow('dst_1', dst_1)
cv2.imshow('dst2_1', dst2_1)

cv2.waitKey()
cv2.destroyAllWindows()
