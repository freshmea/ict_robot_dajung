import cv2 
import numpy as np 


src = cv2.imread('opencv/data/people.png')
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())


loc1, weights1 = hog.detectMultiScale(src)
print(len(loc1))
dst1 = src.copy()
w, h = hog.winSize
for pt in loc1:
    x, y, w, h= pt
    cv2.rectangle(dst1, (x,y), (x+w, y+h),(255,0,0), 2)


cv2.imshow('src', src)
cv2.imshow('dst1', dst1)
cv2.waitKey()
cv2.destroyAllWindows()

