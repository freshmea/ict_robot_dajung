import cv2 
import numpy as np 

src = np.zeros((512,512,3), np.uint8)
cv2.rectangle(src, (50, 100), (450,400), (255,255,255), -1)
cv2.rectangle(src, (100, 150), (400,350), (0,0,0), -1)
cv2.rectangle(src, (200, 200), (300,300), (255,255,255), -1)
gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)

mode = cv2.RETR_LIST
method = cv2.CHAIN_APPROX_SIMPLE
contours, hierarchy = cv2.findContours(gray, mode, method)

print(type(contours))
print(type(contours[0]))
print(len(contours))
print(contours[0].shape)
print(contours[0])
cv2.drawContours(src, contours, -1, (255,0,0), 3)
cv2.imshow('src', src)
cv2.waitKey()
cv2.destroyAllWindows()