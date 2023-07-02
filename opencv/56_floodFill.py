import cv2 
import numpy as np 

src = np.zeros((512,512,3), np.uint8)
cv2.rectangle(src, (50, 100), (450,400), (255,255,255), -1)
cv2.rectangle(src, (100, 150), (400,350), (0,0,0), -1)
cv2.rectangle(src, (200, 200), (300,300), (255,255,255), -1)

dst = src.copy()
cv2.floodFill(dst, None, (150,170), (0,0,255))
cv2.floodFill(dst, None, (25,75), (255,0,0))

cv2.imshow('src', src)
cv2.imshow('dst', dst)
cv2.waitKey()
cv2.destroyAllWindows()
