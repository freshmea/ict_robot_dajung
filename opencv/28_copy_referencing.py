import cv2 
import numpy as np

img = cv2.imread('opencv/data/lena.jpg')

# dst = img # 참조가 일어났다.
# dst = img.copy() # 데이터를 카피해서 새로운 객체가 만들어 졌다. 

roi = img[100:500, 250:400] # 참조가 일어난다. 
roi = img[100:500, 250:400].copy() # 카피가 일어난다. 
roi[100:150, 50:100] = [0, 0, 0]
# dst[100:500, 250:400] = [0, 0, 0]


cv2.imshow('img', img)
cv2.imshow('roi', roi)
# cv2.imshow('dst', dst)
cv2.waitKey()
cv2.destroyAllWindows()