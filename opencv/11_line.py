import cv2
import numpy as np

img = np.zeros(shape=(512,512,3), dtype=np.uint8)# unsigned char in c/c++
pt1 = (100, 100)
pt2 = (400, 400)

cv2.rectangle(img, pt1, pt2, (0,255,0), 2)
# -내부에 overloading 된 함수가 있음.
cv2.rectangle(img, (100,100,300,300), (0,255,0), 2)

cv2.line(img, (0,0),(500,0), (255,0,0), 5)
cv2.line(img, (0,0),(0,500), (0,0,255), 5)

cv2.imshow('img', img)
cv2.waitKey()
cv2.destroyAllWindows()