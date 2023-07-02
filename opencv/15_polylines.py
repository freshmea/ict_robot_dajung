import cv2
import numpy as np 

img = np.zeros(shape=(512,512, 3), dtype=np.uint8) + 255

pts1 = np.array([[100, 140], [200, 130], [200, 260], [100,220]])
pts2 = np.array([[300, 200], [400, 100], [400, 200]])

cv2.polylines(img, [pts1], isClosed=True, color=(255,0,0), thickness=3)
cv2.polylines(img, [pts2], isClosed=True, color=(255,0,0), thickness=3)

cv2.imshow('img', img)
cv2.waitKey()
cv2.destroyAllWindows()