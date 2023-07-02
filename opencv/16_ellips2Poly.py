import cv2
import numpy as np

img = np.zeros(shape=(512,512,3), dtype=np.uint8) + 255

ptCenter = (256,256)
size = 200, 100
# cv2.ellipse(img, ptCenter, size, 0, 0, 360, (255,0,0), thickness=2)
pts1 = cv2.ellipse2Poly(ptCenter, size, 0, 0, 360, delta = 20)

cv2.polylines(img, [pts1], isClosed=True, color=(0,0,255), thickness=2)
cv2.imshow('img', img)
cv2.waitKey()
cv2.destroyAllWindows()