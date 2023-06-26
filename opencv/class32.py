import cv2 
import numpy as np

img = cv2.imread('opencv/data/lena.jpg')

rows, cols, channels = img.shape
M1 = cv2.getRotationMatrix2D((rows // 2, cols // 2), 80, 1.0)
print(type(M1), M1.shape)
print(M1)
dst1 = cv2.warpAffine(img, M1, (rows, cols))

cv2.imshow('dst1', dst1)
cv2.waitKey()
cv2.destroyAllWindows()