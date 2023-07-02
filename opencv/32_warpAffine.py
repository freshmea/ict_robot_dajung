import cv2 
import numpy as np

img = cv2.imread('opencv/data/lena.jpg')

rows, cols, channels = img.shape
# M1 = cv2.getRotationMatrix2D((rows // 2, cols // 2), 80, 1.0)
M1 = cv2.getAffineTransform(np.array([[0,0],[100,100],[0,100]], dtype=np.float32), np.array([[100, 100],[50,50],[0,100]], dtype=np.float32))

M1[0][2] += 200
M1[1][2] += 200
print(type(M1), M1.shape)
print(M1)
dst1 = cv2.warpAffine(img, M1, (rows, cols))

cv2.imshow('dst1', dst1)
cv2.waitKey()
cv2.destroyAllWindows()