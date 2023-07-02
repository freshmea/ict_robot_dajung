import cv2 
import numpy as np

img = cv2.imread('opencv/data/lena.jpg')

# shape = img.shape[0], img.shape[1], 3
# dst = np.zeros(shape, np.uint8)
dst_r = np.zeros_like(img)
dst_g = np.zeros_like(img)
dst_b = np.zeros_like(img)

dst_r[:,:,2] = img[:,:,2]
dst_g[:,:,1] = img[:,:,1]
dst_b[:,:,0] = img[:,:,0]


cv2.imshow('img', img)
cv2.imshow('dst_r', dst_r)
cv2.imshow('dst_g', dst_g)
cv2.imshow('dst_b', dst_b)
cv2.waitKey()
cv2.destroyAllWindows()