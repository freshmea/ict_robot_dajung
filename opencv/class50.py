import cv2 
import numpy as np 

src = cv2.imread('opencv/data/lena.jpg', cv2.IMREAD_GRAYSCALE)

edges1 = cv2.Canny(src, 50, 100)
edges2 = cv2.Canny(src, 50, 200)

cv2.imshow('edges1', edges1)
cv2.imshow('edges2', edges2)
cv2.waitKey()
cv2.destroyAllWindows()