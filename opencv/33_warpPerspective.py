import cv2 
import numpy as np 
from matplotlib import pyplot as plt

img = cv2.imread('opencv/data/book.jpg')

# cv2.circle(img, (55,30), 5, (0,0,255), 3)
# cv2.circle(img, (10,530), 5, (0,0,255), 3)
# cv2.circle(img, (420,15), 5, (0,0,255), 3)
# cv2.circle(img, (470,520), 5, (0,0,255), 3)
pts1 = np.float32([[55,30],[10,530],[420,15],[470,520]])
pts2 = np.float32([[10,10],[10,510],[410,10],[410,510]])

M = cv2.getPerspectiveTransform(pts1, pts2)

dst = cv2.warpPerspective(img, M, (500,500))
cv2.imshow('img', img)
cv2.imshow('dst', dst)
cv2.waitKey()
cv2.destroyAllWindows()