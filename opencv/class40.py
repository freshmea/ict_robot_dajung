import cv2
import numpy as np 
from matplotlib import pyplot as plt 

src = cv2.imread('opencv/data/lena.jpg', cv2.IMREAD_GRAYSCALE)
hist1 = cv2.calcHist([src], [0], None, [256], [0,256])
plt.plot(hist1, color='b', label='hist1 in src')

dst = cv2.equalizeHist(src)
hist2 = cv2.calcHist([dst], [0], None, [256], [0,256])
plt.plot(hist2, color='r', alpha=0.7, label='hist2 in src')

cv2.imshow('src', src)
cv2.imshow('dst',dst)
cv2.waitKey()
cv2.destroyAllWindows()

plt.show()