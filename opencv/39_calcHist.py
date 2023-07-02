import cv2 
import numpy as np 
from matplotlib import pyplot as plt

src = np.array([[0,0,0,0],[1,1,3,5],[6,1,1,3],[4,3,1,7]], dtype=np.uint8)

hist1 = cv2.calcHist([src],[0], None, [4], [0,8])
print('hist1 = ', hist1)

src1 = cv2.imread('opencv/data/lena.jpg', cv2.IMREAD_GRAYSCALE)

hist1_1 = cv2.calcHist([src1], [0], None, [64], [0,256])    
hist1_1 = hist1_1.flatten()
plt.title('hist1_1:binX = np.arange(32)')
binX = np.arange(64)
plt.plot(hist1_1, color='r')
plt.bar(binX, hist1_1, width=1, color='b')

plt.show()