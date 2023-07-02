import cv2
import numpy as np

img = cv2.imread('opencv/data/lena.jpg', cv2.IMREAD_GRAYSCALE)  # Numpy.array -- 모든 속성과 메소드 가 사용 가능

print('img.shape= ', img.shape)
img = img.flatten()
print('img.shape= ', img.shape)
img = img.reshape(512, 512) #(512, 512)
# 윈도우 (-1, 512, 512)
print('img.shape= ', img.shape)

cv2.imshow('img', img)
cv2.waitKey()
cv2.destroyAllWindows()