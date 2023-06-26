import cv2 
import numpy as np

img = cv2.imread('opencv/data/lena.jpg') #, cv2.IMREAD_GRAYSCALE)  # Numpy.array -- 모든 속성과 메소드 가 사용 가능

# dst = np.zeros((512,512,3), dtype=np.uint8) + 255
dst = np.zeros_like(img) + 255
N = 32
height, width, depth = img.shape
h = height // N
w = width // N
for i in range(N):
    for j in range(N):
        x = i * h
        y = j * w
        roi = img[y:y+h, x:x+w]
        dst[y:y+h, x:x+w] = cv2.mean(roi)[0:3]

cv2.imshow('img', img)
cv2.imshow('dst', dst)
cv2.waitKey()
cv2.destroyAllWindows()