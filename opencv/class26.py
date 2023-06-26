import cv2 
import numpy as np

img = cv2.imread('opencv/data/lena.jpg') #, cv2.IMREAD_GRAYSCALE)  # Numpy.array -- 모든 속성과 메소드 가 사용 가능

# dst = np.zeros((512,512,3), dtype=np.uint8) + 255
dst = np.zeros_like(img) + 255
N = 32 # 한 줄의 격자의 갯수.
height, width, depth = img.shape
h = height // N # 격자의 세로
w = width // N # 격자의 가로 
for i in range(N):
    for j in range(N):
        x = i * h # 각 격자의 x 좌표 
        y = j * w # 각 격자의 y 좌표
        roi = img[y:y+h, x:x+w] # 하나의 격자를 ROI로 만듬
        dst[y:y+h, x:x+w] = cv2.mean(roi)[0:3] # roi 를 mean 처리

cv2.imshow('img', img)
cv2.imshow('dst', dst)
cv2.waitKey()
cv2.destroyAllWindows()