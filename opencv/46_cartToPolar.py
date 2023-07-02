import cv2 
import numpy as np 

src = cv2.imread('opencv/data/rect.jpg', cv2.IMREAD_GRAYSCALE)

gx = cv2.Sobel(src, cv2.CV_32F, 1, 0,ksize= 3)
gy = cv2.Sobel(src, cv2.CV_32F, 0, 1,ksize= 3)

mag, angle = cv2.cartToPolar(gx, gy, angleInDegrees=True)
minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(angle)
print('angle =', minVal, maxVal, minLoc, maxLoc)

ret, edge = cv2.threshold(mag, 100, 255, cv2.THRESH_BINARY)
edge = edge.astype(np.uint8)
cv2.imshow('edge', edge)

# height, width = mag.shape[:2]
# angleM = np.full( (height, width, 3), (255,255,255), dtype=np.uint8)

height, width = mag.shape[:2]
angleM = np.zeros((height, width, 3), dtype=np.uint8) + 255

# angleM = cv2.merge([np.zeros_like(mag),np.zeros_like(mag),np.zeros_like(mag)]) +255

for y in range(height):
    for x in range(width):
        if edge[y, x] != 0:
            if angle[y, x] == 0:
                angleM[y, x] = (0,0,255) # red
            elif angle[y, x] == 90:
                angleM[y, x] = (0,255,0) # green
            elif angle[y, x] == 180:
                angleM[y, x] = (255,0,0) # blueq
            elif angle[y, x] == 270:
                angleM[y, x] = (0,255,255) # yellow
            else:
                angleM[y, x] = (128, 128, 128)   

cv2.imshow('angleM', angleM)
cv2.imshow('src', src)
cv2.waitKey()
cv2.destroyAllWindows()
