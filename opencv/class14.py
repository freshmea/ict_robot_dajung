import cv2
import numpy as np

# a = [[] [] []   ]
#     [     ]
#     [     ]
# a[y][x]

img = np.zeros(shape=(1024,512,3), dtype=np.uint8) + 255 # np shape 에서 (y,x, deep) 순서
ptCenter = img.shape[1] // 2, img.shape[0] // 2
size = 200, 100

# overloading 1
cv2.ellipse(img, ptCenter, size, 0, 0, 180, (255,0,0))
cv2.ellipse(img, ptCenter, size, 45, 0, 360, (0,0,255))

# overloading 2
box = (ptCenter, size, 0)
cv2.ellipse(img, box, (255, 0, 0), 5)
box2 = (ptCenter, size, 45)
cv2.ellipse(img, box2, (255, 0, 0), 5)

cv2.imshow('img', img)
cv2.waitKey()
cv2.destroyAllWindows()