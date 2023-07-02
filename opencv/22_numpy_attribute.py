import numpy as np
import cv2

img = cv2.imread('opencv/data/lena.jpg')#, cv2.IMREAD_GRAYSCALE)  # Numpy.array -- 모든 속성과 메소드 가 사용 가능

print('img.ndim= ', img.ndim)
print('img.shape= ', img.shape)
print('img.dtype= ', img.dtype)
print('img.size = ', img.size)

# Numpy 메소드 astype
img = img.astype(np.int32)
print('img.dtype= ', img.dtype)
# img = img.astype(np.uint8)
img = np.uint8(img) # 타입캐스팅.

cv2.imshow('img', img)
cv2.waitKey()
cv2.destroyAllWindows()
