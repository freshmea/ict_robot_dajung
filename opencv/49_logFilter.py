import cv2 
import numpy as np 

src = cv2.imread('opencv/data/lena.jpg', cv2.IMREAD_GRAYSCALE)

def logFilter(ksize = 7):
    k2 = ksize // 2
    sigma = 0.3*(k2-1)+0.8
    LoG = np.zeros((ksize,ksize), dtype=np.float32)
    for y in range(-k2, k2+1):
        for x in range(-k2, k2+1):
            g = -(x*x + y*y)/(2.0*sigma**2.0)
            LoG[y+k2, x+k2] = -(1.0+g)*np.exp(g)/(np.pi*sigma**4.0)
    return LoG

Kernel = logFilter() #7, 15, 31... 
LoG_image = cv2.filter2D(src, cv2.CV_32F, Kernel)

cv2.imshow('log', LoG_image)

cv2.waitKey()
cv2.destroyAllWindows()

