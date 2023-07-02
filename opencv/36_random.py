import cv2
import numpy as np
import time 

# dst = np.full((512,512,3), (255,255,255), dtype=np.uint8)
dst = np.zeros((512,512,3), dtype=np.uint8) + 255

nPoints = 100
pts =np.zeros((1, nPoints, 2), dtype=np.uint16)

cv2.setRNGSeed(int(time.time()))
cv2.randu(pts, (0,0), (512,512))

for n in range(nPoints):
    x, y = pts[0,n][:] # [0,n,:]
    cv2.circle(dst, (x, y), 5, (0,0,255), -1)

cv2.imshow('dst', dst)
cv2.waitKey()
cv2.destroyAllWindows()
