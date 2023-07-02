import cv2 
import numpy as np

img = cv2.imread('opencv/data/lena.jpg')
# roi = cv2.selectROI(img)

# if roi != (0,0,0,0):
#     dst = img[roi[1]:roi[1]+roi[3], roi[0]:roi[0]+roi[2]]
#     cv2.imshow('dst', dst)
#     cv2.waitKey()

rois = cv2.selectROIs('img', img, False, True)
for roi in rois:
    cv2.rectangle(img, (roi[0], roi[1], roi[2], roi[3]), (0, 0, 255), 2)
cv2.imshow('img', img)
cv2.waitKey()
cv2.destroyAllWindows()