import cv2
import numpy as np

img = np.zeros(shape=(800, 800, 3), dtype=np.uint8) + 255

x1, x2 = 100, 700
y1, y2 = 100, 700
cv2.rectangle(img, (x1, y1),(x2, y2), (0,0,255))

pt1 = (120, 50)
pt2 = (300, 750)

cv2.line(img, pt1, pt2, (100,100,0), 2)

imgRect = (x1, y1, x2-x1 , y2-y1)
retvel, rpt1, rpt2 = cv2.clipLine(imgRect, pt1, pt2)
if retvel:
    cv2.circle(img, rpt1, radius=50, color=(0,255,0), thickness=-1)
    cv2.circle(img, rpt2, radius=5, color=(0,255,0), thickness=-1)

cv2.imshow('img', img)
cv2.waitKey()
cv2.destroyAllWindows()
