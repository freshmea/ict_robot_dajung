import cv2
import numpy as np

img = np.zeros(shape=(512,1024,3), dtype=np.uint8) + 255

text = 'OpenCV Programming by Python'
org = (50, 100)
font =cv2.FONT_HERSHEY_SIMPLEX
cv2.putText(img, text, org, font, 1, (255,0,0), 2)
size, baseline = cv2.getTextSize(text, font, 1, 2)
cv2.rectangle(img, org, (org[0]+size[0], org[1]-size[1]), (0,0,255))
cv2.circle(img, org, 3, (0,255,0), 2)

cv2.imshow('img', img)
cv2.waitKey()
cv2.destroyAllWindows()