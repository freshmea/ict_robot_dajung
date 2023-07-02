import cv2
import numpy as np

def onChange(pos):
    global img
    
    b = cv2.getTrackbarPos('B', 'img')
    g = cv2.getTrackbarPos('G', 'img')
    r = cv2.getTrackbarPos('R', 'img')
    
    img[:] = (b ,g ,r)
    # ir = np.zeros((512,515,1), np.uint8) + r
    # ig = np.zeros((512,515,1), np.uint8) + g
    # ib = np.zeros((512,515,1), np.uint8) + b
    # img = (cv2.merge([ib, ig, ir])) 
    cv2.imshow('img', img)

img = np.zeros((512,515,3), np.uint8) + 255

cv2.imshow('img', img)

cv2.createTrackbar('R', 'img', 0, 255, onChange)
cv2.createTrackbar('G', 'img', 0, 255, onChange)
cv2.createTrackbar('B', 'img', 0, 255, onChange)

cv2.waitKey()
cv2.destroyAllWindows()