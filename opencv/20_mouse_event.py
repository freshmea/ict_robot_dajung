import cv2
import numpy as np 


def onMouse(event, x, y, flags, param):
    if event == cv2.EVENT_MBUTTONDOWN:
        param[0] = np.zeros((512,515, 3), np.uint8) + 255
    elif event == cv2.EVENT_LBUTTONDOWN:
        if flags & cv2.EVENT_FLAG_CTRLKEY:
            cv2.rectangle(param[0], (x-5, y-5), (x+5, y+5), (0,0,255))
        else:
            cv2.circle(param[0], (x-5,y-5), 10, (255,0,0))
    elif event == cv2.EVENT_RBUTTONDOWN:
        cv2.circle(param[0], (x-5,y-5), 10, (0,255,0), 3)
    cv2.imshow('img', param[0])

img = np.zeros((512,515, 3), np.uint8) + 255

cv2.imshow('img', img)
cv2.setMouseCallback('img', onMouse, [img])
cv2.waitKey()
cv2.destroyAllWindows()
