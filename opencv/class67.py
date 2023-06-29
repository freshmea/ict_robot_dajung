import cv2 
import numpy as np 

def onMouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDBLCLK:
        param[0][:,:] = 0  # clear image
    param[1][0] = x  # mouse point z
    param[1][1] = y
    
frame = np.zeros((512,512,3), np.uint8)

z = np.zeros((2,1), np.float32)

cv2.namedWindow('Kalman Filter')
cv2.setMouseCallback('Kalman Filter', onMouse, [frame, z])

q = 1e-5
r = 0.01
KF = cv2.KalmanFilter(4,2,0)
KF.transitionMatrix = np.array([[1,0,1,0],         
                                [0,1,0,1],
                                [0,0,1,0],
                                [0,0,0,1]], np.float32)  # A
KF.measurementMatrix = np.array([[1,0,0,0],
                                 [0,1,0,0]],np.float32)  # H
KF.processNoiseCov       = q* np.eye(4, dtype=np.float32)   # Q 
KF.measurementNoiseCov = r* np.eye(2, dtype=np.float32)   # R

#4 initial value
KF.errorCovPost  = np.eye(4, dtype=np.float32)       # P0 = I
KF.statePost     = np.zeros((4, 1), dtype=np.float32) # x0 = 0

last_z = z.copy()
last_estimate = KF.statePost.copy()
#5
while True:
    predict  = KF.predict()
    estimate =KF.correct(z)
    
    x1, y1 = np.int0(last_z.flatten())
    x2, y2 = np.int0(z.flatten())
    cv2.line(frame, (x1, y1),(x2, y2), (0,0,255), 2 )
    
    x1, y1,_, _ = np.int0(last_estimate.flatten())
    x2, y2, _, _ = np.int0(estimate.flatten())
    cv2.line(frame, (x1, y1),(x2, y2), (255,0,0), 2 )
    cv2.imshow('Kalman Filter',frame)
 
    last_z = z.copy()
    last_estimate = estimate.copy()
    
    key = cv2.waitKey(30)
    if key == 27: break
cv2.destroyAllWindows()