import cv2 
import time 

cascade = cv2.CascadeClassifier('opencv/data/haarcascade_frontalface_alt.xml')
cap = cv2.VideoCapture('opencv/data/sample.mp4')

while True:
    start_t = time.time()
    ret, frame = cap.read()
    frame = cv2.resize(frame, dsize=None, fx=0.7, fy=0.7)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    results = cascade.detectMultiScale(
        gray, 
        scaleFactor=1.1, 
        minNeighbors = 5, 
        minSize = (20,20))
    for box in results:
        cv2.rectangle(frame, box, (255,0,0), 2)
    terminate_t = time.time()
    FPS = 'fps'+ str(int(1./(terminate_t-start_t)))
    cv2.putText(frame,FPS, (30,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 1)
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) > 0:
        break
    