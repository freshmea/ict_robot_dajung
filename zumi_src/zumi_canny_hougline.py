from zumi.zumi import Zumi
from zumi.util.screen import Screen
from zumi.util.camera import Camera
from zumi.util.vision import Vision
import IPython.display
from zumi.protocol import Note
from threading import Thread
from zumi.util.color_classifier import ColorClassifier
import time
import cv2
import numpy as np

zumi = Zumi()
screen = Screen()
camera = Camera()
vision = Vision()



# camera.start_camera()
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 128)
cap.set(cv2.CAP_PROP_FPS, 20)
if not cap:
    raise
try:
    cnt = 0
    start_t = time.time()
    # encode_param=[int(cv2.IMWRITE_JPEG_QUALITY),9]
    while True:
#         cnt += 1
        ret, frame = cap.read()
        frame = cv2.flip(frame, -1)
        terminate_t = time.time()
#         FPS = 'fps'+ str(int(cnt/(terminate_t-start_t)))
#         cv2.putText(frame,FPS, (30,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 1)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 100)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180.0, 100, maxLineGap=5)
        try:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(edges,(x1,y1), (x2,y2), (0,0,255), 2)
        except:
            pass
#         print(FPS)
        # result, frame=cv2.imencode('.jpg',frame,encode_param)
        # camera.show_image(cv2.cvtColor(cv2.imdecode(frame,1), cv2.COLOR_BGR2RGB))
        camera.show_image(edges)
        IPython.display.clear_output(wait=True)
finally:
    cap.release()
#     camera.close()