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

zumi = Zumi()
screen = Screen()
camera = Camera()
vision = Vision()



# camera.start_camera()
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 128)
cap.set(cv2.CAP_PROP_FPS, 120)
if not cap:
    raise
try:
    cnt = 0
    start_t = time.time()
    while True:
        cnt += 1
        ret, frame = cap.read()
        frame = cv2.flip(frame, -1)
        terminate_t = time.time()
        FPS = 'fps'+ str(int(cnt/(terminate_t-start_t)))
        cv2.putText(frame,FPS, (30,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 1)
        print(FPS)
#         camera.show_image(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
#         IPython.display.clear_output(wait=True)
finally:
    cap.release()
#     camera.close()