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


cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 128)
cap.set(cv2.CAP_PROP_FPS, 20)
if not cap:
    raise
try:
    while True:
        ret, frame = cap.read()
        frame = cv2.flip(frame, -1)
        # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        data = frame.reshape((-1 , 3)).astype(np.float32)
        K = 4
        term_crit=(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        ret, labels, centers = cv2.kmeans(data, K, None, term_crit, 5, cv2.KMEANS_RANDOM_CENTERS)
        centers = np.uint8(centers)
        res = centers[labels.flatten()]
        dst = res.reshape(frame.shape)
        camera.show_image(cv2.cvtColor(dst, cv2.COLOR_BGR2RGB))
        IPython.display.clear_output(wait=True)
finally:
    cap.release()