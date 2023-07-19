from zumi.zumi import Zumi
from zumi.util.camera import Camera
from zumi.util.vision import Vision
from zumi.util.screen import Screen
import time
import cv2

class Vision_detect(Vision):
    def __init__(self):
        super().__init__()
        
    def detect_stop_sign_distance(self, frame, bounding_box=True, scale_factor=1.05, min_neighbors=8, min_size=(40, 40),
                       max_size=(320, 320)):
        signs = self.stop_sign_cascade.detectMultiScale(frame, scaleFactor=scale_factor, minNeighbors=min_neighbors, minSize=min_size, maxSize=max_size, flags=cv2.CASCADE_SCALE_IMAGE)
        if len(signs) > 0:
            for (x, y, w, h) in signs:
                # draw a bounding box on the stop sign found
                # x is x positon in frame, y is
                # w is width of area enclosing stop sign and h is height
                color_of_box = (255, 0, 0)  # this is in (R,G,B) 8 bit 2^8=256
                if bounding_box:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), color_of_box, 2)
                return [x, y, w, h]
        else:
            return None

zumi = Zumi()
camera = Camera()
vision = Vision()
screen = Screen()
vision2 = Vision_detect()


camera.start_camera()
img = camera.capture()
print(vision.detect_stop_sign(img))
print(vision2.detect_stop_sign_distance(img))
camera.show_image(img)
camera.close()
