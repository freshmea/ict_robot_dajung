from zumi.zumi import Zumi
import time
from zumi.util.vision import Vision
from zumi.util.camera import Camera
from zumi.util.screen import Screen

zumi = Zumi()
camera = Camera()
vision = Vision()
screen = Screen()
zumi.turn
zumi.forward
camera.start_camera()

frame = camera.capture()
camera.close()
camera.show_image(frame)
screen.sleeping()