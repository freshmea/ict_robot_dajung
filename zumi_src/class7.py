from zumi.zumi import Zumi
import time
from zumi.util.vision import Vision
from zumi.util.camera import Camera

zumi = Zumi()
camera = Camera()
vision = Vision()

zumi.turn
zumi.forward
camera.start_camera()

frame = camera.capture()
camera.close()
camera.show_image(frame)