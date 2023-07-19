from zumi.zumi import Zumi
from zumi.util.camera import Camera
from zumi.util.vision import Vision
from zumi.util.screen import Screen
from zumi.module.Barcode import reading_code
from module import Hangman
zumi = Zumi()
camera = Camera()
vision = Vision()
screen = Screen()

# zumi.reset_gyro()
# camera.start_camera()

# for i in range(100):
#     img = camera.capture()
#     stop_sign_detect = vision.detect_stop_sign(img)
#     if stop_sign_detect:
#         print('정지 표시를 발견했습니다.')
#         break
#     else:
#         zumi.forward_step(20, 0)
# zumi.stop()
# camera.close()

zumi.reset_gyro()
camera.start_camera()

stop_go = True
heading = 0
for i in range(30):
    img = camera.capture()
    stop_go = vision.detect_stop_sign(img)
    try:
        while stop_go:
            img = camera.capture()
            stop_go = vision.detect_stop_sign(img)
            print('정지 표시가 사라지길 기다리는 중')
        print('정지 표시 사라졌습니다.')
    finally:
        print('완료')
    zumi.turn(heading)
    heading += 3
