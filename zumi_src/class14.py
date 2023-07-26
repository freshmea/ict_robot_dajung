
from zumi.zumi import Zumi
from zumi.util.camera import Camera
from zumi.util.vision import Vision
from zumi.util.screen import Screen
from zumi.module.Barcode import reading_code
from module import Hangman
import cv2

zumi = Zumi()
camera = Camera()
vision = Vision()
screen = Screen()
camera.start_camera()
camera.capture
vision.find_QR_code()
zumi.drive_over_markers(time_out=5)
zumi.turn(desired_angle= 0, duration= 3, max_speed=10)
zumi.forward()
zumi.reverse()
zumi.forward_avoid_collision(speed=40, duration=10, desired_angle= 0, left_th=100, right_th= 100 )
zumi.funnel_align(speed=20, duration=5)
zumi.calibrate_gyro()
zumi.line_follow_gyro_assist(speed=20, duration=10, angle_adj=45)
screen.clear_drawing()
screen.draw_text()
screen.print()
cap = cv2.VideoCapture(0)
cv2.caph
