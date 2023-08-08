# import 구문 

from zumi.zumi import Zumi
from zumi.util.screen import Screen
from zumi.util.camera import Camera
from zumi.util.vision import Vision
from zumi.protocol import Note
from threading import Thread
from zumi.util.color_classifier import ColorClassifier
import time

zumi = Zumi()
screen = Screen()
camera = Camera()
vision = Vision()
zumi.calibrate_gyro()
zumi.reset_drive() # 서쪽 방향 0 도

def celebrate():
    for _ in range(3):
        time.sleep(.25)
        zumi.play_note(28, 100)
        time.sleep(.005)
        zumi.play_note(52, 100)
        time.sleep(.005)
        zumi.play_note(55, 100)
        time.sleep(.005)
        zumi.play_note(55, 100)
        time.sleep(.02)
        zumi.play_note(52, 100)
        time.sleep(.005)
        zumi.play_note(55, 100)
        zumi.play_note(0, 0)

def happymove():
    time.sleep(0.2)
    init_angle = zumi.read_z_angle()
    for _ in range(5):
        zumi.turn(init_angle+20, 0.5)
        zumi.turn(init_angle-20, 0.5)

def doremi():
    zumi.play_note(Note.C3, 1000)
    zumi.play_note(Note.D3, 1000)
    zumi.play_note(Note.E3, 1000)

try:
    # A-1
    print('A-1 start')
    camera.start_camera()
    qr = False
    while not qr:
        frame = camera.capture()
        qr = vision.find_QR_code(frame)
    doremi()
    message = vision.get_QR_message(qr)
    # A-2
    print('A-2 start')
    if message == 'factory':
        zumi.drive_over_markers(4, time_out=5)
    if message == 'school':
        zumi.drive_over_markers(9, time_out=5)
    if message == 'office':
        zumi.drive_over_markers(12, time_out=5)
    if message == 'museum':
        zumi.drive_over_markers(16, time_out=5)
    # A-3
    print('A-3 start')
    if message == 'factory' or message == 'office':
        zumi.turn(desired_angle= -90, duration= 2, max_speed=10)
    if message == 'school' or message == 'museum':
        zumi.turn(desired_angle= 90, duration= 2, max_speed=10)
    heading = zumi.read_z_angle()
    zumi.forward_avoid_collision(speed=40, duration=10, desired_angle= heading, left_th= 20, right_th= 20 )
    if message == 'factory' or message == 'office':
        zumi.turn(desired_angle= 90, duration= 2, max_speed=10)
    if message == 'school' or message == 'museum':
        zumi.turn(desired_angle= -90, duration= 2, max_speed=10)
    heading = zumi.read_z_angle()
    zumi.forward_avoid_collision(speed=40, duration=10, desired_angle= heading, left_th= 20, right_th= 20 )
    zumi.turn(desired_angle= 0, duration= 2, max_speed=10)
    zumi.forward_avoid_collision(speed=40, duration=10, desired_angle= 0, left_th=50, right_th= 50 )
    # A-4
    print('A-4 start')
    zumi.turn(-90)
    zumi.reset_drive() # 북쪽 방향 0 도
    zumi.funnel_align(speed=20, duration=10)
    zumi.turn(0)
    # B-1
    print('B-1 start')
    while True:
        frame = camera.capture()
        if vision.detect_stop_sign(frame):
            doremi()
            break
    while True:
        frame = camera.capture()
        if not vision.detect_stop_sign(frame):
            break
    # B-2
    print('B-2 start')
    heading = zumi.read_z_angle()
    b2_time = time.time()
    for x in range(300):
        ir = zumi.get_all_IR_data()
        front_right_ir = ir[0]
        front_left_ir = ir[5]
        if front_right_ir < 30 or front_left_ir < 30:
            if time.time() - b2_time < 10 :
                heading -= 30
            else:
                heading += 30
            zumi.turn(heading, 0.5)
        else:
            zumi.forward_step(20, heading)
    # B-3
    print('B-3 start')
    zumi.turn(0)
    zumi.reset_drive() # 북쪽 방향 0도 
    zumi.funnel_align(speed=20, duration=6)
    zumi.turn(0)
    # C-1
    print('C-1 start')
    user_name = 'aa'
    demo_name = 'red_yellow_green2'
    knn = ColorClassifier(user_name = user_name)
    train = knn.load_model(demo_name)
    knn.fit('hsv')
    while True:
        frame = camera.capture()
        predict = knn.predict(frame)
        if predict == 'red':
            doremi()
            break
    while True:
        frame = camera.capture()
        predict = knn.predict(frame)
        if predict == 'green':
            break
    # C-2
    print('C-2 start')
    zumi.forward(10, 0.3)
    zumi.line_follow_gyro_assist(speed=20, duration=2)
    zumi.turn(90)
    zumi.line_follow_gyro_assist(speed=20, duration=7)
    zumi.turn(0)
    zumi.line_follow_gyro_assist(speed=20, duration=3)
    zumi.turn(-90)
    zumi.line_follow_gyro_assist(speed=20, duration=2)
    # C-3
    print('C-3 start')
    qr = False
    while not qr:
        frame = camera.capture()
        qr = vision.find_QR_code(frame)
    doremi()
    message2 = vision.get_QR_message(qr)
    if message2 == 'right':
        zumi.turn(180)
    if message2 == 'left':
        zumi.turn(0)
    zumi.line_follow_gyro_assist(speed=20, duration=3)
    zumi.turn(-90)
    zumi.line_follow_gyro_assist(speed=20, duration=5)
    zumi.forward(40, duration = 1.0)
    a = Thread(target = celebrate)
    b = Thread(target = screen.happy)
    c = Thread(target = happymove)
    a.start()
    b.start()
    c.start()
    a.join()
    b.join()
    c.join()
    screen.clear_drawing()
    screen.print(message,x= 0, y = 20, font_size= 20)
    screen.print(message2,x= 0, y = 40, font_size= 20)


except KeyboardInterrupt:
    zumi.stop()
    zumi.play_note(0,0)
    camera.close()
    print('인터럽터 버튼을 눌렀으므로 정지합니다.')
finally:
    zumi.play_note(0,0)
    zumi.stop()
    camera.close()
    print('코드 종료')