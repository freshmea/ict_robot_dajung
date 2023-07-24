# import 구문 

from zumi.zumi import Zumi
from zumi.util.screen import Screen
from zumi.personality import Sound
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
sound = Sound(zumi)
zumi.calibrate_gyro()
zumi.reset_gyro()


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
    for _ in range(3):
        zumi.turn(init_angle+20, 0.5)
        zumi.turn(init_angle-20, 0.5)
        
def doremi():
    zumi.play_note(Note.C3, 1000)
    zumi.play_note(Note.D3, 1000)
    zumi.play_note(Note.E3, 1000)
    
# A - 1

try:
    camera.start_camera()
    qr = False
    while not qr:
        frame = camera.capture()
        qr = vision.find_QR_code(frame)
    doremi()
    message = vision.get_QR_message(qr)
    print(message)
except KeyboardInterrupt:
    zumi.stop()
    camera.close()
    print('인터럽터 버튼을 눌렀으므로 정지합니다.')
finally:
    zumi.stop()
    camera.close()
    print('코드 종료')
    
# A-2
try:
    if message == 'factory':
        # zumi.drive_over_markers(4, time_out=5)
        zumi.drive_over_markers(4, time_out=5)
    if message == 'school':
        zumi.drive_over_markers(8, time_out=5)
    if message == 'office':
        zumi.drive_over_markers(12, time_out=5)
    if message == 'museum':
        zumi.drive_over_markers(16, time_out=5)
except KeyboardInterrupt:
    zumi.stop()
    camera.close()
    print('인터럽터 버튼을 눌렀으므로 정지합니다.')
finally:
    zumi.stop()
    camera.close()
    print('코드 종료')
    
# A-3 

try:
    if message == 'factory' or message == 'office':
        zumi.turn(desired_angle= -90, duration= 3, max_speed=10)
    if message == 'school' or message == 'museum':
        zumi.turn(desired_angle= 90, duration= 3, max_speed=10)
    zumi.forward(30, 1.4)
    if message == 'factory' or message == 'office':
        zumi.turn(desired_angle= 90, duration= 3, max_speed=10)
    if message == 'school' or message == 'museum':
        zumi.turn(desired_angle= -90, duration= 3, max_speed=10)
    zumi.forward(30, 1.4)
    zumi.turn(desired_angle= 0, duration= 3, max_speed=10)
    zumi.forward_avoid_collision(speed=40, duration=10, desired_angle= 0, left_th=50, right_th= 50 )
except KeyboardInterrupt:
    zumi.stop()
    camera.close()
    print('인터럽터 버튼을 눌렀으므로 정지합니다.')
finally:
    zumi.stop()
    camera.close()
    print('코드 종료')
    
# A-4

try:
    zumi.turn(-90)
    zumi.funnel_align(speed=20, duration=18)
    zumi.turn(-90)
except KeyboardInterrupt:
    zumi.stop()
    camera.close()
    print('인터럽터 버튼을 눌렀으므로 정지합니다.')
finally:
    zumi.stop()
    camera.close()
    print('코드 종료')
    
# B-1, B-2
try:
    # B-1
    camera.start_camera()
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
    heading = zumi.read_z_angle()
    for x in range(350):
        ir = zumi.get_all_IR_data()
        front_right_ir = ir[0]
        front_left_ir = ir[5]
        front_left_ir -= 90 # ir data calibration
        if front_right_ir < 100 or front_left_ir < 100:
            zumi.stop()
            if front_right_ir < front_left_ir:
                heading += 30
            else:
                heading -= 30
            zumi.turn(heading, 0.5)
        else:
            zumi.forward_step(20, heading)
    
    
        
except KeyboardInterrupt:
    zumi.stop()
    camera.close()
    print('인터럽터 버튼을 눌렀으므로 정지합니다.')
finally:
    zumi.stop()
    camera.close()
    print('코드 종료')

# B-3
try:
    zumi.funnel_align(speed=20, duration=6)
except KeyboardInterrupt:
    zumi.stop()
    camera.close()
    print('인터럽터 버튼을 눌렀으므로 정지합니다.')
finally:
    zumi.stop()
    camera.close()
    print('코드 종료')
    
# C-1, C-2
try:
    # C-1 
    camera.start_camera()
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
    print('start')
    zumi.line_follow_gyro_assist(speed=20, duration=2)
    zumi.turn(0)
    zumi.line_follow_gyro_assist(speed=20, duration=7)
    zumi.turn(-90)
    zumi.line_follow_gyro_assist(speed=20, duration=3)
    zumi.turn(180)
    zumi.line_follow_gyro_assist(speed=20, duration=2)
    
    
    
        
except KeyboardInterrupt:
    zumi.stop()
    camera.close()
    print('인터럽터 버튼을 눌렀으므로 정지합니다.')
finally:
    zumi.stop()
    camera.close()
    print('코드 종료')
    
# C-3

try:
    camera.start_camera()
    qr = False
    while not qr:
        frame = camera.capture()
        qr = vision.find_QR_code(frame)
    doremi()
    message2 = vision.get_QR_message(qr)
    if message2 == 'right':
        zumi.turn(90)
    if message2 == 'left':
        zumi.turn(-90)
    zumi.line_follow_gyro_assist(speed=20, duration=3)
    zumi.turn(180)
    zumi.line_follow_gyro_assist(speed=20, duration=5)
    zumi.forward(40, duration = 1)
    a= Thread(target = celebrate)
    b= Thread(target = screen.happy)
    c= Thread(target = happymove)
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
    camera.close()
    print('인터럽터 버튼을 눌렀으므로 정지합니다.')
finally:
    zumi.stop()
    camera.close()
    print('코드 종료')
    