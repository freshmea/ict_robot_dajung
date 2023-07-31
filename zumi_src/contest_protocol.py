# import 구문 

from zumi.zumi import Zumi
from zumi.util.screen import Screen
from zumi.util.camera import Camera
from zumi.util.vision import Vision
from zumi.protocol import Note
from threading import Thread
from zumi.util.color_classifier import ColorClassifier
import time

class Zumi2(Zumi):
    def __init__(self):
        super().__init__()
        
        # drive PID values; used in forward(),go_straight() etc
        self.D_P = 2.9
        self.D_I = 0.01
        self.D_D = 0.05

        # turn PID values; used in turn()
        self.T_P = 0.6
        self.T_I = 0.001
        self.T_D = 0.001
        
    def line_follow_gyro_assist(self, speed=20, duration=1, angle=None, angle_adj=2, l_th=None, r_th=None):
        if l_th is None:
            l_th = self.IR_THRESHOLD_LIST[3]
        if r_th is None:
            r_th = self.IR_THRESHOLD_LIST[1]

        self.reset_PID()
        start_time = time.time()
        flag = 0
        # if no angle is inserted, drive in the direction currently facing
        if angle is None:
            self.update_angles()
            init_ang = self.angle_list[2]
        else:
            init_ang = angle
        try:
            while time.time() - start_time < duration:
                ir_readings = self.get_all_IR_data()
                left_bottom_ir = ir_readings[3]
                right_bottom_ir = ir_readings[1]
                # if both sensors are outside stop
                if left_bottom_ir < l_th and right_bottom_ir < r_th:
                    self.turn(init_ang)
                    if flag % 9 > 3:
                        init_ang -= 20
                    else:
                        init_ang += 20
                    flag += 1
                # if both sensor detect black keep going
                elif left_bottom_ir > l_th and right_bottom_ir > r_th:
                    self.go_straight(speed, init_ang)
                # if left sensor out turn to right
                elif left_bottom_ir < l_th and right_bottom_ir > r_th:
                    init_ang = init_ang - angle_adj
                    # while left_bottom_ir <100 and right_bottom_ir >100:
                    self.go_straight(speed, init_ang)
                # if right sensor out turn to left
                elif left_bottom_ir > l_th and right_bottom_ir < r_th:
                    init_ang = init_ang + angle_adj
                    # while left_bottom_ir >100 and right_bottom_ir <100:
                    self.go_straight(speed, init_ang)
                else:
                    pass
        finally:
            self.stop()

    def forward2(self, speed=40, duration=1.0,):
        start_time = time.time()
        right_speed = -.00375*(speed-136.66)**2+60.041
        try:
            while (time.time() - start_time) < abs(duration):
                self.control_motors(int(right_speed), speed)
                
        except KeyboardInterrupt:
            raise
        finally:
            self.stop()

zumi = Zumi2()
screen = Screen()
camera = Camera()
vision = Vision()
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
    zumi.funnel_align(speed=20, duration=6)
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
    for x in range(50):
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
            # if 0 < heading < 90:
            #     heading = 90
            # if 0 > heading < -90:
            #     heading = -90
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
    zumi.turn(-90)
    zumi.funnel_align(speed=20, duration=6)
    zumi.turn(-90)
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
    zumi.forward(speed = 20, duration = 0.2)
    zumi.line_follow_gyro_assist(speed=20, duration=40)
    
    
    
        
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
    zumi.line_follow_gyro_assist(speed=20, duration=10)
    zumi.forward(40, duration = 0.5)
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
    