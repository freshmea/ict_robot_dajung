from threading import Thread
import time
from zumi.zumi import Zumi
from zumi.util.screen import Screen

zumi = Zumi()
screen = Screen()

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

a= Thread(target = celebrate)
b= Thread(target = screen.happy)
c= Thread(target = happymove)
a.start()
b.start()
c.start()
print('job is start')
a.join()
b.join()
c.join()
print('job is done')