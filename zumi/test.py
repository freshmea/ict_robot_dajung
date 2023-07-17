from zumi.zumi import Zumi 
from zumi.util.screen import Screen
import time 

zumi = Zumi()
screen = Screen()
def acc_text(string, run_time, line = 25):
    previous_time = time.time()
    state = 0
    while(time.time() < previous_time + run_time):
        angles = zumi.update_angles()
        x_angle = angles[3]
        if(x_angle > 5):
            if(state !=1):
                state = 1
                screen.loop_text('L', string,line)
        elif(x_angle < -5):
            if(state !=2):
                state = 2
                screen.loop_text('R', string,line)
    screen.loop_text('S')
    print("done")

acc_text('Slope',3)