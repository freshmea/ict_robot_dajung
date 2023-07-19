from zumi.zumi import Zumi 
import time 

zumi = Zumi()

zumi.reset_gyro()
heading = 0
try:
    for x in range(1000):
        ir = zumi.get_all_IR_data()
        front_right_ir = ir[0]
        front_left_ir = ir[5]
        if front_right_ir < 100:
            zumi.stop()
            heading += 30
            zumi.turn(heading, 0.5)
        if front_left_ir < 100:
            zumi.stop()
            heading -= 30
            zumi.turn(heading, 0.5)
        if front_right_ir < 100 and front_left_ir < 100:
            zumi.stop()
            zumi.reverse()
            heading -= 180
            zumi.turn(heading)
        else:
            zumi.forward_step(20, heading)
finally:
    zumi.stop()
print('Done!!')