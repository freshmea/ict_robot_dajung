from zumi.zumi import Zumi 
import time 

zumi = Zumi()

zumi.reset_gyro()
try:
    for x in range(400):
        ir = zumi.get_all_IR_data()
        if ir[0] < 100:
            zumi.stop()
        else:
            zumi.forward_step(40,0)
finally:
    zumi.stop()
    print('Done!!')