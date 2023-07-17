from zumi.zumi import Zumi 
import time 

zumi = Zumi()

for i in range(20):
    ir_readings = zumi.get_all_IR_data()
    print(len(ir_readings))
    front_right_ir = ir_readings[0]
    front_left_ir = ir_readings[5]
    
    print('IR readings' + str(front_right_ir), str(front_left_ir))
    time.sleep(0.1)
print('완료')

for i in range(20):
    ir_readings = zumi.get_all_IR_data()
#     print(len(ir_readings))
    bottom_right_ir = ir_readings[1]
    bottom_left_ir = ir_readings[3]
    
    print('IR readings' + str(bottom_right_ir), str(bottom_left_ir))
    time.sleep(0.1)
print('완료')

for i in range(20):
    ir_readings = zumi.get_all_IR_data()
#     print(len(ir_readings))
    back_right_ir = ir_readings[2]
    back_left_ir = ir_readings[4]
    
    print('IR readings' + str(back_right_ir), str(back_left_ir))
    time.sleep(0.1)
print('완료')