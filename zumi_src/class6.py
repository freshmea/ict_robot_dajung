from zumi.zumi import Zumi
zumi = Zumi()

zumi.reset_gyro()
try:
    for x in range(300):
        ir=zumi.get_all_IR_data()
        br_ir = ir[1]
        bl_ir = ir[3]
        if br_ir > 100 and bl_ir > 100:
            print('검은색 감지됨')
            zumi.stop()
            break
        else:
            zumi.forward_step(20, 0)
finally:
    zumi.stop()
zumi.line_follow_gyro_assist()
print('Done!!')