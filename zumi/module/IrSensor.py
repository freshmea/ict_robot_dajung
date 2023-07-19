from zumi.zumi import Zumi
from zumi.protocol import IR

'''
front_right = 0
bottom_right = 1
back_right = 2
bottom_left = 3
back_left = 4
front_left = 5
'''
zumi = Zumi()
    
def boolean_ir(sensor ='', direction = '', distance = 100):
    
    readIR = -1
    detectSecsor = False
    
    if(sensor == 'front'):        
        if(direction == 'L'):
            readIR = zumi.get_IR_data(IR.front_left)
        elif(direction == 'R'):
            readIR = zumi.get_IR_data(IR.front_right)

    elif(sensor == 'back'):
        if(direction == 'L'):
            readIR = zumi.get_IR_data(IR.back_left)
        elif(direction == 'R'):
            readIR = zumi.get_IR_data(IR.back_right)

    elif(sensor == 'bottom'):
        if(direction == 'L'):
            readIR = zumi.get_IR_data(IR.bottom_left)
        elif(direction == 'R'):
            readIR = zumi.get_IR_data(IR.bottom_right)
    
    if(readIR < distance):
        detectSecsor = True
    
    return detectSecsor


def reading_ir(sensor ='', direction = ''):
    
    readIR = -1

    if(sensor == 'front'):        
        if(direction == 'L'):
            readIR = zumi.get_IR_data(IR.front_left)
        elif(direction == 'R'):
            readIR = zumi.get_IR_data(IR.front_right)

    elif(sensor == 'back'):
        if(direction == 'L'):
            readIR = zumi.get_IR_data(IR.back_left)
        elif(direction == 'R'):
            readIR = zumi.get_IR_data(IR.back_right)

    elif(sensor == 'bottom'):
        if(direction == 'L'):
            readIR = zumi.get_IR_data(IR.bottom_left)
        elif(direction == 'R'):
            readIR = zumi.get_IR_data(IR.bottom_right)

    return readIR
    