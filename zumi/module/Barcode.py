from zumi.zumi import Zumi
from zumi.protocol import IR
import time

zumi = Zumi()

barcodeList = {}

def reading_code(): #바코드 읽는 함수

    motor_speed = 10
    ir_threshold = 125
    line_length_S = 0.03
    line_length_L = 0.18

    zumi.reset_gyro()
    heading = 0

    left_on_white = True
    right_on_white  = True
    right_switch = 0
    left_switch = 0

    time_switch = False

    oldTime = 0
    count = 0;
    lineWidth = []

    numLines = 4

    while True: 

        bottom_left_ir = zumi.get_IR_data(IR.bottom_left)
        bottom_right_ir = zumi.get_IR_data(IR.bottom_right)

        # white check  
        if bottom_left_ir < ir_threshold:
            if not left_on_white:
                left_switch += 1
            left_on_white = True
        else:
            left_on_white = False

        if bottom_right_ir < ir_threshold:
            if not right_on_white:
                right_switch += 1
            right_on_white = True
        else:
            right_on_white = False

        if time_switch == False:
            if left_on_white == False:
                time_switch = True
                oldTime = time.time()

        elif time_switch == True:
            if left_on_white == True:
                time_switch = False
                timeGap = time.time() - oldTime

                if(timeGap > line_length_L):
                    count+=1
                    #print("\t"+"l"+"\t"+str(timeGap)[:5])   
                    lineWidth.append("-")
                    
                elif(timeGap > line_length_S):
                    count+=1
                    #print("\t"+"s"+"\t"+str(timeGap)[:5])   
                    lineWidth.append(".")                    

                #else:
                    #print("\t"+"n"+"\t"+str(timeGap)[:5])  
                    #lineWidth.append("N")

                #print(timeGap)   

        if right_on_white and not left_on_white:
            heading -= 1

        if left_on_white and not right_on_white:
            heading += 1

        if right_switch == numLines or left_switch == numLines:
            break

        zumi.go_straight(motor_speed, heading)

    zumi.stop()
    
    return lineWidth


##바코드 등록하는 함수
def regist_code():
    readingCode = ""
    Name = input("product Name : ")

    if Name in barcodeList.keys():
        print("registered Name")
        return

    input("reading Ready, input any key")
    
    #lineWidth = reading_code()
    
    
    motor_speed = 10
    ir_threshold = 125
    line_length_S = 0.03
    line_length_L = 0.18

    zumi.reset_gyro()
    heading = 0

    left_on_white = True
    right_on_white  = True
    right_switch = 0
    left_switch = 0

    time_switch = False

    oldTime = 0
    count = 0;
    lineWidth = []

    numLines = 4

    while True: 

        bottom_left_ir = zumi.get_IR_data(IR.bottom_left)
        bottom_right_ir = zumi.get_IR_data(IR.bottom_right)

        # white check  
        if bottom_left_ir < ir_threshold:
            if not left_on_white:
                left_switch += 1
            left_on_white = True
        else:
            left_on_white = False

        if bottom_right_ir < ir_threshold:
            if not right_on_white:
                right_switch += 1
            right_on_white = True
        else:
            right_on_white = False

        if time_switch == False:
            if left_on_white == False:
                time_switch = True
                oldTime = time.time()

        elif time_switch == True:
            if left_on_white == True:
                time_switch = False
                timeGap = time.time() - oldTime

                if(timeGap > line_length_L):
                    count+=1
                    #print("\t"+"l"+"\t"+str(timeGap)[:5])   
                    lineWidth.append("-")
                    
                elif(timeGap > line_length_S):
                    count+=1
                    #print("\t"+"s"+"\t"+str(timeGap)[:5])   
                    lineWidth.append(".")                    

                #else:
                    #print("\t"+"n"+"\t"+str(timeGap)[:5])  
                    #lineWidth.append("N")

                #print(timeGap)   

        if right_on_white and not left_on_white:
            heading -= 1

        if left_on_white and not right_on_white:
            heading += 1

        if right_switch == numLines or left_switch == numLines:
            break

        zumi.go_straight(motor_speed, heading)

    zumi.stop()

    for name, code in barcodeList.items():
        if readingCode == code:
            print("registered code -", name, code)
            return

    barcodeList[Name] = lineWidth
    print(barcodeList)
    print("finish")

    
# 바코드 사용
def using_code():
    barcode = ''

    input("input any key")
        
    #lineWidth = reading_code()
    
    #print(barcodeList)
    motor_speed = 10
    ir_threshold = 125
    line_length_S = 0.03
    line_length_L = 0.18

    zumi.reset_gyro()
    heading = 0

    left_on_white = True
    right_on_white  = True
    right_switch = 0
    left_switch = 0

    time_switch = False

    oldTime = 0
    count = 0;
    lineWidth = []

    numLines = 4

    while True: 

        bottom_left_ir = zumi.get_IR_data(IR.bottom_left)
        bottom_right_ir = zumi.get_IR_data(IR.bottom_right)

        # white check  
        if bottom_left_ir < ir_threshold:
            if not left_on_white:
                left_switch += 1
            left_on_white = True
        else:
            left_on_white = False

        if bottom_right_ir < ir_threshold:
            if not right_on_white:
                right_switch += 1
            right_on_white = True
        else:
            right_on_white = False

        if time_switch == False:
            if left_on_white == False:
                time_switch = True
                oldTime = time.time()

        elif time_switch == True:
            if left_on_white == True:
                time_switch = False
                timeGap = time.time() - oldTime

                if(timeGap > line_length_L):
                    count+=1
                    #print("\t"+"l"+"\t"+str(timeGap)[:5])   
                    lineWidth.append("-")
                    
                elif(timeGap > line_length_S):
                    count+=1
                    #print("\t"+"s"+"\t"+str(timeGap)[:5])   
                    lineWidth.append(".")                    

                #else:
                    #print("\t"+"n"+"\t"+str(timeGap)[:5])  
                    #lineWidth.append("N")

                #print(timeGap)   

        if right_on_white and not left_on_white:
            heading -= 1

        if left_on_white and not right_on_white:
            heading += 1

        if right_switch == numLines or left_switch == numLines:
            break

        zumi.go_straight(motor_speed, heading)

    zumi.stop()
    
    
    print(lineWidth)
    for key, value in barcodeList.items():
        if lineWidth == value:
            print(key)
            return

    print('No data')
        

#바코드 지우기 함수
def clear_code():
    barcodeList.clear()