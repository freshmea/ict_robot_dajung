import time
from zumi.protocol import IR

class Line_Tracer:
    def __init__(self,zumi_obj):
        self.zumi = zumi_obj 
    def crossing_line(self,numLines , motor_speed = 10, ir_threshold = 125):
                        
        # 직진을 위한 자이로 센서 보정
        #zumi.mpu.calibrate_MPU(100)
        self.zumi.reset_gyro()
        heading = 0
        
        left_on_white = True
        right_on_white  = True
        right_switch = 0
        left_switch = 0
        
        time_switch = False
        lineCount = 1
                
        # 흰색에서 출발
        print("start")

        while True:
            
            bottom_left_ir = self.zumi.get_IR_data(IR.bottom_left)
            bottom_right_ir = self.zumi.get_IR_data(IR.bottom_right)

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
                if left_on_white == True:
                    time_switch = True
            
            elif time_switch == True:
                if left_on_white == False:
                    time_switch = False
                    print(lineCount)
                    lineCount = lineCount + 1
                
            if right_on_white and not left_on_white:
                heading -= 1

            if left_on_white and not right_on_white:
                heading += 1
                
            if right_switch == numLines or left_switch == numLines:
                break

            self.zumi.go_straight(motor_speed, heading)
                        
        self.zumi.stop()
        print("stop")
                     
    def counting_line(self,move_time , motor_speed = 10, ir_threshold = 125, line_length_S = 0.03, line_length_L = 0.18):
        
        self.zumi.reset_gyro()
        heading = 0
        
        left_on_white = True
        right_on_white  = True
        right_switch = 0
        left_switch = 0
        
        oldTime = 0
        time_switch = False
        
        line_threshold_S = 0.03
        line_threshold_L = 0.18
        
        count = 0;
        
        lineWidth = []
        
        # 흰색에서 출발
        print("start")
        
        start_time = time.time()
        
        
        while time.time()-start_time < move_time:   

            bottom_left_ir = self.zumi.get_IR_data(IR.bottom_left)
            bottom_right_ir = self.zumi.get_IR_data(IR.bottom_right)

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
                    #count+=1
                    #print(count)
            
            elif time_switch == True:
                if left_on_white == True:
                    time_switch = False
                    timeGap = time.time() - oldTime
                                    
                    if(timeGap > line_length_L):
                        count+=1
                        #print("\t"+"l"+"\t"+str(timeGap)[:5])   
                      #  lineWidth.append("L")
                        
                    elif(timeGap > line_length_S):
                        count+=1
                        #print("\t"+"s"+"\t"+str(timeGap)[:5])   
                      #  lineWidth.append("S")                    

                    #else:
                        #print("\t"+"n"+"\t"+str(timeGap)[:5])  
                        #lineWidth.append("N")
                        
                    #print(timeGap)   
                
            if right_on_white and not left_on_white:
                heading -= 1

            if left_on_white and not right_on_white:
                heading += 1
                
            #if right_switch == numLines or left_switch == numLines:
            #    break

            self.zumi.go_straight(motor_speed, heading)
                    
        self.zumi.stop()
        print(count)
       # print(lineWidth)
        
        data = "" 
            
        for i in range(len(lineWidth)):
            #screen.draw_text(lineWidth[i]+" ")
            data = data + str(lineWidth[i])+" "
            time.sleep(0.1)
            
        