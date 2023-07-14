import time
from PIL import Image, ImageFont, ImageDraw

class Gyro_Draw:    
            
    # GyroDraw 클래스 초기화합니다.
    def __init__(self,zumi_obj,screen_obj):        
        
        # OELD 제어를 위한 설정
        self.disp = screen_obj.disp
        self.disp.begin()
        self.image = Image.new('1', (self.disp.width, self.disp.height))
        self.draw = ImageDraw.Draw(self.image)
        
        # 포인터의 크기
        self.size = 3       
        
        # 자이로 X,Y 각도 값
        self.oldCharX = 255
        self.oldCharY = 255
        
        # 펜의 좌표 X,Y 값
        self.penX = 0
        self.penY = 0
        
        # 자이로 센서
        self.zumi = zumi_obj
        self.zumi.reset_gyro()
        
       
    # OLED 화면에 작은 사각형태의 점을 표시합니다.(포인터)
    def draw_xy(self, posX = 0, posY = 0):
        
        # 사각형으로 포인터를 그리며, 위치를 구분하기 위해 현재 위치는 원으로 표시합니다.
        self.draw.rectangle((self.oldCharX-1, self.oldCharY, self.oldCharX+self.size, self.oldCharY+self.size), outline=1, fill=1) 
        self.draw.ellipse((posX, posY, posX+self.size+2, posY+self.size+2), outline= 1, fill= 0) 
        
        self.disp.image(self.image)
        self.disp.display()
        
        self.oldCharX = posX
        self.oldCharY = posY
                
    # OLED 화면에 십자선을 그립니다.
    def draw_crossline(self):
        self.draw.line((0, 32, 128 , 32), fill=255);         
        self.draw.line((64, 0, 64 , 64), fill=255); 
        
        self.disp.image(self.image)
        self.disp.display()
        
    
    # 센서에서 읽은 데이터를 계산하여 자이로 각도 X,Y 값을 반환합니다. 
    def get_angle(self):
        
        updateAngles = self.zumi.update_angles()   
        
        angles=[0,0]
        angles[0] = -1 * int(updateAngles[3])
        angles[1] = -1 * int(updateAngles[4])
                        
        return angles
    
    # 자이로 X,Y 각도 값의 이전 상태와 현재 상태를 비교하여 펜의 좌표를 반환합니다.
    def pen_xy(self):
        
        # 자이로 각도 값 가져오기
        angle = self.get_angle()

        penSpeed = 3
        
        # X ,Y축의 자이로 기울기에 따른 펜의 좌표 값 
        if(angle[0] > 10) :
            self.penX = self.penX + penSpeed        
        elif(angle[0] < -10):
            self.penX = self.penX - penSpeed   
            
        if(angle[1] > 10) :
            self.penY = self.penY - penSpeed        
        elif(angle[1] < -10):
            self.penY = self.penY + penSpeed
            
        # 펜이 화면 밖으로 나가지 않도록 좌표 값 크기 제어
        if(self.penX > 59): 
            self.penX = 59
        elif(self.penX < -59): 
            self.penX = -59     
            
        if(self.penY > 27): 
            self.penY = 27
        elif(self.penY < -27): 
            self.penY = -27
        
        
        angle[0] = self.penX
        angle[1] = self.penY
        
        return angle
    
    
    # 입력된 값을 원하는 범위의 스케일로 변환 
    def scale_change(self,x, in_min, in_max, out_min, out_max) :
        return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)  
    
    # 화면 필셀 제거
    def clear_screen(self) :
        self.draw.rectangle((0,0,self.disp.width,self.disp.height), outline=0, fill=0)
        self.disp.image(self.image)
        self.disp.display()
        
        
        
        
        