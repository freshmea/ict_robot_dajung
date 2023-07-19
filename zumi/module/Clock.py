
import Adafruit_SSD1306
from PIL import Image, ImageFont, ImageDraw
import os
import math
import time
TEXT_FILE_PATH =  os.path.dirname(os.path.realpath(__file__)) + "/futura.ttf"

class Clock():    
    
    def __init__(self):
        
        self.disp = Adafruit_SSD1306.SSD1306_128_64(rst=24)
        self.disp.begin()
            
    def on_screen(self, hour, minute , string ='', font_size = 16):
        
        #TEXT_FILE_PATH = os.path.dirname(os.path.abspath('__file__')) + "/module/futura.ttf"
        
        #disp = Adafruit_SSD1306.SSD1306_128_64(rst=24)

        #disp.begin()

        # Clear display.
        #disp.clear()
        #disp.display()

        # Create blank image for drawing.
        # Make sure to create image with mode '1' for 1-bit color.
        width = self.disp.width
        height = self.disp.height


        # Clock
        image = Image.new('1', (width, height))
        draw = ImageDraw.Draw(image)
        draw.rectangle((0,0,width,height), outline=0, fill=0)


        # Draw an ellipse.
        draw.ellipse((1, 1 , 64, 64), outline=255, fill=0)
        draw.ellipse((6, 6 , 58, 58), outline=255, fill=0)
        draw.ellipse((31, 31 , 33, 33), outline=255, fill=0)

        Timer_x = 32 
        Timer_y = height/2 

        # minute    
        basicMinHand = 19
        ampMinHand = 0.1

        if(minute <= 15):
            minHand = basicMinHand +(minute * ampMinHand)        
        elif(minute <= 30):
            minHand = basicMinHand +((15-(minute-15)) * ampMinHand)
        elif(minute <= 45):
            minHand = basicMinHand +((minute-30) * ampMinHand)    
        elif(minute <= 60):
            minHand = basicMinHand +((15-(minute-45)) * ampMinHand)

        claMinute = minute * 6 + 270 
        if(claMinute > 360):
            claMinute = claMinute -360

        radMin = 3.14159 /180 * claMinute; 

        x = minHand * math.cos(radMin) + Timer_x; 
        y = minHand * math.sin(radMin) + Timer_y;
        draw.line((Timer_x, Timer_y, x , y), fill=255); 
        #draw.ellipse((x-1, y-1, x+1 , y+1), fill=255);     


        # hour  

        ampmHour = hour

        if(hour > 12):
            hour = hour - 12


        basicHourHand = 15
        ampHourHand = 0.1

        if(hour <= 3):
            hourHand = basicHourHand + (hour * ampHourHand)   
        elif(hour <= 6):
            hourHand = basicHourHand + ((3-(hour-3)) * ampHourHand)
        elif(hour <= 9):
            hourHand = basicHourHand + ((hour-6) * ampHourHand)    
        elif(hour <= 12):
            hourHand = basicHourHand + ((3-(hour-9)) * ampHourHand)

        calHour = (hour * 30 + 270) #+ (6*claMinute/60)
        if(calHour > 360):
            calHour = calHour -360        

        radMin = 3.14159 /180 * calHour; 

        x = hourHand * math.cos(radMin) + Timer_x; 
        y = hourHand * math.sin(radMin) + Timer_y;

        draw.line((Timer_x, Timer_y, x , y), fill=255); 

        # Load default font.
        #font = ImageFont.load_default()
        font = ImageFont.truetype(TEXT_FILE_PATH, 15)

        if(ampmHour >= 10):
            if(minute >= 10):
                draw.text((73, 18),str(ampmHour) + ' : ' + str(minute),  font=font, fill=255)
            else:
                draw.text((73, 18),str(ampmHour) + ' : 0' + str(minute),  font=font, fill=255)
        else:
            if(minute >= 10):
                draw.text((73, 18),'  ' + str(ampmHour) + ' : ' + str(minute),  font=font, fill=255)
            else:
                draw.text((73, 18),'  ' + str(ampmHour) + ' : 0' + str(minute),  font=font, fill=255)

        length = len(string)
        if(length > 8):
            print('Please enter 8 characters or fewer than 8 characters.')
        else:
            font = ImageFont.truetype(TEXT_FILE_PATH, 12)
            draw.text((69, 18 + 25),string, font=font, fill=255)

        # Display image.
        self.disp.image(image)
        self.disp.display()
    
    