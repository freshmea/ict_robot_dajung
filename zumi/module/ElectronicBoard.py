from zumi.zumi import Zumi
from zumi.util.screen import Screen
import time
#import sys
import os
TEXT_FILE_PATH =  os.path.dirname(os.path.realpath(__file__)) + "/futura.ttf"

def moving_text(direction, string, line = 25, speed = 5): 
    import Adafruit_SSD1306
    import time
    #import os
    from PIL import Image, ImageFont, ImageDraw
    
    font_size = 16   
    
    #TEXT_FILE_PATH = os.path.dirname(os.path.abspath('__file__')) + "/module/futura.ttf"
    #TEXT_FILE_PATH =  os.path.dirname(os.path.realpath('__file__')) + "/futura.ttf"
    disp = Adafruit_SSD1306.SSD1306_128_64(rst=24)
    
    disp.begin()
    width = disp.width
    height = disp.height

    image = Image.new('1', (width, height))
    font = ImageFont.truetype(TEXT_FILE_PATH, font_size)    
    draw = ImageDraw.Draw(image)
    
    size = draw.textsize (string , font = font)
    pos = 128 - size[0]    
    length = len(string)
    
    if(line >= 0 and line <= 45):     
           
        if(length > 8):
            print('Please enter 8 characters or fewer than 8 characters.')
        else:      
            if(direction == 'R'):  
                for i in range(0, pos, speed):
                    image = Image.new('1', (width, height))
                    draw = ImageDraw.Draw(image)
                    disp.display()                
                    draw.text((0 + i, line), string, font=font, fill=255)                
                    disp.image(image)
                    disp.display()      

            if(direction == 'L'):  
                for i in range(pos, 0, -speed):

                    image = Image.new('1', (width, height))
                    draw = ImageDraw.Draw(image)
                    disp.display()                
                    draw.text((0 + i, line), string, font=font, fill=255)                
                    disp.image(image)
                    disp.display()  
    else:
        print('Lines can be entered from 0 to 45.')
                

def flicker_text(count, speed , string):
    #zumi = Zumi()
    screen = Screen()
    if(len(string) > 8):
        print('Please enter 8 characters or fewer than 8 characters.')
    else:        
        for i in range(0, count):        
            message = string
            screen.clear_display()  
            screen.draw_text_center(message)
            time.sleep(0.1 * speed) 

        
def size_text(count, size, string):
    screen = Screen()
    if(len(string) > 8):
        print('Please enter 8 characters or fewer than 8 characters.')
    else:     
        message = string
        screen.draw_text_center(message,font_size=16)    
        
        for i in range(0, count): 
            
            for j in range(16,size,2):            
                screen.draw_text_center(message,font_size=j)
            
            for j in range(size, 16, -2):    
                screen.draw_text_center(message,font_size=j)


def loop_text(direction, string ='', line = 25, font_size = 16):
    import Adafruit_SSD1306
    import time
    #import os
    from PIL import Image, ImageFont, ImageDraw
    
    
    #print(os.path.dirname(os.path.abspath('__file__')))
    #print(os.getcwd())
    #print(os.path.dirname(os.path.realpath(__file__)) )    
    #print(os.path.dirname(os.path.realpath(__file__)) + "/futura.ttf")
    #print(sys.path)
       
    #TEXT_FILE_PATH = os.path.dirname(os.path.abspath('__file__')) + "/module/futura.ttf"
    disp = Adafruit_SSD1306.SSD1306_128_64(rst=24)
    
    length = len(string)
    #print(length)
    disp.begin()
    width = disp.width
    height = disp.height

    image = Image.new('1', (width, height))
    font = ImageFont.truetype(TEXT_FILE_PATH, font_size)
    draw = ImageDraw.Draw(image)
    
    size = draw.textsize (string , font = font) 
    
    if(line >= 0 and line <= 45):     
           
        if(size[0] > 128):
            print("The string entered is too long.")

        else:

            if(direction == 'R'):

                disp.command(0x2E)#SSD1306_DEACTIVATE_SCROLL

                draw.text((1, line),string, font=font, fill=255)
                disp.image(image)
                disp.display()

                disp.command(0x26)#SSD1306_RIGHT_HORIZONTAL_SCROLL
                disp.command(0x00)#dummy    
                disp.command(0x00)#start
                disp.command(0x00)
                disp.command(0x0f)#stop
                disp.command(0x00)
                disp.command(0xff)
                disp.command(0x2f)#activate scroll

            elif(direction == 'L'):

                disp.command(0x2E)#SSD1306_DEACTIVATE_SCROLL

                draw.text((1, line),string, font=font, fill=255)
                disp.image(image)
                disp.display()

                disp.command(0x27)#SSD1306_LEFT_HORIZONTAL_SCROLL
                disp.command(0x00)#dummy    
                disp.command(0x00)#start
                disp.command(0x00)
                disp.command(0x0f)#stop
                disp.command(0x00)
                disp.command(0xff)
                disp.command(0x2f)#activate scroll

            elif(direction == 'S'):  
                disp.command(0x2E)#SSD1306_DEACTIVATE_SCROLL
    else:
        print('Lines can be entered from 0 to 45.')
        
def acc_text(string ='', line = 25, count = 100, font_size = 16):
        
    import Adafruit_SSD1306
    import time
    #import os
    from PIL import Image, ImageFont, ImageDraw
    
    length = len(string)
        
    zumi = Zumi()
    zumi.reset_gyro()

    state = 0

    #TEXT_FILE_PATH = os.path.dirname(os.path.abspath('__file__')) + "/module/futura.ttf"
    disp = Adafruit_SSD1306.SSD1306_128_64(rst=24)

    #print(length)
    disp.begin()
    width = disp.width
    height = disp.height
    screen_image = None

    image = Image.new('1', (width, height))
    font = ImageFont.truetype(TEXT_FILE_PATH, font_size)
    draw = ImageDraw.Draw(image)

    disp.clear()
    disp.display()
    
    size = draw.textsize (string , font = font) 
    
    if(line >= 0 and line <= 45):     
           
        if(size[0] > 128):
            print("The string entered is too long.")

        else:    

            draw.text((50, line),string, font=font, fill=255)
            disp.image(image)
            disp.display()

            disp.command(0x2E)#SSD1306_DEACTIVATE_SCROLL

            for i in range(0, count):

                angles = zumi.update_angles()   
                time.sleep(0.1)

                if(angles[3] > 5):   

                    if(state != 1):
                        state = 1
                        #loop_text('L', string)

                        disp.command(0x27)#SSD1306_LEFT_HORIZONTAL_SCROLL
                        disp.command(0x00)#dummy    
                        disp.command(0x00)#start
                        disp.command(0x00)
                        disp.command(0x0f)#stop
                        disp.command(0x00)
                        disp.command(0xff)
                        disp.command(0x2f)#activate scroll

                elif(angles[3] < -5): 

                     if(state != 2):
                            state = 2
                            #loop_text('R', string)  

                            disp.command(0x26)#SSD1306_RIGHT_HORIZONTAL_SCROLL
                            disp.command(0x00)#dummy    
                            disp.command(0x00)#start
                            disp.command(0x00)
                            disp.command(0x0f)#stop
                            disp.command(0x00)
                            disp.command(0xff)
                            disp.command(0x2f)#activate scroll

                else:
                    if(state != 0):
                        state = 0
                        disp.command(0x2E)#SSD1306_DEACTIVATE_SCROLL

            disp.command(0x2E)#SSD1306_DEACTIVATE_SCROLL

    else:
        print('Lines can be entered from 0 to 45.')
    
    
    
    
    
    
    
    
    
    
