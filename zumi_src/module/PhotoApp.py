from picamera.array import PiRGBArray
from picamera import PiCamera
import IPython.display

#from ipywidgets import interact, Layout, Image, Select, Button, IntSlider, RadioButtons
from ipywidgets import interact, Layout
import ipywidgets as widgets 
import cv2
import numpy as np
from PIL import Image
import time

file_name = ''

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
 

def take_photo():
    global file_name
    print('- 사진을 찍습니다.')

    file_name = input('- 사진 이름은 ? ')

    print('지원 가능한 사진 크기')
    print('- 640 x 480, 320 x 240, 160 x 128')
    
    while True: 
        width = int(input('- 사진 길이는 ? '))
        if(width <= 640):
            break
            
    while True:    
        height= int(input('- 사진 높이는 ? '))
        if(height <= 480):
            break

    print('입력한 사진 크기는 %d x %d 입니다.'%(width, height))

    camera.resolution = (width, height)
    camera.framerate = 20
    rawCapture = PiRGBArray(camera, size=(width, height))

    print("- 셋 !")
    time.sleep(0.5)
    print("- 둘 !")
    time.sleep(0.5)
    print("- 하나 !")
    time.sleep(0.5)
    print("- 사진을 찍습니다 !")
    
    camera.capture(rawCapture, format="rgb")

    frame = rawCapture.array
    frame = cv2.flip(frame, 0)
    frame = cv2.flip(frame, 1)
    rawCapture.truncate(0)

    savePhoto = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    cv2.imwrite('photo/'+file_name + ".png" ,savePhoto) 
    
    print("- 사진 저장을 완료하였습니다")

    IPython.display.display(Image.fromarray(frame))
    IPython.display.clear_output(wait=True) 


def rgb_channel():
    global file_name    
    imageLocation = 'photo/zumi.png'
    
    if(file_name !=''):
        imageLocation = 'photo/'+file_name + ".png"     

    wImgC = widgets.Image(layout = Layout(border="solid"),
                          width=300, height=400 # 이미지 크기
                         )

    wSelect = widgets.Select(
        options=['RGB','R', 'G', 'B'],
        value='RGB',
        description='Channel:',
    )

    def on_RGB_clicked(b):    
        RGBChannelSelect(b['new'])

    def RGBChannelSelect(channel):

        image = cv2.imread(imageLocation) # 이미지 읽기
        b, g, r = cv2.split(image)

        if(channel =='RGB'):
            image = cv2.imencode(".png", image)[1].tostring()         
        elif(channel =='R'):        
            image = cv2.imencode(".png", r)[1].tostring()         
        elif(channel =='G'):
            image = cv2.imencode(".png", g)[1].tostring() 
        elif(channel =='B'):
            image = cv2.imencode(".png", b)[1].tostring() 

        wImgC.value = image 



    RGBChannelSelect('RGB')


    wSelect.observe(on_RGB_clicked, names = 'value')

    BOX_NEW = widgets.Box([wImgC, wSelect])
    display(BOX_NEW)
        

def mosaic():
    global file_name    
    imageLocation = 'photo/zumi.png'
    
    if(file_name !=''):
        imageLocation = 'photo/'+file_name + ".png" 
                
    #mosic_ratio = 0.05
    
    mosic_ratio = 0.2

    wImgM = widgets.Image(layout = Layout(border="solid"),
                          width=300, height=400 # 이미지 크기
                         )

    wbuttonM = widgets.Button(description='모자이크 만들기', 
                     layout= Layout(
                         width='160px', height='40px',
                          border='1px solid black')
                            )

    def on_mosaic_clicked(b):    
        mosaic_change()

    def mosaic_change(): 

        image = cv2.imread(imageLocation)

        # mosic_ratio = 모자이크 비율
        
        small = cv2.resize(image, None, fx=mosic_ratio, fy=mosic_ratio, interpolation=cv2.INTER_NEAREST)    
        image = cv2.resize(small, image.shape[:2][::-1], interpolation=cv2.INTER_NEAREST)

        image = cv2.imencode(".png", image)[1].tostring() 
        wImgM.value = image 


    image = cv2.imread(imageLocation) # 이미지 읽기
    originalImg = cv2.imencode(".png", image)[1].tostring() 
    wImgM.value = originalImg 

    wbuttonM.on_click(on_mosaic_clicked)

    BOX_NEW = widgets.VBox([wImgM, wbuttonM])
    display(BOX_NEW)
        
def transform():
    global file_name    
    imageLocation = 'photo/zumi.png'
    
    if(file_name !=''):
        imageLocation = 'photo/'+file_name + ".png" 
        
    #RotaionRatio = 0.5  #  회전시 잘리는 경우 1을 입력

    RotaionRatio = 1 #  회전시 잘리는 경우 1을 입력


    wImagR = widgets.Image(layout = Layout(border="solid"),
                          width=300, height=400 # 이미지 크기
                         )

    wRadioR = widgets.RadioButtons(
        options=['원위치','왼쪽으로 90도 회전', '오른쪽으로 90도 회전', '좌우 반전','상하 반전'],
        description='Rotate:',
        disabled=False
    )


    def on_Rotate_change(change):  
        ImageRotate(change['new'])

    def ImageRotate(direction):

        image = cv2.imread(imageLocation) # 이미지 읽기

        (height, width) = image.shape[:2]
        (center_height,center_width) = (height/2.0, width/2.0)

        if(direction == '왼쪽으로 90도 회전'):  
            angle = 90   
            M = cv2.getRotationMatrix2D((center_width, center_height), angle, RotaionRatio)     
            image = cv2.warpAffine(image, M, (width, height))

        elif(direction == '오른쪽으로 90도 회전') :  
            angle = -90       
            M = cv2.getRotationMatrix2D((center_width, center_height), angle, RotaionRatio)     
            image = cv2.warpAffine(image, M, (width, height))

        elif(direction == '좌우 반전') :  
            image = cv2.flip(image, 1) 

        elif(direction == '상하 반전') :  
            image = cv2.flip(image, 0) 

        originalImg = cv2.imencode(".png", image)[1].tostring() 
        wImagR.value = originalImg 



    ImageRotate('원위치')

    wRadioR.observe(on_Rotate_change, names='value')

    BOX_NEW = widgets.Box([wImagR, wRadioR])
    display(BOX_NEW)
    
def grayscale():
    
    global file_name    
    imageLocation = 'photo/zumi.png'
    
    if(file_name !=''):
        imageLocation = 'photo/'+file_name + ".png" 

    wImgB = widgets.Image(layout = Layout(border="solid"),
                          width=300, height=400 # 이미지 크기
                         )

    wSlideB = widgets.IntSlider(
        value=0, min=-100, max=100, step=1,
        description='variable:',
        continuous_update=False,
        orientation='vertical',
        )   

    def on_Bright_change(change):   
        changeGrayScale(change['new'])

    def changeGrayScale(bright):      
        image = cv2.imread(imageLocation, cv2. IMREAD_GRAYSCALE)    
        image = image + bright    
        blackImg = cv2.imencode(".png", image)[1].tostring()     
        wImgB.value = blackImg 


    changeGrayScale(0)

    wSlideB.observe(on_Bright_change, names='value')

    BOX_NEW = widgets.Box([wImgB, wSlideB])
    display(BOX_NEW)
    
    
def btn_borderline():
    
    global file_name    
    imageLocation = 'photo/zumi.png'
    
    if(file_name !=''):
        imageLocation = 'photo/'+file_name + ".png" 
        
    wImg1E = widgets.Image(layout = Layout(border="solid"), 
                          width=300, height=400 # 이미지 크기
                         )
    wImg2E = widgets.Image(layout = Layout(border="solid"), 
                          width=300, height=400 # 이미지 크기
                         )

    wbuttonE = widgets.Button(description='윤곽선 검출', 
                     layout= Layout(
                         width='160px', height='40px',
                          border='1px solid black')
                            )

    def on_edge_clicked(b):
        contour()

    def contour():

        image = cv2.imread(imageLocation) # 이미지 읽기

        gray_img =   cv2.cvtColor(image, cv2.COLOR_RGB2GRAY) # 흑백이미지로 변환
        canny_img =  cv2.Canny(gray_img, 0, 10)# Canny edge 알고리즘

        canny_img = cv2.imencode(".png", canny_img)[1].tostring() 
        wImg2E.value = canny_img 


    image = cv2.imread(imageLocation) # 이미지 읽기
    originalImg = cv2.imencode(".png", image)[1].tostring() 
    wImg1E.value = originalImg 
    wImg2E.value = originalImg 

    wbuttonE.on_click(on_edge_clicked)

    BOX_NEW = widgets.VBox([wImg1E, wImg2E, wbuttonE])
    display(BOX_NEW)

    
def slide_borderline():
    global file_name    
    imageLocation = 'photo/zumi.png'
    
    if(file_name !=''):
        imageLocation = 'photo/'+file_name + ".png" 
        
    wImg1E = widgets.Image(layout = Layout(border="solid"), 
                          width=300, height=400 # 이미지 크기
                         )

    wImg2E = widgets.Image(layout = Layout(border="solid"), 
                          width=300, height=400 # 이미지 크기
                         )

    wSlidE = widgets.IntSlider(
        value=10, min=0, max=1500, step=10,
        description='variable:',
        continuous_update=False,
        orientation='vertical',
        )   


    def on_edge_change(change):    
        contour(change['new'])

    def contour(range):

        image = cv2.imread(imageLocation) # 이미지 읽기

        gray_img =   cv2.cvtColor(image, cv2.COLOR_RGB2GRAY) # 흑백이미지로 변환
        originalImg = cv2.imencode(".png", image)[1].tostring() 
        wImg1E.value = originalImg 

        canny_img =  cv2.Canny(gray_img, 0, range)# Canny edge 알고리즘
        canny_img = cv2.imencode(".png", canny_img)[1].tostring() 
        wImg2E.value = canny_img 


    contour(10)

    wSlidE.observe(on_edge_change, names='value')

    BOX_NEW = widgets.Box([wImg1E, wImg2E, wSlidE])
    display(BOX_NEW)
    
    
    
 