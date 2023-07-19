from zumi.zumi import Zumi
#from zumi.util.camera import Camera
from zumi.util.screen import Screen
from zumi.protocol import Note

import time
import random
import IPython.display
from PIL import Image

from ipywidgets import interact
from ipywidgets import Layout, Button, Box, VBox, FloatText, Textarea, Dropdown, Label, IntSlider
import ipywidgets as widgets

from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np

zumi = Zumi()
screen = Screen()
camera = PiCamera()

zumiSpeed = 40
zumiDuration = 1
zumiAngle = 90
zumiStep = 5
    
def basic_drive_app():
    
    zumiSpeed = 40
    zumiDuration = 1
    zumiAngle = 90
    zumiStep = 5
    
    imageLocation = 'image/ch20/quickstart_guide_print.jpg'


    form_item_layout = Layout(
        display='flex',flex_flow='row',
        align_items = 'stretch',justify_content='space-between',
    )

    wSlide1 = widgets.IntSlider(
        value=40,min=0,max=80,step=1,
        disabled=False,continuous_update=False,
        )   

    wSlide2 = widgets.FloatSlider(
        value=1,min=0,max=10,step=0.5,
        disabled=False,continuous_update=False,
        )   

    wSlide3 = widgets.IntSlider(
        value=90,min=0,max=180,step=1,
        disabled=True,continuous_update=False,
        )   

    wImg = widgets.Image(layout = widgets.Layout(border="solid"),)
                        # width=300, height=400) 

    wToggleButtons = widgets.ToggleButtons(
        options=['forward()','reverse()', 'turn_left()', 'turn_right()'],
        value='forward()',description='move:',
        disabled=False,
    )

    wButton1 = Button(description='움직이기', layout=Layout(flex='1 1 auto', width='auto'), button_style='danger')
    wButton2 = Button(description='정지', layout=Layout(flex='1 1 auto', width='auto'), button_style='danger')

    wTextarea = widgets.Textarea(
        value='앞으로 이동',
        disabled=False
    )


    form_items1 = [
        Box([wImg,wToggleButtons], layout=form_item_layout),       
        Box([Label(value='속도'), wSlide1], layout=form_item_layout),     
        Box([Label(value='각도'), wSlide3], layout=form_item_layout),     
        Box([Label(value='지속 시간'),wSlide2], layout=form_item_layout),        
        Box([Label(value='설명'),wTextarea], layout=form_item_layout),    
        Box([wButton1], layout=form_item_layout)
    ]


    def reLoad(value):

        if(value == 'forward()'):
            wTextarea.value = '앞으로 이동'        
            wSlide1.disabled=False
            wSlide3.disabled=True        
            form1

        elif(value == 'reverse()'):
            wTextarea.value = '뒤로 이동'
            wSlide1.disabled=False
            wSlide3.disabled=True
            form1

        elif (value == 'turn_left()'):
            wTextarea.value = '왼쪽으로 회전'
            wSlide1.disabled=True
            wSlide3.disabled=False
            form1

        elif(value == 'turn_right()'):
            wTextarea.value = '오른쪽으로 회전'
            wSlide1.disabled=True
            wSlide3.disabled=False
            form1

    def on_Toggle_change(change):  
        reLoad(change['new'])

    def on_Speed_change(change):   
        global zumiSpeed
        zumiSpeed = change['new']

    def on_Duration_change(change):   
        global zumiDuration
        zumiDuration = change['new']

    def on_Angle_change(change):   
        global zumiAngle
        zumiAngle = change['new']

    def on_Start_clicked(b):
        global zumiSpeed
        global zumiDuration
        global zumiAngle

        if(wToggleButtons.value == 'forward()'):
            zumi.forward(speed=zumiSpeed, duration=zumiDuration)
        elif(wToggleButtons.value == 'reverse()'):
            zumi.reverse(speed=zumiSpeed, duration=zumiDuration)
        elif(wToggleButtons.value == 'turn_left()'):
            zumi.turn_left(desired_angle=zumiAngle, duration=zumiDuration)
        elif(wToggleButtons.value == 'turn_right()'):
            zumi.turn_right(desired_angle=zumiAngle, duration=zumiDuration)

    def on_Stop_clicked(b):  
        print('zumi stop')
        zumi.stop()   

    wToggleButtons.observe(on_Toggle_change, names='value')
    #wToggleButtons.style.button_width='90%'

    wSlide1.observe(on_Speed_change, names='value')
    wSlide2.observe(on_Duration_change, names='value')
    wSlide3.observe(on_Angle_change, names='value')

    wButton1.on_click(on_Start_clicked)
    wButton2.on_click(on_Stop_clicked)

    image1 = cv2.imread(imageLocation, cv2. COLOR_RGB2BGR)     
    image1 = cv2.imencode(".png", image1)[1].tostring()     
    wImg.value = image1 

    form1 = Box(form_items1, layout=Layout(
        display='flex',
        flex_flow='column',
        border='solid 2px',
        align_items='stretch',
        width='43%'
    ))

    zumi.reset_gyro()
    display(form1)

def draw_shape_app():
    
    global zumiSpeed
    global zumiDuration
    global zumiStep
    global zumiAngle
    global zumiSeconds
    
    zumiSpeed = 40
    zumiDuration = 1
    zumiAngle = 90
    zumiStep = 5

    imageLocation = 'image/ch20/quickstart_guide_print.jpg'
    
    form_item_layout = Layout(
        display='flex',flex_flow='row',
        align_items = 'stretch',
        justify_content='space-between',
    )

    wSlide1 = widgets.IntSlider(
        value=40,min=0,max=80,step=1,
        disabled=False, continuous_update=False,
        )   

    wSlide2 = widgets.IntSlider(
        value=5,min=1,max=10,step=1,
        disabled=False, continuous_update=False,
        )   

    wSlide3 = widgets.FloatSlider (
        value=1,min=0,max=10,step=0.5,
        disabled=True, continuous_update=False,
        )   

    wImg = widgets.Image(layout = widgets.Layout(border="solid"),)
                        # width=300, height=400) 

    wToggleButtons = widgets.ToggleButtons(
        options=['left_circle()','right_circle()', 'square_left()', 'square_right()'],
        value='left_circle()',
        description='move:',
        disabled=False,
        #button_style='', # 'success', 'info', 'warning', 'danger' or ''
    )

    wButton1 = Button(description='움직이기', layout=Layout(flex='1 1 auto', width='auto'), button_style='danger')
    wButton2 = Button(description='정지', layout=Layout(flex='1 1 auto', width='auto'), button_style='danger')

    wTextarea = widgets.Textarea(
        value='왼쪽으로 원돌기',
        #placeholder='Type something',
        disabled=False
    )

    form_items1 = [
        Box([wImg, 
             wToggleButtons], layout=form_item_layout),   

        Box([Label(value='속도'),          
             wSlide1], layout=form_item_layout), 

        Box([Label(value='스텝'),          
             wSlide2], layout=form_item_layout), 

        Box([Label(value='도달 시간'),          
             wSlide3], layout=form_item_layout),    

        Box([Label(value='설명'),wTextarea], layout=form_item_layout),  

        Box([wButton1], layout=form_item_layout),  
    #    Box([wButton2], layout=form_item_layout),  
    ]


    def reLoad(value):

        if(value == 'left_circle()'):
            wTextarea.value = '왼쪽으로 원돌기'        
            wSlide2.disabled=False
            wSlide3.disabled=True        
            form1

        elif(value == 'right_circle()'):
            wTextarea.value = '오른쪽으로 원돌기'
            wSlide2.disabled=False
            wSlide3.disabled=True
            form1

        elif (value == 'square_left()'):
            wTextarea.value = '왼쪽으로 사각형 이동'
            wSlide2.disabled=True
            wSlide3.disabled=False
            form1

        elif(value == 'square_right()'):
            wTextarea.value = '오른쪽으로 사각형 이동'
            wSlide2.disabled=True
            wSlide3.disabled=False
            form1

    def on_Toggle_change(change):  
        reLoad(change['new'])

    def on_Speed_change(change):   
        global zumiSpeed
        zumiSpeed = change['new']

    def on_Duration_change(change):   
        global zumiDuration
        zumiDuration = change['new']

    def on_Step_change(change):   
        global zumiStep
        zumiStep = change['new']

    def on_Start_clicked(b):
        global zumiSpeed
        global zumiDuration
        global zumiStep

        if(wToggleButtons.value == 'left_circle()'):
            zumi.left_circle(speed=zumiSpeed, step=zumiStep)
        elif(wToggleButtons.value == 'right_circle()'):
            zumi.right_circle(speed=zumiSpeed, step=zumiStep)
        elif(wToggleButtons.value == 'square_left()'):
            zumi.square_left(speed=zumiSpeed, seconds=zumiDuration)
        elif(wToggleButtons.value == 'square_right()'):
            zumi.square_right(speed=zumiSpeed, seconds=zumiDuration)

    def on_Stop_clicked(b):  
        print('zumi stop')
        zumi.stop()   

    wToggleButtons.observe(on_Toggle_change, names='value')
    #wToggleButtons.style.button_width='90%'

    wSlide1.observe(on_Speed_change, names='value')
    wSlide2.observe(on_Step_change, names='value')
    wSlide3.observe(on_Duration_change, names='value')

    wButton1.on_click(on_Start_clicked)
    wButton2.on_click(on_Stop_clicked)

    image1 = cv2.imread(imageLocation, cv2. COLOR_RGB2BGR)     
    image1 = cv2.imencode(".png", image1)[1].tostring()     
    wImg.value = image1 

    form1 = Box(form_items1, layout=Layout(
        display='flex',
        flex_flow='column',
        border='solid 2px',
        align_items='stretch',
        width='43%'
    ))

    zumi.reset_gyro()

    display(form1)

    
def control_app():
    
    imageLocation = 'image/ch20/quickstart_guide_print.jpg'
    imageLocation2 = 'image/ch20/zumi_topview.png'
    #################################################################################
    
    global zumiSpeed
    global zumiDuration
    global zumiStep
    global zumiAngle
    global zumiSeconds

    zumiSpeed = 40
    zumiDuration = 1
    zumiAngle = 90
    zumiStep = 5
    zumiSeconds = 1

    note_duration = 250

    width = 320
    height= 240   

    #################################################################################
    # 이벤트 함수
    def takePhoto(b):

        width = 320
        height= 240    
        camera.resolution = (width, height)
        camera.framerate = 20
        rawCapture = PiRGBArray(camera, size=(width, height))
        camera.capture(rawCapture, format="rgb")

        frame = rawCapture.array
        frame = cv2.flip(frame, 0)
        frame = cv2.flip(frame, 1)
        rawCapture.truncate(0)

        image1 = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        image1 = cv2.imencode(".png", image1)[1].tostring()     
        wImg.value = image1 

    def takePhoto_DISPLAY_OLED(b):

        width = 320
        height= 240    
        camera.resolution = (width, height)
        camera.framerate = 20
        rawCapture = PiRGBArray(camera, size=(width, height))
        camera.capture(rawCapture, format="rgb")

        frame = rawCapture.array
        frame = cv2.flip(frame, 0)
        frame = cv2.flip(frame, 1)
        rawCapture.truncate(0)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # Convert it to gray
        small = cv2.resize(gray, (128,64)) # Resize it to fit the screen
        screen.draw_image(Image.fromarray(small).convert('1')) # show the picture! 

    def on_Toggle_change(change):

        value = change['new'] 

        if(value == 'forward()'):
            wTextarea.value = '앞으로 이동'      
            speedSlider.disabled=False    
            durationSlider.disabled=False    
            angleSlider.disabled=True    
            stepSlider.disabled=True    
            secondsSlider.disabled=True    
            form1

        elif(value == 'reverse()'):
            wTextarea.value = '뒤로 이동'
            speedSlider.disabled=False    
            durationSlider.disabled=False    
            angleSlider.disabled=True    
            stepSlider.disabled=True    
            secondsSlider.disabled=True
            form1

        elif (value == 'turn_left()'):
            wTextarea.value = '왼쪽으로 회전'
            speedSlider.disabled=False    
            durationSlider.disabled=True    
            angleSlider.disabled=False    
            stepSlider.disabled=True    
            secondsSlider.disabled=True 
            form1

        elif(value == 'turn_right()'):
            wTextarea.value = '오른쪽으로 회전'
            speedSlider.disabled=False    
            durationSlider.disabled=True    
            angleSlider.disabled=False    
            stepSlider.disabled=True    
            secondsSlider.disabled=True   
            form1

        elif(value == 'left_circle()'):
            wTextarea.value = '왼쪽으로 원돌기'        
            speedSlider.disabled=False    
            durationSlider.disabled=True    
            angleSlider.disabled=True    
            stepSlider.disabled=False    
            secondsSlider.disabled=True       
            form1

        elif(value == 'right_circle()'):
            wTextarea.value = '오른쪽으로 원돌기'
            speedSlider.disabled=False    
            durationSlider.disabled=True    
            angleSlider.disabled=True    
            stepSlider.disabled=False    
            secondsSlider.disabled=True    
            form1

        elif (value == 'square_left()'):
            wTextarea.value = '왼쪽으로 사각형 이동'
            speedSlider.disabled=False    
            durationSlider.disabled=True    
            angleSlider.disabled=True    
            stepSlider.disabled=True    
            secondsSlider.disabled=False    
            form1

        elif(value == 'square_right()'):
            wTextarea.value = '오른쪽으로 사각형 이동'
            speedSlider.disabled=False    
            durationSlider.disabled=True    
            angleSlider.disabled=True    
            stepSlider.disabled=True    
            secondsSlider.disabled=False    
            form1


    def on_Start_clicked(b):

        global zumiSpeed
        global zumiDuration
        global zumiStep
        global zumiAngle
        global zumiSeconds

        if(wToggleButtons.value == 'forward()'):
            zumi.forward(speed=zumiSpeed, duration=zumiDuration)
        elif(wToggleButtons.value == 'reverse()'):
            zumi.reverse(speed=zumiSpeed, duration=zumiDuration)
        elif(wToggleButtons.value == 'turn_left()'):
            zumi.turn_left(desired_angle=zumiAngle, duration=zumiDuration)
        elif(wToggleButtons.value == 'turn_right()'):
            zumi.turn_right(desired_angle=zumiAngle, duration=zumiDuration)

        elif(wToggleButtons.value == 'left_circle()'):
            zumi.left_circle(speed=zumiSpeed, step=zumiStep)
        elif(wToggleButtons.value == 'right_circle()'):
            zumi.right_circle(speed=zumiSpeed, step=zumiStep)
        elif(wToggleButtons.value == 'square_left()'):
            zumi.square_left(speed=zumiSpeed, seconds=zumiSeconds)
        elif(wToggleButtons.value == 'square_right()'):
            zumi.square_right(speed=zumiSpeed, seconds=zumiSeconds)


    def on_Speed_change(change):   
        global zumiSpeed
        zumiSpeed = change['new']

    def on_Duration_change(change):   
        global zumiDuration
        zumiDuration = change['new']

    def on_Angle_change(change):   
        global zumiAngle
        zumiAngle = change['new']

    def on_Step_change(change):   
        global zumiStep
        zumiStep = change['new']

    def on_Seconds_change(change):   
        global zumiSeconds
        zumiSeconds = change['new']

    def on_EmotionButton_clicked(b):

        if b.description == 'sad':
            screen.sad()
        elif b.description == 'happy':
            screen.happy()
        elif b.description == 'blink':
            screen.blink()  
        elif b.description == 'angry':
            screen.angry()  
        elif b.description == 'hello':
            screen.hello()  
        elif b.description == 'glimmer':
            screen.glimmer()  
        elif b.description == 'sleeping':
            screen.sleeping()  

    def on_MelodyButton_clicked(b):

        if(b.description == 'C'):            
            note_type = Note.C4
        elif(b.description == 'C#'):
            note_type = Note.CS4
        elif(b.description == 'D'):
            note_type = Note.D4
        elif(b.description == 'D#'):
            note_type = Note.DS4
        elif(b.description == 'E'):
            note_type = Note.E4
        elif(b.description == 'F'):
            note_type = Note.F4
        elif(b.description == 'F#'):
            note_type = Note.FS4  
        elif(b.description == 'G'):
            note_type = Note.G4
        elif(b.description == 'G#'):
            note_type = Note.GS4    
        elif(b.description == 'A'):
            note_type = Note.A4
        elif(b.description == 'A#'):
            note_type = Note.AS4      
        elif(b.description == 'B'):
            note_type = Note.B4
        elif(b.description == 'C+'):
            note_type = Note.C5

        zumi.play_note(note_type, note_duration)


    # LED
    def on_LEDButton_change(b):    

        if b['owner'].description == '왼쪽 LED':
            if(b['new'] == True):
                zumi.led_on(0)        
            else:
                zumi.led_off(0)
        elif b['owner'].description == '오른쪽 LED':
            if(b['new'] == True):
                zumi.led_on(1)        
            else:
                zumi.led_off(1)

        elif b['owner'].description == '왼쪽뒤 LED':
            if(b['new'] == True):
                zumi.led_on(2)        
            else:
                zumi.led_off(2)

        elif b['owner'].description == '오른쪽뒤 LED':
            if(b['new'] == True):
                zumi.led_on(3)        
            else:
                zumi.led_off(3)



    #################################################################################

    # 위젯 제작
    # 이미지
    wImg = widgets.Image(layout = widgets.Layout(border="solid"),
                         width=300, height=400 
                        )
    # 토글 버튼
    wToggleButtons = widgets.ToggleButtons(
        options=['forward()','reverse()', 'turn_left()', 'turn_right()','left_circle()','right_circle()', 'square_left()', 'square_right()'],
        value='forward()',
        button_style='success'
    )

    # 슬라이더
    speedSlider = widgets.IntSlider(
        value=40,min=0,max=80,step=1,
        disabled=False,continuous_update=False,
        )   

    durationSlider = widgets.FloatSlider(
        value=1,min=0,max=10,step=0.5,
        disabled=False,continuous_update=False,
        )   

    angleSlider = widgets.IntSlider(
        value=90,min=0,max=180,step=1,
        disabled=True,continuous_update=False,
        )   

    stepSlider = widgets.IntSlider(
        value=5,min=1,max=10,step=1,
        disabled=True, continuous_update=False,
        )   

    secondsSlider = widgets.FloatSlider (
        value=1,min=0,max=10,step=0.5,
        disabled=True, continuous_update=False,
        )   

    # 버튼
    wButton1 = Button(description='움직이기', layout=Layout(flex='1 1 auto', width='auto'), button_style='danger')
    wButton2 = Button(description='사진 촬영', layout=Layout(flex='1 1 auto', width='auto'), button_style='danger')
    wButton3 = Button(description='사진 촬영 (OLED 표시)', layout=Layout(flex='1 1 auto', width='auto'), button_style='danger')


    # 텍스트
    wTextarea = widgets.Textarea(
        value='앞으로 이동',
        disabled=False
    )


    # LED


    wToggleButton1 = widgets.ToggleButton(
        description='왼쪽 LED',
        button_style = 'warning',
    )

    wToggleButton2 = widgets.ToggleButton(
        description='오른쪽 LED',
        button_style = 'warning',
    )

    wToggleButton3 = widgets.ToggleButton(
        description='왼쪽뒤 LED',
        button_style = 'warning',
    )

    wToggleButton4 = widgets.ToggleButton(
        description='오른쪽뒤 LED',
        button_style = 'warning',
    )

    wImg2 = widgets.Image(layout = widgets.Layout(border="solid"),
                    width=300/3, height=400/3,    
                    )


    #################################################################################

    # 아이템 설정

    item_layout_row = Layout(
        display='flex',
        flex_flow='row',
        align_items = 'stretch',
        justify_content='space-between',
        width='100%'   
    )

    item_layout_column = Layout(
        display='flex',
        flex_flow='column',
        align_items = 'stretch',
        justify_content='space-between',
        width='100%'   
    )


    item_layout_column_center = Layout(
        display='flex',
        flex_flow='column',
        align_items = 'center',
        justify_content='space-between',
        width='100%'   
    )
    #################################################################################

    # 위젯 묶음

    # 이미지
    items1 = [
        Box([wImg], layout=item_layout_row),
    ]

    # LED
    items1_2 = [
        Box([wToggleButton1,wToggleButton2], layout=item_layout_row)
    ]

    items1_3 = [
        Box([wImg2], layout=item_layout_column_center)
    ]
    items1_4 = [
        Box([wToggleButton3,wToggleButton4], layout=item_layout_row)
    ]

    # 이동
    items2_1 = [    
        Box([wToggleButtons], layout=item_layout_column)
    ]

    items2_2 = [    
        Box([Label(value='속도'), speedSlider], layout=item_layout_row),     
        Box([Label(value='지속 시간'), durationSlider], layout=item_layout_row),     
        Box([Label(value='각도'),angleSlider], layout=item_layout_row),      
        Box([Label(value='스텝'), stepSlider], layout=item_layout_row),
        Box([Label(value='도달 시간'), secondsSlider], layout=item_layout_row), 

        Box([wButton1], layout=item_layout_row),

       # Box([Label(value='설명'),wTextarea], layout=item_layout1),  
    ]

    # 카메라
    items3 = [
        Box([wButton2], layout=item_layout_row),
        Box([wButton3], layout=item_layout_row)
    ]



    # 표정
    emotions = 'sad,happy,blink,angry,hello,glimmer,sleeping'.split(',')

    items4 =[]

    for emotion in emotions:   
        button1 = widgets.Button(description=emotion, layout=Layout(flex='1 1 auto', width='auto'),button_style = 'info')    
        button1.on_click(on_EmotionButton_clicked)
        items4.append(button1)


    # 멜로디

    notes = 'C,C#,D,D#,E,F,F#,G,G#,A,A#,B,C+'.split(',')
    items5 =[]

    for note in notes: 
        button1 = widgets.Button(description=note, layout=Layout(flex='1 1 auto', width='auto'),button_style = 'warning')    
        button1.on_click(on_MelodyButton_clicked)
        items5.append(button1)

    #################################################################################

    # 이미지 읽기
    image1 = cv2.imread(imageLocation, cv2. COLOR_RGB2BGR)     
    image1 = cv2.imencode(".png", image1)[1].tostring()     
    wImg.value = image1 

    image2 = cv2.imread(imageLocation2, cv2. COLOR_RGB2BGR)     
    image2 = cv2.imencode(".png", image2)[1].tostring()     
    wImg2.value = image2 

    #################################################################################

    #################################################################################

    # 이벤트 연결

    wToggleButtons.observe(on_Toggle_change, names='value')

    wButton1.on_click(on_Start_clicked)
    wButton2.on_click(takePhoto)
    wButton3.on_click(takePhoto_DISPLAY_OLED)


    speedSlider.observe(on_Speed_change, names='value')
    durationSlider.observe(on_Duration_change, names='value')
    angleSlider.observe(on_Angle_change, names='value')
    stepSlider.observe(on_Step_change, names='value')
    secondsSlider.observe(on_Seconds_change, names='value')

    wToggleButton1.observe(on_LEDButton_change, names='value')
    wToggleButton2.observe(on_LEDButton_change, names='value')
    wToggleButton3.observe(on_LEDButton_change, names='value')
    wToggleButton4.observe(on_LEDButton_change, names='value')

    #################################################################################
    # 레이아웃 설정

    # 박스
    box_layout_row= Layout(
        display='flex',
        flex_flow='row',
        border='solid 1px',
        align_items='center',
        width='100%'   
    )

    box_layout_column= Layout(
        display='flex',
        flex_flow='column',
        border='solid 1px',
        align_items='stretch',
        width='100%'   
    )

    # 레이어
    box_layout_Layer_row= Layout(
        display='flex',
        flex_flow='row',
        border='solid 2px',
        align_items='stretch',
        width='100%'
    )
    box_layout_Layer_column = Layout(
        display='flex',
        flex_flow='column',
        border='solid 2px',
        align_items='stretch',
        width='100%'
    )

    # 폼
    box_layout_Form = Layout(
        display='flex',#inline-flex
        flex_flow='column',
        border='solid 2px',
        align_items='stretch',
        width='68%'
    )

    #################################################################################



    #################################################################################

    # 위젯 배치
    # 상단

    # 이미지
    part1 = Box(children=items1, layout=box_layout_row)

    part1_2 = Box(children=items1_2, layout=box_layout_row)
    part1_3 = Box(children=items1_3, layout=box_layout_row)
    part1_4 = Box(children=items1_4, layout=box_layout_row)
    layer1_1 = Box([part1_2,part1_3,part1_4],layout=box_layout_Layer_column)

    layer1 = Box([part1,layer1_1],layout=box_layout_Layer_row)


    # 이동
    part2_1 = Box(children=items2_1, layout=box_layout_row)
    part2_2 = Box(children=items2_2, layout=box_layout_column)
    layer2 = Box([part2_1,part2_2],layout=box_layout_Layer_column)

    # 카메라
    part3 = Box(children=items3, layout=box_layout_Layer_column)
    layer3 = Box([part3],layout=box_layout_Layer_row)

    # 표정
    part4 = Box(children=items4, layout=box_layout_Layer_column)
    layer4 = Box([part4],layout=box_layout_Layer_row)

    # 멜로디
    part5 = Box(children=items5, layout=box_layout_Layer_row)
    layer5 = Box([part5],layout=box_layout_Layer_row)


    accordion1 = widgets.Accordion(children=[layer2,layer3,layer4,layer5])
    accordion1.set_title(0, 'Move')
    accordion1.set_title(1, 'Camera')
    accordion1.set_title(2, 'Emotion')
    accordion1.set_title(3, 'Melody')


    #################################################################################

    # 전체 배치
    form1 = Box([layer1,accordion1],layout=box_layout_Form)

    #################################################################################

    # 실행

    zumi.reset_gyro()    
    
    display(form1)
    #################################################################################

