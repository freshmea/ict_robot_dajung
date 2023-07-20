from zumi.zumi import Zumi
from zumi.protocol import Note
import IPython.display
import ipywidgets as widgets
import cv2
import time

zumi=Zumi()

#1
time = 1

durationBit = widgets.BoundedIntText(
    value = 1,
    min = 0,
    max = 5,
    step = 1,
    description = 'Duration : ',
    disabled = False
)

runBtn = widgets.Button(
    description = 'RUN',
    button_style = 'success'
)

def change_duration(change):
    global time
    time = change['new']
    
def run_forward(val):
    global time
    zumi.forward(duration = time)
    
durationBit.observe(change_duration, names= 'value')
runBtn.on_click(run_forward)

display(durationBit, runBtn)


#2
with open('image/ch19/character.png', 'rb') as file:
    image = file.read()
widgets.Image(
    value = image,
    format = 'png',
    width = 300,
    height = 400
)

#3
import cv2
import numpy as np 
from PIL import Image

#4
play = widgets.Play(
    value=0,
    min=0,
    max=100,
    step=1,
    description = 'Press play',
    disabled=False
)

slider = widgets.IntSlider()
widgets.jslink((play, 'value'), (slider, 'value'))
widgets.HBox([play, slider])

#5
nameList=[]

for i in range(20):
    name = str(i)+'.png'
    nameList.append(name)

imageList =[]
for fileName in nameList:
    file = cv2.imread('image/ch19/animation_image/'+fileName, cv2.COLOR_RGB2BGR)
    file = cv2.imencode('.png', file)[1].tostring()
    imageList.append(file)
    
name = imageList[0]

screen = widgets.Image(
    value = name,
    format='png',
    width=300,
    height=300,
    continue_update = True
)

def slider_func(change):
    screen.value = imageList[change['new']]

slider = widgets.IntSlider(min=0, max=19, step=1, continue_update = True)
slider.observe(slider_func, names= 'value')

play = widgets.Play(
    value=0,
    min=0,
    max=20,
    step=1,
    description = 'Press play',
    disabled=False
)

widgets.jslink((play, 'value'), (slider, 'value'))
animationPlay = widgets.HBox([play, slider])

display(animationPlay, screen)
