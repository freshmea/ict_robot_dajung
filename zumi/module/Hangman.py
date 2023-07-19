import cv2
import numpy as np
import IPython.display
from IPython.display import display, HTML
from PIL import Image
import time
from ipywidgets import interact
from ipywidgets import Layout, Button, Box, FloatText, Textarea, Dropdown, Label, IntSlider
import ipywidgets as widgets
import random

import os

IMG_FILE_PATH = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))+ '/'
TEXT_FILE_PATH =  os.path.dirname(os.path.dirname(os.path.realpath(__file__)))+ '/'

lives = 7  
word = []
blank_word =[]
length = 0
box_1 = Box()
box_2 = Box()
box_3 = Box()
box_4 = Box()


imageLife7 = IMG_FILE_PATH + 'image/ch19/hangman/hangman7.png'
imageLife6 = IMG_FILE_PATH + 'image/ch19/hangman/hangman6.png'
imageLife5 = IMG_FILE_PATH + 'image/ch19/hangman/hangman5.png'
imageLife4 = IMG_FILE_PATH + 'image/ch19/hangman/hangman4.png'
imageLife3 = IMG_FILE_PATH + 'image/ch19/hangman/hangman3.png'
imageLife2 = IMG_FILE_PATH + 'image/ch19/hangman/hangman2.png'
imageLife1 = IMG_FILE_PATH + 'image/ch19/hangman/hangman1.png'
imageLife0 = IMG_FILE_PATH + 'image/ch19/hangman/hangman0.png'

wImg = widgets.Image(layout = widgets.Layout(border="solid"),
                     width=300, height=400) 

wLabel = Label(value='test', layout=Layout(flex='3 1 auto', width='auto'))

items_label = [
    Label(value='', layout=Layout(flex='1 1 auto', width='auto')),
    wLabel,
    Label(value='', layout=Layout(flex='1 1 auto', width='auto')),
 ]

box_layout = Layout(display='flex',flex_flow='row',
                    align_items='stretch',width='70%')
    
layout1 = widgets.Layout(
    width='40px', height='40px',
    border='1px solid black',
    button_style = 'info')

layout2 = widgets.Layout(
    width='83px', height='40px',
    border='1px solid black',
    button_style = 'info')

    
def on_button_clicked(b):   

    global lives
    global blank_word
    global word
    global length
    
    global box_1
    global box_2
    global box_3
    global box_4
    
    value = b.description
    
    if(value == 'reset'):        
        print("hangman을 다시 시작합니다.")   
        IPython.display.clear_output(wait=True)        
        play_game()
        
    else:

        if(lives > 0):   

            b.disabled=True        
            print(value)           


            if (value not in word):
                print("틀렸습니다.")
                lives -= 1        
                b. button_style = '' 

                if(lives == 6):
                    image1 = cv2.imread(imageLife6, cv2. COLOR_RGB2BGR)     
                elif(lives == 5):
                    image1 = cv2.imread(imageLife5, cv2. COLOR_RGB2BGR)     
                elif(lives == 4):
                    image1 = cv2.imread(imageLife4, cv2. COLOR_RGB2BGR)     
                elif(lives == 3):
                    image1 = cv2.imread(imageLife3, cv2. COLOR_RGB2BGR)     
                elif(lives == 2):
                    image1 = cv2.imread(imageLife2, cv2. COLOR_RGB2BGR)     
                elif(lives == 1):
                    image1 = cv2.imread(imageLife1, cv2. COLOR_RGB2BGR)     
                else:
                    image1 = cv2.imread(imageLife0, cv2. COLOR_RGB2BGR)  
                    print("hangman을 구하는데 실패하였습니다.")
                    print("정답은 " + str(''.join(word)) + " 입니다.")

                image1 = cv2.imencode(".png", image1)[1].tostring()     
                wImg.value = image1   

            else:
                print("맞았습니다.")
                for i in range(length):
                    if value == word[i]:        
                        blank_word[i] = value

                wLabel.value = ' '.join(blank_word)

                if(blank_word == word):
                    print("hangman을 구했습니다.")

            box_Lable = Box(children=items_label, layout=box_layout) 
            widgets.VBox([wImg, box_Lable, box_1, box_2,box_3,box_4])

        
def play_game():
    
    global lives
    global blank_word
    global word
    global length
    
    global box_1
    global box_2
    global box_3
    global box_4
    
    lives = 7  
    word = []
    blank_word =[]
    length = 0

    
    alphabet1 = 'A,B,C,D,E,F,G'.split(',')
    alphabet2 = 'H,I,J,K,L,M,N'.split(',')
    alphabet3 = 'O,P,Q,R,S,T,U'.split(',')
    alphabet4 = 'V,W,X,Y,Z'.split(',')

    
    buttons1 = []
    buttons2 = []
    buttons3 = []
    buttons4 = []

    
    for alphabet in alphabet1:   
        button1 = widgets.Button(description=alphabet, layout=layout1,button_style = 'info')    
        button1.on_click(on_button_clicked)
        buttons1.append(button1)

    for alphabet in alphabet2:        
        button2 = widgets.Button(description=alphabet, layout=layout1,button_style = 'success')
        button2.on_click(on_button_clicked)
        buttons2.append(button2)

    for alphabet in alphabet3:        
        button3 = widgets.Button(description=alphabet, layout=layout1,button_style = 'warning')
        button3.on_click(on_button_clicked)
        buttons3.append(button3)

    for alphabet in alphabet4:        
        button4 = widgets.Button(description=alphabet, layout=layout1,button_style = 'danger')
        button4.on_click(on_button_clicked)
        buttons4.append(button4)

    button4 = widgets.Button(description='reset', layout=layout2,button_style = 'danger')
    button4.on_click(on_button_clicked)
    buttons4.append(button4)
    
    #print(os.path.dirname(os.path.abspath('__file__')))
    #print(os.getcwd())
    
    #print(os.path.dirname(os.path.realpath(__file__)) )       
    #print(os.path.dirname(os.path.realpath('..')) )  
    #print(os.path.dirname(os.path.realpath(__file__+'.')) )  
    

    #print(os.path.dirname(os.path.realpath(__file__)) + "/futura.ttf")
    #print(sys.path)
  
    #currentdir = os.path.dirname(os.path.realpath(__file__))
    #parentdir = os.path.dirname(currentdir)+ '/image/ch19/hangman/hangman7.png'
    #parentdir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))+ '/image/ch19/hangman/hangman7.png'
    #print(parentdir)
    #sys.path.append(parentdir)

    #print(TEXT_FILE_PATH + 'image/ch19/hangman/hangman7.png')
     
    
    image1 = cv2.imread(imageLife7, cv2. COLOR_RGB2BGR)     
    image1 = cv2.imencode(".png", image1)[1].tostring()     
    wImg.value = image1   

    line = random.choice(open(TEXT_FILE_PATH + "module/words.txt").readlines()) 
    wordslist = line.split()
    length = sum(len(word) for word in wordslist)
    word = list(''.join(wordslist))
    blank_word = ("_ " * length).split()

    #print(word)
    #print(length)
    #print(blank_word)

    wLabel.value = ' '.join(blank_word)
    box_Lable = Box(children=items_label, layout=box_layout)

    box_1 = Box(children=buttons1, layout=box_layout)
    box_2 = Box(children=buttons2, layout=box_layout)
    box_3 = Box(children=buttons3, layout=box_layout)
    box_4 = Box(children=buttons4, layout=box_layout)
    BOX_NEW = widgets.VBox([wImg, box_Lable, box_1, box_2,box_3,box_4])
    display(BOX_NEW)

    js = "<script>$('.output_scroll').removeClass('output_scroll')</script>"
    display(HTML(js))
    
    #return BOX_NEW
    