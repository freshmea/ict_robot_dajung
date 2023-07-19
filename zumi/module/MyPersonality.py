from zumi.zumi import Zumi
from zumi.util.screen import Screen
from zumi.personality import Personality
from zumi.protocol import Note
import time

zumi = Zumi()
screen = Screen()
personality = Personality(zumi, screen)

def poor(): #불쌍해 보이는 감정
    for i in range(3):
        screen.sad()
        time.sleep(0.5)
        screen.close_eyes()
        time.sleep(0.5)
        zumi.reverse(5, 0.1)
        zumi.reverse(5, 0.1)
        
def wonder(): 
    #눈을 깜박이며 궁금해하는 감정 코드를 여기에 작성해 보세요.(pass는 지워주세요.)
    pass
    
    
def oops():
    # 놀라 잠에서 깨어나는 감정 코드를 여기에 작성해 보세요.(pass는 지워주세요.)
    pass