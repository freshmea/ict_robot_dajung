# 1
from zumi.zumi import Zumi 
from zumi.util.camera import Camera 
from zumi.util.screen import Screen 
from zumi.util.color_classifier import ColorClassifier 
import time 

camera = Camera()
screen = Screen()
zumi = Zumi()

# 2
user_name = 'aa'
demo_name = 'red_yellow_green'

knn = ColorClassifier(user_name=user_name)
train = knn.load_model(demo_name)
knn.fit('hsv')

# 3
camera.start_camera()

while True:
    user_input = input('예측을 하려면 Enter 종료는 q 를 누르세요.')
    if user_input == 'q':
        break
    image = camera.capture()
    predict = knn.predict(image)
    print(predict)

camera.close()

# 4
camera.start_camera()

while True:
    user_input = input('예측을 하려면 Enter 종료는 q 를 누르세요.')
    if user_input == 'q':
        break
    image = camera.capture()
    predict = knn.predict(image)
    if user_input == 'q':
        break
    image = camera.capture()
    predict = knn.predict(image)
    if predict == 'green':
        zumi.forward(speed = 30)
        print('green')
    elif predict == 'yellow':
        zumi.forward(speed = 15)
        print('yellow')
    elif predict == 'red':
        zumi.stop()
        print('red')
camera.close()

