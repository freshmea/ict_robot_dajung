import cv2
import numpy as np

x = 0; y = 0; direction = 0
while True:
    key = cv2.waitKeyEx(30)
    if key == 0x1B:
        break
    elif key == 65363: # right 0xff53
        direction = 0
    elif key == 65364: # down 0xff54
        direction = 1
    elif key == 65361: # left 0xff51
        direction = 2
    elif key == 65362: # up 0xff52
        direction = 3
        
    if direction == 0: #right
        x += 10
    elif direction == 1: # down
        y += 10
    elif direction == 2: # left
        x -= 10
    elif direction == 3: # up
        y -= 10
    
    img = np.zeros(shape=(512,512,3), dtype=np.uint8) + 255 # 지우기
    cv2.circle(img, (x, y), 50, (0,0,255), -1) # 원 그리기
    cv2.imshow('img', img)

cv2.destroyAllWindows()