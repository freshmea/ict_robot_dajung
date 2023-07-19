import cv2
import numpy as np
from PIL import Image
import time
    
class ObjectTracking():
        
    def __init__(self):
        self.lower =  np.array([0, 50, 50])
        self.upper =  np.array([10, 280, 280])
        self.center_x = None
        self.center_y = None
        self.area = 0
        self.max_index = -1
        
    def register_color(self, frame):

        height, width, channels = frame.shape
        width = int(width/2)
        height = int(height/2)

        #색상을 감지할 구역을 정하기
        crop_offset = 1
        frame_copy = frame[height-crop_offset:height+crop_offset, width-crop_offset:width+crop_offset]
        cv2.rectangle(frame, (width - crop_offset-1-2, height-crop_offset-1-2), (width+crop_offset + 2, height+crop_offset + 2), (255, 255, 255), 2)
        hsv = cv2.cvtColor(frame,cv2.COLOR_RGB2HSV)
        pixel = hsv[width][height]

        # 해당 구역의 색상의 평균값을 구합니다.
        frame_copy_hsv = cv2.cvtColor(frame_copy,cv2.COLOR_RGB2HSV)
        pixel = cv2.mean(frame_copy_hsv)

        # 색상값을 기준으로 색의 임계 상한과 하한 값을 지정합니다.
        h_offset = 10
        self.lower =  np.array([pixel[0] - h_offset, 50, 50])
        self.upper =  np.array([pixel[0] + h_offset, 280, 280])

        return frame
             
    
    def color_tracking(self, img_color):
        
        img_color = cv2.flip(img_color, 1)
        img_hsv = cv2.cvtColor(img_color, cv2.COLOR_RGB2HSV)
        img_mask = cv2.inRange(img_hsv, self.lower, self.upper)
        kernel = cv2.getStructuringElement( cv2.MORPH_RECT, ( 5, 5 ) )
        img_mask = cv2.morphologyEx(img_mask, cv2.MORPH_DILATE, kernel, iterations = 3)

        nlabels, labels, stats, centroids = cv2.connectedComponentsWithStats(img_mask)

        max = -1
        self.max_index = -1 

        for i in range(nlabels):

            if i < 1:
                continue
            area = stats[i, cv2.CC_STAT_AREA]
            if area > max:
                max = area
                self.max_index = i

        if self.max_index != -1:
            self.center_x = int(centroids[self.max_index, 0])
            self.center_y = int(centroids[self.max_index, 1]) 
            left = stats[self.max_index, cv2.CC_STAT_LEFT]
            top = stats[self.max_index, cv2.CC_STAT_TOP]
            width = stats[self.max_index, cv2.CC_STAT_WIDTH]
            height = stats[self.max_index, cv2.CC_STAT_HEIGHT]

            cv2.rectangle(img_color, (left, top), (left + width, top + height), (0, 200, 200), 2)
            cv2.circle(img_color, (self.center_x, self.center_y), 4, (200, 150, 0), -1)
           
            self.area = width*height
            
        return img_color
    
    def object_location(self):        
        if self.max_index != -1:            
             return (self.center_x,self.center_y)
        else :        
            return (-1,-1)
    
    def object_area(self):        
        if self.max_index != -1:            
             return (self.area)
        else :        
            return (-1)
        
        
        
        
        
        
        
        
        
        
        
        