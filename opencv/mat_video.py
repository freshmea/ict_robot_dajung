import cv2
import matplotlib.pyplot as plt

class Video:
    def __init__(self, device=0):
            self.cap = cv2.VideoCapture(device)
            self.retval, self.frame =  self.cap.read()
            self.im = plt.imshow(cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB))

    def updateFrame(self, k):
        self.retval, self.frame = self.cap.read()
        if self.retval:
            self.im.set_array(cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB))

    def close(self):
        if self.cap.isOpened():
            self.cap.release()
