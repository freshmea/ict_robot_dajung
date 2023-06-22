import cv2
from matplotlib import pyplot as plt
import pygame
imgBGR = cv2.imread('opencv/data/lena.jpg')

imgRGB = cv2.cvtColor(imgBGR, cv2.COLOR_BGR2RGB)
plt.axis('off')
plt.imshow(imgRGB)
plt.show()