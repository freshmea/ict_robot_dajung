import cv2 
import numpy as np 
from matplotlib import pyplot as plt
src = cv2.imread('opencv/data/people1.png')
src2 = cv2.imread('opencv/data/people.png')

hog1 = cv2.HOGDescriptor()
des1 = hog1.compute(src)

cv2.imshow('src', src)
cv2.imshow('src2', src2)
cv2.waitKey()
cv2.destroyAllWindows()

