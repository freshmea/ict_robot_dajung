import cv2
from matplotlib import pyplot as plt
imgBGR = cv2.imread('opencv/data/lena.jpg')

# imgRGB = cv2.cvtColor(imgBGR, cv2.COLOR_BGR2RGB)
b,g,r = cv2.split(imgBGR)
imgRGB = cv2.merge([r,g,b])
plt.axis('off')
plt.imshow(imgRGB) # b, g, r, imgBGR
plt.show()