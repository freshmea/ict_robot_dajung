import cv2 
import numpy as np 

src = cv2.imread('opencv/data/rect.jpg')
gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
edges = cv2.Canny(gray, 50, 100)
lines = cv2.HoughLinesP(edges, 1, np.pi/180.0, 100, maxLineGap=5)
print(type(lines))
print('lines.shape', lines.shape)
for line in lines:
    x1, y1, x2, y2 = line[0]
    cv2.line(src,(x1,y1), (x2,y2), (0,0,255), 2)

cv2.imshow('src', src)
cv2.imshow('edges', edges)
cv2.waitKey()
cv2.destroyAllWindows()
