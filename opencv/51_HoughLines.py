import cv2 
import numpy as np 

src = cv2.imread('opencv/data/rect.jpg')
gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
edges = cv2.Canny(gray, 50, 100)
lines = cv2.HoughLines(edges, 1, np.pi/180.0, 100)
print(type(lines))
print('lines.shape', lines.shape)
for line in lines:
    rho, theta = line[0]
    c = np.cos(theta)
    s = np.sin(theta)
    x0 = c*rho
    y0 = s*rho
    x1 = int(x0 + 1000*(-s))
    y1 = int(y0 + 1000*(c))
    x2 = int(x0 - 1000*(-s))
    y2 = int(y0 - 1000*(c))
    cv2.line(src, (x1,y1), (x2,y2), (0,0,255), 2)

cv2.imshow('src', src)
cv2.imshow('edges', edges)
cv2.waitKey()
cv2.destroyAllWindows()
