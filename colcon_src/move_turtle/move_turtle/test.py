import cv2 

a = cv2.imread('traffic_stop.png')
print(a.shape)
cv2.imshow('a', a)
cv2.waitKey()
cv2.destroyAllWindows()
