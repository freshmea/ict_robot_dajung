import cv2

img = cv2.imread('opencv/data/lena.jpg') #, cv2.IMREAD_GRAYSCALE)  # Numpy.array -- 모든 속성과 메소드 가 사용 가능

print(img[100, 200]) # x = 200, y = 100
# for x in range(10):
#     for y in range(10):
img[100,200] = [0, 0, 0]
cv2.circle(img, (200,100), 10,(0,0,255)) # x = 200, y = 100


cv2.imshow('img', img)
cv2.waitKey()
cv2.destroyAllWindows()