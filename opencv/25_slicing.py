import cv2

img = cv2.imread('opencv/data/lena.jpg') #, cv2.IMREAD_GRAYSCALE)  # Numpy.array -- 모든 속성과 메소드 가 사용 가능

print (img[100, 200:210])

img[100:400, 200:300] = [255, 0, 0]
img[100:400, 300:400, 2] = 255 # BGR 채널에 접근 하기.
# img[100:400, 300:400, 0] = 0
# img[100:400, 300:400, 1] = 0
cv2.imshow('img', img)
cv2.waitKey()
cv2.destroyAllWindows()