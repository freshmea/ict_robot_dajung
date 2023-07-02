import cv2

img = cv2.imread('opencv/data/lena.jpg')
img_gray = cv2.imread('opencv/data/lena.jpg', 0) #cv2.IMREAD_GRAYSCALE
# print(type(img))
# print(img.shape)
# print(cv2.IMREAD_GRAYSCALE)


cv2.imshow('Lena color', img)
cv2.imshow('Lena gray', img_gray)
# cv2.imwrite('opencv/data/out/lena.jpg', img)
cv2.waitKey(0)
cv2.destroyWindow('Lena color')
print( 'end of program')
