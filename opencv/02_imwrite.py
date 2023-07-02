import cv2

img = cv2.imread('opencv/data/lena.jpg')

cv2.imwrite('opencv/data/out/Lena.bmp', img)
cv2.imwrite('opencv/data/out/Lena.png', img)
cv2.imwrite('opencv/data/out/Lena2.png', img, [cv2.IMWRITE_PNG_COMPRESSION, 9])
cv2.imwrite('opencv/data/out/Lena.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 90])