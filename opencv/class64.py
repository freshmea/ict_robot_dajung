import cv2 
import numpy as np 

src1 = cv2.imread('opencv/data/book1.jpg')
src2 = cv2.imread('opencv/data/book2.jpg')
gray1 = cv2.cvtColor(src1, cv2.COLOR_BGR2GRAY)
gray2 = cv2.cvtColor(src2, cv2.COLOR_BGR2GRAY)

orbF = cv2.ORB_create(nfeatures=800)

kp1, des1 = orbF.detectAndCompute(gray1, None)
kp2, des2 = orbF.detectAndCompute(gray2, None)

bf = cv2.BFMatcher_create(cv2.NORM_HAMMING, crossCheck=True)
matches = bf.match(des1, des2)

matches = sorted(matches, key = lambda m: m.distance)
minDist = matches[0].distance


good_matches = list(filter(lambda m: m.distance<5*minDist, matches))

dst = cv2.drawMatches(gray1,kp1,gray2,kp2,good_matches,None,flags=2)

src1_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches])
src2_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches])

H, mask = cv2.findHomography(src1_pts, src2_pts, cv2.RANSAC, 3.0)
mask_matches = mask.ravel().tolist()

draw_params =dict(matchColor = (255,0,0), singlePointColor = None, matchesMask = mask_matches, flags = 2)


h,w = gray1.shape
pts = np.float32([[0,0],[0,h-1],[w-1,h-1],[w-1,0]]).reshape(-1,1,2)
pts2 = cv2.perspectiveTransform(pts, H)
src2 = cv2.polylines(src2, [np.int32(pts2)], True, (0,0,255),2)

dst2 = cv2.drawMatches(src1,kp1,src2,kp2,good_matches,None,**draw_params)

cv2.imshow('dst', dst)
cv2.imshow('dst2', dst2)
cv2.imshow('src1', src1)
cv2.imshow('src2', src2)
cv2.waitKey()
cv2.destroyAllWindows()