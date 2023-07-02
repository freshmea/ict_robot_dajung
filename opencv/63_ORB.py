import cv2 
import numpy as np 

def distance(f1,f2):
    x1, y1 = f1.pt
    x2, y2 = f2.pt
    return np.sqrt((x2-x1)**2+(y2-y1)**2)

def filteringByDistance(kp, distE= 0.5):
    size = len(kp)
    mask = np.arange(1,size+1).astype(np.bool8)
    for i, f1 in enumerate(kp):
        if not mask[i]:
            continue
        else:
            for j, f2 in enumerate(kp):
                if i == j:
                    continue
                if distance(f1,f2)< distE:
                    mask[j] = False
    np_kp = np.array(kp)
    return list(np_kp[mask])

src = cv2.imread('opencv/data/chessBoard.jpg')
gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)

orbF = cv2.ORB_create(scoreType = 1)

kp = orbF.detect(gray)
kp = sorted(kp, key=lambda f: f.response, reverse=True)
filtered_kp = list(filter(lambda f: f.response>50 , kp))
filtered_kp = filteringByDistance(kp, 10)
print(len(kp))
print(len(filtered_kp))

dst = cv2.drawKeypoints(src, filtered_kp, None, (255,0,0))

cv2.imshow('dst', dst)
cv2.waitKey()
cv2.destroyAllWindows()