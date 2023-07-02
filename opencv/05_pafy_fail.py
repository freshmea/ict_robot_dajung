import cv2, pafy, youtube_dl

url = 'https://www.youtube.com/watch?v=u_Q7Dkl7AIk'

video = pafy.new(url)
print(video.title)
print(video.rating)
print(video.duration)

best = video.getbest()
print(best.resolution)

cap = cv2.VideoCapture(best.url)

while True:
    retval, frame = cap.read()
    if not retval:
        break
    cv2.imshow('frame', frame)

    key = cv2.waitKey(25)
    if key == 27:
        break

if cap.isOpened():
    cap.release()
cv2.destroyAllWindows()