import cv2
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mat_video import Video


def main():
    fig = plt.figure(figsize=(10,6))
    fig.canvas.manager.set_window_title('Video Capture')
    plt.axis('off')

    camera = Video('opencv/data/vtest.avi')
    ani = animation.FuncAnimation(fig, camera.updateFrame, interval=50)
    plt.show()
    camera.close()

if __name__ == '__main__':
    main()