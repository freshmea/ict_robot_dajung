import cv2
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mat_video10 import Video


def main():
    camera = Video('opencv/data/vtest.avi')
    plt.show()
    camera.close()

if __name__ == '__main__':
    main()