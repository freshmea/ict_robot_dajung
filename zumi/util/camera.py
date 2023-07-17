# by github.com/tawnkramer
import cv2
import os
import sys
import time
import subprocess

from picamera import PiCamera
from picamera.array import PiRGBArray
import IPython.display
import PIL.Image


class Camera:
    def __init__(self, image_w=160, image_h=128, image_d=3, framerate=20, auto_start=False, streaming=False):
        self.image_width = image_w
        self.image_heigth = image_h
        self.resolution = (image_w, image_h)
        self.framerate = framerate

        self.streaming = False

        if streaming:
            self.streaming = True

        # initialize the frame and the variable used to indicate
        # if the thread should be stopped
        self.frame = None
        self.on = True
        self.image_d = image_d
        self.camera = None
        self.count_show_image = 0
        if not auto_start:
            return
        # initialize the camera and stream
        self.camera = PiCamera()  # PiCamera gets resolution (height, width)
        self.camera.resolution = self.resolution
        self.camera.framerate = framerate
        # rotate the frame so is matches
        # the position inside of Zumi's mounting bracket
        self.camera.vflip = True
        self.camera.hflip = True
        self.rawCapture = PiRGBArray(self.camera, size=self.resolution)
        self.stream = self.camera.capture_continuous(self.rawCapture, format="rgb", use_video_port=True)

    def is_closed(self):
        if self.camera is None:
            return True
        return self.camera.closed

    def start_camera(self):
        if not self.is_closed():
            self.close()

        if self.streaming:
            '''
            Calls web-streaming function
            '''
            p = subprocess.Popen(['sudo', 'sh', '/usr/local/lib/python3.5/dist-packages/zumidashboard/shell_scripts/webstreaming.sh', '.'])
            return
        print('Starting PiCamera')
        self.camera = PiCamera()  # PiCamera gets resolution (height, width)
        self.camera.resolution = self.resolution
        self.camera.framerate = self.framerate
        self.camera.vflip = True
        self.camera.hflip = True
        self.rawCapture = PiRGBArray(self.camera, size=self.resolution)
        self.stream = self.camera.capture_continuous(self.rawCapture, format="rgb", use_video_port=True)


    def set_resolution(self, width, height):
        self.image_width = width
        self.image_heigth = height
        self.camera.resolution = (width, height)

    def capture(self):
        f = next(self.stream)
        frame = f.array
        self.rawCapture.truncate(0)
        if self.image_d == 1:
            frame = cv2.rgb2gray(frame)
        return frame

    def close(self):
        # indicate that the thread should be stopped
        self.on = False
        print('Closing PiCamera')
        time.sleep(.5)
        self.stream.close()
        self.rawCapture.close()
        self.camera.close()

    # takes and saves a photo, default name will be the current time
    def save_photo(self, file_name=str(time.time()), print_msg=True):
        file_name = file_name + ".jpg"
        self.camera.capture(file_name)
        if print_msg:
            print("Saved photo at: " + os.getcwd() + "/" + file_name)


    # must save the file locally ex: file name can be "test"
    # and the picture will be inside local folder as test.jpg
    # there are some issues trying to save to upper directories so avoid
    def save_image(self, image, file_name, print_msg=True):
        file_name = file_name + ".jpg"
        # convert to correct color space for jpeg
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # write the image, save the image with the specific file name
        cv2.imwrite(file_name, image)
        # print a message to the user
        if print_msg:
            print("Saved photo at: " + os.getcwd() + "/" + file_name)

    # TODO: ADD A RESIZE
    # camera.capture('foo.jpg', resize=(320, 240))
    # this method will record video for a duration given
    # the framerate will not match since the pi zero is slow.
    # the file will be saved in the local directory the path will be printed as output
    def record_video(self, duration, file_name=str(time.time()), fps=30.0, print_msg=True):
        fourcc = cv2.VideoWriter_fourcc(*'H264')
        file_name = file_name + ".mp4"
        # check if the file exists
        if os.path.isfile(file_name):
            # if it exists remove it
            os.remove(file_name)
        video_writer = cv2.VideoWriter(file_name, fourcc, fps, self.resolution)
        time_elapsed = 0
        time_start = time.time()
        # record for the duration set
        while duration > time_elapsed:
            image = self.capture()
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            video_writer.write(image)
            time_elapsed = time.time() - time_start
        video_writer.release()
        del fourcc
        del video_writer
        if print_msg:
            print("Saved video at: " + os.getcwd() + "/" + file_name)

    # set the brightness of the camera
    # if too much sunlight set this lower
    # max val = 255 min val = 0
    def set_brightness(self, val):
        self.camera.brightness = val

    # allows user to set the ISO value
    def set_iso(self, val):
        self.camera.iso = val

    # allows user to set the shutter speed value in microseconds
    def set_shutter_speed(self, microseconds):
        self.camera.shutter_speed = microseconds

    # Displays an image taken, only works if using Jupyter notebook
    def show_image(self, frame, blockly=False):
        if blockly is True:
            # will save the current image passed in to a folder that blockly can read from, to display the image
            self.count_show_image += 1
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            cv2.imwrite('/usr/local/lib/python3.5/dist-packages/zumidashboard/dashboard/code_editor_image' + str(self.count_show_image % 5)  + '.jpg', frame)
            time.sleep(0.5)
            print('show image <' + str(self.count_show_image % 5) + '>')
            return frame
        else:
            # otherwise display the image using the jupyter notebook display
            IPython.display.display(PIL.Image.fromarray(frame))

    # Remove all output including images and print output on Jupyter notebook
    def clear_output(self):
        IPython.display.clear_output(wait=True)
