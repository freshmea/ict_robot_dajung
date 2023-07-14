import numpy as np
import pyzbar.pyzbar as pyzbar
import cv2
import time
import os


class Vision:

    def __init__(self):

        # paths for "haarcascade" files
        face_xml_path = "/usr/local/lib/python3.5/dist-packages/zumi/util/src/haarcascade_frontalface_default.xml"
        smile_xml_path = "/usr/local/lib/python3.5/dist-packages/zumi/util/src/haarcascade_smile.xml"
        stop_sign_xml_path = "/usr/local/lib/python3.5/dist-packages/zumi/util/src/stop_sign.xml"
        self.face_cascade = cv2.CascadeClassifier(face_xml_path)
        self.smile_cascade = cv2.CascadeClassifier(smile_xml_path)
        self.stop_sign_cascade = cv2.CascadeClassifier(stop_sign_xml_path)

    def detect_movement(self, img1, img2, sens=8, threshold=2):
        """
        returns True if movement is detected by the camera
        this method works best if you set the
        camera resolution to 80*64
        The first couple of images will be false positives
        since the camera will be adjusting for lighting
        make sure both images are the same size

        :param img1: the past image
        :param img2: the current image
        :param sens: sensitivity to change, the larger the more sensitive
        :param threshold: black and white threshold number, range 0-255
        :return: True or False (true if something detected)
        """
        # find dimensions of image
        height = img1.shape[0]
        width = img1.shape[1]

        # turn img1 and img2 into gray scaled images
        gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

        # get both images and subtract them from each other
        frameDelta = cv2.absdiff(gray1, gray2)

        # set a threshold on the image where any pixel can only be either 1(black) or 0(white)
        ret, thresh = cv2.threshold(frameDelta, threshold, 255, cv2.THRESH_BINARY)

        # this finds all "blobs" of white that appear and finds their coordinates and sizes
        contoursArray = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
                                         cv2.CHAIN_APPROX_SIMPLE)[-2]

        # check if something found
        if len(contoursArray) > 0:
            # grab the biggest contour blob found
            contour = contoursArray[-1]  # -1 goes the end of the list

            # find the x value, y value, width, and height of the rectangle
            # that bounds the contour
            x, y, w, h = cv2.boundingRect(contour)

            # the w is the width of the blob found and h the height
            # the area will be the product
            area = w * h

            # check if the area of movement is bigger than the threshold
            if area > width * height / sens:
                # if so there must be something moving
                return True
            else:
                return False
        else:
            return False

    def pic_on_movement(self, img1, img2, sens=8, threshold=2):
        """
        Takes a picture if movement is detected,
        takes in two images of the same size
        photo is saved in the same folder as the program is in
        with a time stamp as the file name

        :param img1: image before
        :param img2: image now
        :param sens: sensitivity to change, the larger the more sensitive
        :param threshold: black and white threshold number, range 0-255
        :return: nothing
        """

        # take 2 pictures
        # image = camera.capture()
        image = img1
        height = image.shape[0]
        width = image.shape[1]

        # and turn them into gray images, no color
        gray1 = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # image = camera.capture()
        image = img2
        gray2 = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # get both images and subtract them from each other
        frameDelta = cv2.absdiff(gray1, gray2)

        # set a threshold on the image where any pixel can only be either 1(black) or 0(white)
        ret, thresh = cv2.threshold(frameDelta, threshold, 255, cv2.THRESH_BINARY)

        # this finds all "blobs" of white that appear and finds their coordinates and sizes
        contoursArray = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
                                         cv2.CHAIN_APPROX_SIMPLE)[-2]

        # check
        if len(contoursArray) > 0:
            # grab the biggest contour blob found
            countour = contoursArray[-1]  # -1 goes the end of the list

            # find the x value, y value, width, and height of the rectangle
            # that bounds the contour
            x, y, w, h = cv2.boundingRect(countour)

            # the w is the width of the blob found and h the height
            # the area will be the product
            area = w * h

            # check if the area of movement is bigger than the threshold
            if (area > width * height / sens):
                file_name = str(time.time()) + ".jpg"

                # if so there must be something moving
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                # write the image, save the image with the specific file name
                cv2.imwrite(file_name, image)
                print("Saved photo at: " + os.getcwd() + "/" + file_name)

    # requires input frame that is grayscale
    def find_face(self, frame, bounding_box=True, scale_factor=1.05, min_neighbors=8, min_size=(40, 40),
                  max_size=(320, 320)):
        """
        finds a face in a given frame, the frames must be gray scaled.
        will draw a bounding box around the face, will be drawn on grayscale image
        :param frame: a gray scale image you wish to find a face in
        :param bounding_box: can set true or false if you want a bounding box drawn on image
        :param scale_factor: scaling of area
        :param min_neighbors: minimum neighbors of features
        :param min_size: minimum pixel area (width,height)
        :return: [x,y,w,h] the x and y coordinate along with the width and height
        """

        faces = self.face_cascade.detectMultiScale(frame, scaleFactor=scale_factor, minNeighbors=min_neighbors,
                                                   minSize=min_size, maxSize=max_size, flags=cv2.CASCADE_SCALE_IMAGE)
        if len(faces) > 0:
            for (x, y, w, h) in faces:
                # draw a bounding box on the stop sign found
                # x is x position in frame, y is
                # w is width of area enclosing stop sign and h is height
                color_of_box = (255, 255, 255)  # this is in (R,G,B) 8 bit 2^8=256
                if bounding_box:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), color_of_box, 2)
                return [x, y, w, h]
        else:
            return None

    def detect_face(self, frame, bounding_box=True, scale_factor=1.05, min_neighbors=8, min_size=(40, 40),
                    max_size=(320, 320)):
        """
        finds a face in a given frame
        will draw a white bounding box around the face.
        white box will be drawn on original image

        :param frame: a gray scale image you wish to find a face in
        :param bounding_box: can set true or false if you want a bounding box drawn on image
        :param scale_factor: scaling of area
        :param min_neighbors: minimum neighbors of features
        :param min_size: minimum pixel area (width,height)
        :return: boolean true or false whether it found a face or not
        """
        gray_frame = self.convert_to_gray(frame)
        faces = self.face_cascade.detectMultiScale(gray_frame, scaleFactor=scale_factor, minNeighbors=min_neighbors,
                                                   minSize=min_size, maxSize=max_size, flags=cv2.CASCADE_SCALE_IMAGE)
        if len(faces) > 0:
            for (x, y, w, h) in faces:
                # draw a bounding box on the stop sign found
                # x is x position in frame, y is
                # w is width of area enclosing stop sign and h is height
                color_of_box = (255, 255, 255)  # this is in (R,G,B) 8 bit 2^8=256
                if bounding_box:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), color_of_box, 2)
                return True
        else:
            return False

    # requires input frame that is grayscale
    def find_smile(self, frame, bounding_box=True, scale_factor=1.05, min_neighbors=8, min_size=(40, 40),
                   max_size=(320, 320)):
        """
        finds a smile in a given frame, the frames must be gray scaled.
        will draw a bounding box around the face, will be drawn on grayscale image
        this works better if you know where the face is and try
        to detect a smile in that area
        :param frame: a gray scale image you wish to find a smile in
        :param bounding_box: can set true or false if you want a bounding box drawn on image
        :param scale_factor: scaling of area
        :param min_neighbors: minimum neighbors of features
        :param min_size:  minimum pixel area (width,height)
        :return:[x,y,w,h] the x and y coordinate along with the width and height
        """

        smiles = self.smile_cascade.detectMultiScale(frame, scaleFactor=scale_factor, minNeighbors=min_neighbors,
                                                     minSize=min_size, maxSize=max_size, flags=cv2.CASCADE_SCALE_IMAGE)
        if len(smiles) > 0:
            for (x, y, w, h) in smiles:
                # draw a bounding box on the stop sign found
                # x is x positon in frame, y is
                # w is width of area enclosing stop sign and h is height
                color_of_box = (255, 255, 255)  # this is in (R,G,B) 8 bit 2^8=256
                if bounding_box:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), color_of_box, 2)
                return [x, y, w, h]
        else:
            return None

        # requires input frame that is grayscale

    def find_stop_sign(self, frame, bounding_box=True, scale_factor=1.05, min_neighbors=8, min_size=(40, 40),
                       max_size=(320, 320)):
        """
        finds a stop sign and draws a white bounding box around the area detected
        the bounding box is drawn on the input grayscale frame
        the bounding box will be white
        :param frame: input image
        :param bounding_box: Set to true or false to toggle bounding box
        :param scale_factor: scaling of area
        :param min_neighbors:  minimum neighbors of features
        :param min_size: minimum pixel area (width,height)
        :return:
        """

        signs = self.stop_sign_cascade.detectMultiScale(frame, scaleFactor=scale_factor, minNeighbors=min_neighbors,
                                                        minSize=min_size, maxSize=max_size,
                                                        flags=cv2.CASCADE_SCALE_IMAGE)
        if len(signs) > 0:
            for (x, y, w, h) in signs:
                # draw a bounding box on the stop sign found
                # x is x positon in frame, y is
                # w is width of area enclosing stop sign and h is height
                color_of_box = (255, 255, 255)  # this is in (R,G,B) 8 bit 2^8=256
                if bounding_box:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), color_of_box, 2)
                return [x, y, w, h]
        else:
            return None

    def detect_stop_sign(self, frame, bounding_box=True, scale_factor=1.05, min_neighbors=8, min_size=(40, 40),
                         max_size=(320, 320), color_of_box=(255, 255, 255)):
        """
        finds a face in a given frame, the frames must be gray scaled.
        will draw a bounding box around the face, will be drawn on the image
        :param frame: a gray scale image you wish to find a face in
        :param bounding_box: can set true or false if you want a bounding box drawn on image
        :param scale_factor: scaling of area
        :param min_neighbors: minimum neighbors of features
        :param min_size: minimum pixel area (width,height)
        :return: boolean true or false
        """
        gray_frame = self.convert_to_gray(frame)
        signs = self.stop_sign_cascade.detectMultiScale(gray_frame, scaleFactor=scale_factor,
                                                        minNeighbors=min_neighbors,
                                                        minSize=min_size, maxSize=max_size,
                                                        flags=cv2.CASCADE_SCALE_IMAGE)
        if len(signs) > 0:
            for (x, y, w, h) in signs:
                # draw a bounding box on the stop sign found
                # x is x positon in frame, y is
                # w is width of area enclosing stop sign and h is height
                cv2.rectangle(frame, (x, y), (x + w, y + h), color_of_box, 2)
            return True
        else:
            return False

    def convert_to_gray(self, frame):
        """
        converts an input image into a gray scale image
        :param frame: image frame
        :return: gray scale frame
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return gray

    def convert_to_hsv(self, frame):
        """
        converts an input image into a hue, saturation and value image
        :param frame: image frame
        :return: HSV colorspace frame
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        return hsv

    def find_QR_code(self, frame, draw_color=(255, 85, 255), draw_msg=True, bounding_box=True):
        """
        will find a QR code object and return a QR object
        only returns a single QR detected not multiple.
        :param frame: image frame
        :param draw_color: color to draw the bounding box (R,G,B)
        :param draw_msg: Draws the qr message on frame if True
        :param bounding_box: Draws a bounding box on the frame if true
        :return: returns the first QR code object found
        """
        decoded_objects = pyzbar.decode(frame)
        if len(decoded_objects) > 0:
            if draw_msg:
                obj = decoded_objects[0]
                data = obj.data.decode("utf-8")
                cv2.putText(frame, data, (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, draw_color, 2)
            if bounding_box:
                obj = decoded_objects[0]
                # This box will be a rotating bounding box
                # made with 4 lines from 4 points
                p1, p2, p3, p4 = obj.polygon
                cv2.line(frame, p1, p2, draw_color, 2)
                cv2.line(frame, p2, p3, draw_color, 2)
                cv2.line(frame, p4, p3, draw_color, 2)
                cv2.line(frame, p4, p1, draw_color, 2)
            return decoded_objects[0]
        else:
            return None

    def get_QR_message(self, QR_object):
        """
        returns only the message in the QR object
        you must use find_QR_code() to obtain a QR object
        :param QR_object: pass in a QR object using find_QR_code()
        :return: the string message on the QR found
        """
        if QR_object is not None:  # If the code finds more than one code...
            obj = QR_object
            data = obj.data.decode("utf-8")  # Decode the message
            return data
        else:
            return None

    def get_QR_message_blockly(self, frame):
        """
        returns only the message in the QR object
        :param frame: frame from the camera
        :return: the string message on the QR found
        """
        QR_object = self.find_QR_code(frame)
        if QR_object is not None:  # If the code finds more than one code...
            obj = QR_object
            data = obj.data.decode("utf-8")  # Decode the message
            return data
        else:
            return None

    def get_QR_center(self, QR_object):
        """
        returns the x,y coordinate of the QR object
        :param QR_object: pass in a QR object using find_QR_code()
        :return: returns a tuple of the x and y coordinates of QR code
        """
        if QR_object is not None:
            x, y, w, h = QR_object.rect
            x = x + int(w / 2)
            y = y + int(h / 2)
            return x, y
        else:
            return None

    def get_QR_dimensions(self, QR_object):
        """
        returns the width and height of the QR object
        :param QR_object: pass in a QR object using find_QR_code()
        :return: returns a tuple (w,h) width and height of QR code
        """
        if QR_object is not None:
            obj = QR_object
            x, y, w, h = obj.rect
            return w, h
        else:
            return None

    def get_QR_polygon(self, QR_object):
        """
        returns the four points that bound
        the QR object in a polygon
        p1,p2,p3,p4 each point has a coordinate (x,y)
        :param QR_object: pass in a QR object using find_QR_code()
        :return: returns a tuple p1,p2,p3,p4 which
        are the 4 points of the bounding polygon around the QR code
        """
        if QR_object is not None:
            p1, p2, p3, p4 = QR_object.polygon
            return p1, p2, p3, p4
        else:
            return None

    def warp_frame(self, frame, w_ratio=0.4, h_ratio=0.6):
        """
        #        location of the coordinates
        # --------------------------------------------\
        #        top_left          top_right          \
        #                                             \
        #                                             \
        #                                             \
        # bottom_left                     bottom_right\
        # --------------------------------------------\
        four points are used to warp an image, top_left, top_right, bottom_left, and bottom_right
        top left and top_right are controlled by the width_ratio and height_ratio
        bottom left and bottom right cannot be changed.

        width ratio controls how far into the center top_left & top_right go along the x axis
        heigth_ratio control how low into the center top_left & top_right go along the y axis

        :param frame: the input frame to be warped
        :param w_ratio: number between 0-1 controls x axis distortion
        :param h_ratio: number between 0-1  controls y axis distortion
        :return: frame that is warped
        """
        height, width, channels = frame.shape

        top_left = [int(width * w_ratio), int(height * h_ratio)]
        top_right = [int(width - width * w_ratio), int(height * h_ratio)]

        bottom_left = [0, height]
        bottom_right = [width, height]

        pts1 = np.float32([top_left, top_right, bottom_left, bottom_right])
        pts2 = np.float32([[0, 0], [width, 0], [0, height], [width, height]])
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
        result = cv2.warpPerspective(frame, matrix, (int(width), int(height)))
        return result

    def rotate_frame(self, frame, angle):
        """
        rotates a frame using an input angle in degrees
        :param frame: input image
        :param angle: angle to rotate in degrees
        :return: rotated image
        """
        height, width, channels = frame.shape
        # does some fancy math to rotate the image depending on the angle
        M = cv2.getRotationMatrix2D((width / 2, height / 2), angle, 1)
        result = cv2.warpAffine(frame, M, (width, height))
        return result

    def track_this_hue(self, image, color):
        """
        creates 2 frames that only allow one
        color selected to be filtered, list with 2 tuples of HSV ranges
        example:
            blueLower = (hue - h_range, sat - s_range, val - v_range)
            blueUpper = (hue + h_range, sat + s_range, val + v_range)
            color = [blueLower, blueUpper]

        :param image: original image to process
        :param color: list of HSV ranges
        :return: colorCutout, filteredFrame (2 frames that are filtered)
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        filteredFrame = cv2.inRange(hsv, color[0], color[1])
        colorCutout = cv2.bitwise_and(image, image, mask=filteredFrame)
        return colorCutout, filteredFrame

    def find_blue_object(self, frame, h_range=10, s_range=65, v_range=65, draw_color=(255, 85, 255),
                         bounding_box=True):
        """
        Finds the biggest blue object in the given frame
        and draws a pink bounding box around the
        object in the original frame

        -making the hue range bigger allows for a bigger range
        of colors to be detected if you want to narrow it down choose a smaller value
            hue goes from 0-180 degrees where 180 is a full rotation along the rainbow
        -the saturation range determines the window of saturation range to find the specified color
            saturation is the concentration of the color in the pixel
        -the value range determines the window of saturation range to find the specified color
            value is the amount of black in the pixel
        :param frame: colored frame
        :param h_range: hue range
        :param s_range: saturation range
        :param v_range: value range
        :param draw_color: (R,G,B) tuple of color of bounding box
        :param bounding_box: Boolean to toggle bounding box
        :return: [x,y,w,h] x and y coordinate and width and height
        """
        colorCutout, filteredFrame = self.blue_filter(frame, h_range, s_range, v_range)
        contoursArray = cv2.findContours(filteredFrame.copy(), cv2.RETR_EXTERNAL,
                                         cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(contoursArray) > 0:
            # only return one contour
            contour = contoursArray[-1]
            x, y, w, h = cv2.boundingRect(contour)
            if bounding_box:
                cv2.rectangle(frame, (x, y), (x + w, y + h), draw_color, 2)
            return [x, y, w, h]
        else:
            return None

    def find_green_object(self, frame, h_range=15, s_range=65, v_range=65, draw_color=(255, 85, 255),
                          bounding_box=True):
        """
        Finds the biggest green object in the given frame
        and draws a pink bounding box around the
        object in the original frame

        -making the hue range bigger allows for a bigger range
        of colors to be detected if you want to narrow it down choose a smaller value
            hue goes from 0-180 degrees where 180 is a full rotation along the rainbow
        -the saturation range determines the window of saturation range to find the specified color
            saturation is the concentration of the color in the pixel
        -the value range determines the window of saturation range to find the specified color
            value is the amount of black in the pixel
        :param frame: colored frame
        :param h_range: hue range
        :param s_range: saturation range
        :param v_range: value range
        :param draw_color: (R,G,B) tuple of color of bounding box
        :param bounding_box: Boolean to toggle bounding box
        :return: [x,y,w,h] x and y coordinate and width and height
        """
        colorCutout, filteredFrame = self.green_filter(frame, h_range, s_range, v_range)
        contoursArray = cv2.findContours(filteredFrame.copy(), cv2.RETR_EXTERNAL,
                                         cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(contoursArray) > 0:
            # only return one contour
            contour = contoursArray[-1]
            x, y, w, h = cv2.boundingRect(contour)
            if bounding_box:
                cv2.rectangle(frame, (x, y), (x + w, y + h), draw_color, 2)
            return [x, y, w, h]
        else:
            return None

    def find_yellow_object(self, frame, h_range=5, s_range=53, v_range=68, draw_color=(255, 85, 255),
                           bounding_box=True):
        """
        Finds the biggest yellow object in the given frame
        and draws a pink bounding box around the
        object in the original frame

        -making the hue range bigger allows for a bigger range
        of colors to be detected if you want to narrow it down choose a smaller value
            hue goes from 0-180 degrees where 180 is a full rotation along the rainbow
        -the saturation range determines the window of saturation range to find the specified color
            saturation is the concentration of the color in the pixel
        -the value range determines the window of saturation range to find the specified color
            value is the amount of black in the pixel
        :param frame: colored frame
        :param h_range: hue range
        :param s_range: saturation range
        :param v_range: value range
        :param draw_color: (R,G,B) tuple of color of bounding box
        :param bounding_box: Boolean to toggle bounding box
        :return: [x,y,w,h] x and y coordinate and width and height
        """
        colorCutout, filteredFrame = self.yellow_filter(frame, h_range, s_range, v_range)
        contoursArray = cv2.findContours(filteredFrame.copy(), cv2.RETR_EXTERNAL,
                                         cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(contoursArray) > 0:
            # only return one contour
            contour = contoursArray[-1]
            x, y, w, h = cv2.boundingRect(contour)
            if bounding_box:
                cv2.rectangle(frame, (x, y), (x + w, y + h), draw_color, 2)
            return [x, y, w, h]
        else:
            return None

    def find_orange_object(self, frame, h_range=5, s_range=53, v_range=68, draw_color=(255, 85, 255),
                           bounding_box=True):
        """
        Finds the biggest orange object in the given frame
        and draws a pink bounding box around the
        object in the original frame

        -making the hue range bigger allows for a bigger range
        of colors to be detected if you want to narrow it down choose a smaller value
            hue goes from 0-180 degrees where 180 is a full rotation along the rainbow
        -the saturation range determines the window of saturation range to find the specified color
            saturation is the concentration of the color in the pixel
        -the value range determines the window of saturation range to find the specified color
            value is the amount of black in the pixel
        :param frame: colored frame
        :param h_range: hue range
        :param s_range: saturation range
        :param v_range: value range
        :param draw_color: (R,G,B) tuple of color of bounding box
        :param bounding_box: Boolean to toggle bounding box
        :return: [x,y,w,h] x and y coordinate and width and height
        """
        colorCutout, filteredFrame = self.orange_filter(frame, h_range, s_range, v_range)
        contoursArray = cv2.findContours(filteredFrame.copy(), cv2.RETR_EXTERNAL,
                                         cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(contoursArray) > 0:
            # only return one contour
            contour = contoursArray[-1]
            x, y, w, h = cv2.boundingRect(contour)
            if bounding_box:
                cv2.rectangle(frame, (x, y), (x + w, y + h), draw_color, 2)
            return [x, y, w, h]
        else:
            return None

    def find_red_object(self, frame, h_range=6, s_range=50, v_range=70, draw_color=(255, 85, 255),
                        bounding_box=True):
        """
        Finds the biggest red object in the given frame
        and draws a pink bounding box around the
        object in the original frame

        -making the hue range bigger allows for a bigger range
        of colors to be detected if you want to narrow it down choose a smaller value
            hue goes from 0-180 degrees where 180 is a full rotation along the rainbow
        -the saturation range determines the window of saturation range to find the specified color
            saturation is the concentration of the color in the pixel
        -the value range determines the window of saturation range to find the specified color
            value is the amount of black in the pixel
        :param frame: colored frame
        :param h_range: hue range
        :param s_range: saturation range
        :param v_range: value range
        :param draw_color: (R,G,B) tuple of color of bounding box
        :param bounding_box: Boolean to toggle bounding box
        :return: [x,y,w,h] x and y coordinate and width and height
        """
        colorCutout, filteredFrame = self.red_filter(frame, h_range, s_range, v_range)
        contoursArray = cv2.findContours(filteredFrame.copy(), cv2.RETR_EXTERNAL,
                                         cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(contoursArray) > 0:
            # only return one contour
            contour = contoursArray[-1]
            x, y, w, h = cv2.boundingRect(contour)
            if bounding_box:
                cv2.rectangle(frame, (x, y), (x + w, y + h), draw_color, 2)
            return [x, y, w, h]
        else:
            return None

    def find_purple_object(self, frame, h_range=5, s_range=113, v_range=98, draw_color=(255, 85, 255),
                           bounding_box=True):
        """
        Finds the biggest purple object in the given frame
        and draws a pink bounding box around the
        object in the original frame

        -making the hue range bigger allows for a bigger range
        of colors to be detected if you want to narrow it down choose a smaller value
            hue goes from 0-180 degrees where 180 is a full rotation along the rainbow
        -the saturation range determines the window of saturation range to find the specified color
            saturation is the concentration of the color in the pixel
        -the value range determines the window of saturation range to find the specified color
            value is the amount of black in the pixel
        :param frame: colored frame
        :param h_range: hue range
        :param s_range: saturation range
        :param v_range: value range
        :param draw_color: (R,G,B) tuple of color of bounding box
        :param bounding_box: Boolean to toggle bounding box
        :return: [x,y,w,h] x and y coordinate and width and height
        """
        colorCutout, filteredFrame = self.red_filter(frame, h_range, s_range, v_range)
        contoursArray = cv2.findContours(filteredFrame.copy(), cv2.RETR_EXTERNAL,
                                         cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(contoursArray) > 0:
            # only return one contour
            contour = contoursArray[-1]
            x, y, w, h = cv2.boundingRect(contour)
            if bounding_box:
                cv2.rectangle(frame, (x, y), (x + w, y + h), draw_color, 2)
            return [x, y, w, h]
        else:
            return None

    def blue_filter(self, image, h_range=10, s_range=65, v_range=65):
        hue = 30
        sat = 135
        val = 135
        blueLower = (hue - h_range, sat - s_range, val - v_range)
        blueUpper = (hue + h_range, sat + s_range, val + v_range)
        blue = [blueLower, blueUpper]
        return self.track_this_hue(image, blue)

    def green_filter(self, image, h_range=15, s_range=65, v_range=65):
        hue = 55
        sat = 135
        val = 135
        greenLower = (hue - h_range, sat - s_range, val - v_range)
        greenUpper = (hue + h_range, sat + s_range, val + v_range)
        green = [greenLower, greenUpper]
        return self.track_this_hue(image, green)

    def yellow_filter(self, image, h_range=5, s_range=53, v_range=68):
        hue = 95
        sat = 202
        val = 187
        yellowLower = (hue - h_range, sat - s_range, val - v_range)
        yellowUpper = (hue + h_range, sat + s_range, val + v_range)
        yellow = [yellowLower, yellowUpper]
        return self.track_this_hue(image, yellow)

    def orange_filter(self, image, h_range=5, s_range=53, v_range=68):
        hue = 105
        sat = 202
        val = 187
        orangeLower = (hue - h_range, sat - s_range, val - v_range)
        orangeUpper = (hue + h_range, sat + s_range, val + v_range)
        orange = [orangeLower, orangeUpper]
        return self.track_this_hue(image, orange)

    def red_filter(self, image, h_range=6, s_range=50, v_range=70):
        hue = 120
        sat = 177
        val = 177
        redLower = (hue - h_range, sat - s_range, val - v_range)
        redUpper = (hue + h_range, sat + s_range, val + v_range)
        red = [redLower, redUpper]
        return self.track_this_hue(image, red)

    def purple_filter(self, image, h_range=5, s_range=113, v_range=98):
        hue = 155
        sat = 165
        val = 158
        purpleLower = (hue - h_range, sat - s_range, val - v_range)
        purpleUpper = (hue + h_range, sat + s_range, val + v_range)
        purple = [purpleLower, purpleUpper]
        return self.track_this_hue(image, purple)
