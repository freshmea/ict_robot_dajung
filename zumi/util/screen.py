import time
import Adafruit_SSD1306
import os
from PIL import Image, ImageFont, ImageDraw
import math
import smbus2


class Screen:
    # Raspberry Pi pin configuration:
    RST = 24
    EYE_IMAGE_FOLDER_PATH = os.path.dirname(os.path.abspath(__file__)) + '/images/'
    TEXT_FILE_PATH = os.path.dirname(os.path.abspath(__file__)) + "/futura.ttf"
    EXCITED = {"excited1", "excited2", "excited3"}

    def __init__(self, clear=True):
        # Constants
        self.SSD1306_I2C_ADDRESS = 0x3C  # 011110+SA0+RW - 0x3C or 0x3D
        self.SSD1306_SETCONTRAST = 0x81
        self.SSD1306_DISPLAYALLON_RESUME = 0xA4
        self.SSD1306_DISPLAYALLON = 0xA5
        self.SSD1306_NORMALDISPLAY = 0xA6
        self.SSD1306_INVERTDISPLAY = 0xA7
        self.SSD1306_DISPLAYOFF = 0xAE
        self.SSD1306_DISPLAYON = 0xAF
        self.SSD1306_SETDISPLAYOFFSET = 0xD3
        self.SSD1306_SETCOMPINS = 0xDA
        self.SSD1306_SETVCOMDETECT = 0xDB
        self.SSD1306_SETDISPLAYCLOCKDIV = 0xD5
        self.SSD1306_SETPRECHARGE = 0xD9
        self.SSD1306_SETMULTIPLEX = 0xA8
        self.SSD1306_SETLOWCOLUMN = 0x00
        self.SSD1306_SETHIGHCOLUMN = 0x10
        self.SSD1306_SETSTARTLINE = 0x40
        self.SSD1306_MEMORYMODE = 0x20
        self.SSD1306_COLUMNADDR = 0x21
        self.SSD1306_PAGEADDR = 0x22
        self.SSD1306_COMSCANINC = 0xC0
        self.SSD1306_COMSCANDEC = 0xC8
        self.SSD1306_SEGREMAP = 0xA0
        self.SSD1306_CHARGEPUMP = 0x8D
        self.SSD1306_EXTERNALVCC = 0x1
        self.SSD1306_SWITCHCAPVCC = 0x2

        # Scrolling constants
        self.SSD1306_ACTIVATE_SCROLL = 0x2F
        self.SSD1306_DEACTIVATE_SCROLL = 0x2E
        self.SSD1306_SET_VERTICAL_SCROLL_AREA = 0xA3
        self.SSD1306_RIGHT_HORIZONTAL_SCROLL = 0x26
        self.SSD1306_LEFT_HORIZONTAL_SCROLL = 0x27
        self.SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL = 0x29
        self.SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL = 0x2A
        try:
            # 128x64 display with hardware I2C:
            # ADDED
            self.bus = smbus2.SMBus(1)

            self.disp = Adafruit_SSD1306.SSD1306_128_64(rst=self.RST)
            # Initialize library.
            self.disp.begin()

            self.width = self.disp.width
            self.height = self.disp.height

            # ADDED
            self._vccstate = None
            self._pages = self.height // 8
            self._buffer = [0] * (self.width * self._pages)

            # create a a new black image of 128X64 pixels
            self.screen_image = Image.new('1', (self.width, self.height))
            self.draw = ImageDraw.Draw(self.screen_image)
            self.stop_scroll()  # just in case the screen is still scrolling
            # Clear display.
            if clear:
                self.disp.clear()
                self.disp.display()
        except:
            print("OLED screen is not connected")

    def command(self, value):
        """Send command byte to display."""
        # I2C write.
        control = 0x00  # Co = 0, DC = 0
        value = value & 0xff
        self.bus.write_byte_data(self.SSD1306_I2C_ADDRESS, control, value)

    def begin(self):
        """Initialize display."""
        # Save vcc state.
        self._vccstate = self.SSD1306_SWITCHCAPVCC
        # initialize display.
        self._initialize()
        # Turn on the display.
        self.command(self.SSD1306_DISPLAYON)

    def _initialize(self):
        # 128x64 pixel specific initialization.
        self.command(self.SSD1306_DISPLAYOFF)  # 0xAE
        self.command(self.SSD1306_SETDISPLAYCLOCKDIV)  # 0xD5
        self.command(0x80)  # the suggested ratio 0x80
        self.command(self.SSD1306_SETMULTIPLEX)  # 0xA8 and 0X3F
        self.command(0x3F)  # 0x3F is 63 for the Mux
        self.command(self.SSD1306_SETDISPLAYOFFSET)  # 0xD3
        self.command(0x0)  # no offset
        self.command(self.SSD1306_SETSTARTLINE | 0x0)  # line #0
        self.command(self.SSD1306_CHARGEPUMP)  # 0x8D
        if self._vccstate == self.SSD1306_EXTERNALVCC:
            self.command(0x10)
        else:
            self.command(0x14)
        self.command(self.SSD1306_MEMORYMODE)  # 0x20
        self.command(0x00)  # 0x0 act like ks0108
        self.command(self.SSD1306_SEGREMAP | 0x1)
        self.command(self.SSD1306_COMSCANDEC)
        self.command(self.SSD1306_SETCOMPINS)  # 0xDA
        self.command(0x12)
        self.command(self.SSD1306_SETCONTRAST)  # 0x81
        if self._vccstate == self.SSD1306_EXTERNALVCC:
            self.command(0x9F)
        else:
            self.command(0xCF)
        self.command(self.SSD1306_SETPRECHARGE)  # 0xd9
        if self._vccstate == self.SSD1306_EXTERNALVCC:
            self.command(0x22)
        else:
            self.command(0xF1)
        self.command(self.SSD1306_SETVCOMDETECT)  # 0xDB
        self.command(0x40)
        self.command(self.SSD1306_DISPLAYALLON_RESUME)  # 0xA4
        self.command(self.SSD1306_NORMALDISPLAY)  # 0xA6

    def display(self):
        """Write display buffer to physical display."""
        self.command(self.SSD1306_COLUMNADDR)
        self.command(0)  # Column start address. (0 = reset)
        self.command(self.width - 1)  # Column end address.
        self.command(self.SSD1306_PAGEADDR)
        self.command(0)  # Page start address. (0 = reset)
        self.command(self._pages - 1)  # Page end address.
        # Write buffer data.
        for i in range(0, len(self._buffer), 16):
            control = 0x40  # Co = 0, DC = 0
            self.bus.write_i2c_block_data(self.SSD1306_I2C_ADDRESS, control, self._buffer[i:i + 16])

    def image(self, image):
        """Set buffer to value of Python Imaging Library image.  The image should
        be in 1 bit mode and a size equal to the display size.
        """
        if image.mode != '1':
            raise ValueError('Image must be in mode 1.')
        imwidth, imheight = image.size
        if imwidth != self.width or imheight != self.height:
            raise ValueError('Image must be same dimensions as display ({0}x{1}).' \
                             .format(self.width, self.height))
        # Grab all the pixels from the image, faster than getpixel.
        pix = image.load()
        # Iterate through the memory pages
        index = 0
        for page in range(self._pages):
            # Iterate through all x axis columns.
            for x in range(self.width):
                # Set the bits for the column of pixels at the current position.
                bits = 0
                # Don't use range here as it's a bit slow
                for bit in [0, 1, 2, 3, 4, 5, 6, 7]:
                    bits = bits << 1
                    bits |= 0 if pix[(x, page * 8 + 7 - bit)] == 0 else 1
                # Update buffer byte and increment to next byte.
                self._buffer[index] = bits
                index += 1

    def clear(self):
        """Clear contents of image buffer."""
        self._buffer = [0] * (self.width * self._pages)

    # the following draw shapes will grab the current drawn image
    # draw and update that draw image and then display it by default

    def draw_rect(self, x, y, width, height, thickness=1, fill_in=0):
        """
                   width
        (x,y)|---------------|
             |               | height
             |_______________|

        draws a rectangle on the screen added in 1.4
        :param x: the top left corner x coordinate
        :param y: the top left corner y coordinate
        :param width: width of rectangle
        :param height: height of rectangle
        :param thickness: thickness of rectangle border
        :param fill_in: 0 would be white, 1 would be black
        :return: nothing
        """
        self.draw.rectangle((x, y, x + width, y + height), outline=thickness, fill=fill_in)
        self.disp.image(self.screen_image)
        self.disp.display()

    # draws a square given 1 top left coordinates x1,y1 and a width? added in 1.4
    def draw_square(self, x, y, width, thickness=1, fill_in=0):
        """
               width
        (x,y)|------|
             |      | width
             |______|

        draws a square on the screen
        :param x: top left corner x coordinate
        :param y: top left corner y coordinate
        :param width:
        :param thickness:
        :param fill_in:
        :return:
        """
        self.draw.rectangle((x, y, x + width, y + width), outline=thickness, fill=fill_in)
        self.disp.image(self.screen_image)
        self.disp.display()

    # draws a pixel or a point on x,y coordinate added in 1.4
    def draw_point(self, x, y, fill_in=1):
        """
        draws a single pixel in the set x and y coordinate
        :param x: x coordinate
        :param y: y coordinate
        :param fill_in:
        :return: nothing
        """
        self.draw.point((x, y), fill=fill_in)
        self.disp.image(self.screen_image)
        self.disp.display()

    # draws a line between points x1,y1 and x2,y2 added in 1.4
    def draw_line(self, x1, y1, x2, y2, thickness=1, fill_in=1):
        """
        (x1,y1) \
                 \
                  \
                   \ (x2,y2)

        draws a line between points (x1,y1) and (x2,y2)
        :param x1: point 1 x coordinate
        :param y1: point 1 y coordinate
        :param x2: point 2 x coordinate
        :param y2: point 2 y coordinate
        :param thickness: pixel width of line
        :param fill_in:
        :return: nothing
        """
        self.draw.line(((x1, y1), (x2, y2)), width=thickness, fill=fill_in)
        self.disp.image(self.screen_image)
        self.disp.display()

    def draw_polygon(self, points_list, fill_in=1):
        """
        draws lines between points in given
        list [(x1,y1),...,(xn,yn)] and
        fills in the shape *added in 1.4
        :param points_list: [x1,y1,x2,y2,....xn,yn] however many points you want
        :param fill_in:
        :return: nothing
        """
        self.draw.polygon(points_list, fill=fill_in)
        self.disp.image(self.screen_image)
        self.disp.display()

    def draw_triangle(self, x1, y1, x2, y2, x3, y3, fill_in=1):
        """
                   p2
                /|
               / |
              /  |
          p1 /___| p3

        draws a triangle with 3 points in given
        list p1,p2,p3 and
        fills in the triangle
        added in 1.4
        :param x1: point 1 x coordinate
        :param y1: point 1 y coordinate
        :param x2: point 2 x coordinate
        :param y2: point 2 y coordinate
        :param x3: point 3 x coordinate
        :param y3: point 3 y coordinate
        :param fill_in:
        :return:
        """
        points_list = [(x1, y1), (x2, y2), (x3, y3)]
        self.draw.polygon(points_list, fill=fill_in)
        self.disp.image(self.screen_image)
        self.disp.display()

    def draw_arc(self, x1, y1, x2, y2, start_ang, end_ang, fill_in=1):
        """
        (x1,y1) |---------------------------|
                |            ...            |
                |      .             .      |
                |   .                   .   |
                | .                       . |
                |. ang_s            end_ang.|
                -----------------------------(x2,y2)
        draws an arc within boundaries of x1,y1, and x2,y2?
        with an arc that starts at angle start_ang and ends at end_ang
        added in 1.4
        :param x1: top left x coordinate of rectangle enclosing the arc
        :param y1: top left y coordinate of rectangle enclosing the arc
        :param x2: bottom left x coordinate of rectangle enclosing the arc
        :param y2: bottom left y coordinate of rectangle enclosing the arc
        :param start_ang: angle which arc will start at
        :param end_ang:  angle which arc will end at
        :param fill_in: whether the arc will be filled in
        :return: nothing
        """
        self.draw.arc([x1, y1, x2, y2], start_ang, end_ang, fill=fill_in)
        self.disp.image(self.screen_image)
        self.disp.display()

    def draw_chord(self, x1, y1, x2, y2, start_ang, end_ang, fill_in=1):
        """
        draws an chord within boundaries of a rectangle with
        top left corner on x1,y1, and bottom right corner on x2,y2
        with an arc that starts at angle start_ang and ends at end_ang
        added in 1.4
        :param x1:
        :param y1:
        :param x2:
        :param y2:
        :param start_ang:
        :param end_ang:
        :param fill_in:
        :return:
        """
        self.draw.chord([x1, y1, x2, y2], start_ang, end_ang, fill=fill_in)
        self.disp.image(self.screen_image)
        self.disp.display()

    def draw_ellipse(self, x, y, width, height, fill_in=0):
        """
        draws an ellipse that fits in a rectangle
        with the top left point being x and y
        and a width and height from that point
        added in 1.4
        :param x:
        :param y:
        :param width:
        :param height:
        :param fill_in:
        :return:
        """
        # outline Color to use for the outline.
        # fill Color to use for the fill.
        self.draw.ellipse((x, y, x + width, y + height), outline=1, fill=fill_in)
        self.disp.image(self.screen_image)
        self.disp.display()

    def draw_circle(self, x, y, diameter, fill_in=0):
        """
                diameter
        (x,y) |-----------|
              |   . . .   |
              | .       . |
              |.         .|
              | .       . |
              |   . . .   |
              -------------
        draws a circle that fits inside a square
        that starts has a top left corner at x,y and
        width and height of that circles diameter
        added in 1.4
        :param x: x coordinate top left corner
        :param y: y coordinate top left corner
        :param diameter: diameter of square that bounds the circle
        :param fill_in: fills in circle if true or 1
        :return: nothing
        """
        # outline Color to use for the outline white or black.
        # fill Color to use for the fill only white or black.
        self.draw.ellipse((x, y, x + diameter, y + diameter), outline=1, fill=fill_in)
        self.disp.image(self.screen_image)
        self.disp.display()

    def print(self, *messages, x=0, y=0, fill_in=1, font_size=12):
        """
        prints on the screen
        :param messages: multiple inputs that will
        turned into strings to display on the screen
        :param x: x coordinate
        :param y: y coordinate
        :param fill_in: fills in text as black=0 or white=1
        :param font_size: the pixel size of the font
        :return: nothing
        """
        # outline Color to use for the outline white or black.
        # fill Color to use for the fill only white or black.
        full_message = ""
        for message in messages:
            full_message = full_message + str(message)

        font = ImageFont.truetype(self.TEXT_FILE_PATH, font_size)
        self.draw.text((x, y), full_message, font=font, fill=fill_in)
        self.disp.image(self.screen_image)
        self.disp.display()

    def draw_graph(self, x, y, y_offset=0, x_offset=0, scale=1, draw_axes=True):
        """
        draws a graph on the OLED screen with 2 lines for the x and y axis
        :param x: x center of axis
        :param y: y center of axis
        :param y_offset: up and down offset of center of axis
        :param x_offset: left and right offset of center of axis
        :param scale: scales the graph output
        :param draw_axes: draws the lines that form the graph axis
        :return: nothing
        """
        y_center = self.height / 2 - y_offset
        x_center = self.width / 2 + x_offset
        x_new = int((x_center + x) * scale)
        y_new = int((y_center - y) * scale)  # because its inverted on screen y positive goes down the axes
        self.draw.point((x_new, y_new), fill=1)
        if draw_axes:
            # x axis line left to right
            self.draw.line(((0, y_center), (self.width, y_center)), width=1, fill=1)
            # y axis line top and down
            self.draw.line(((x_center, 0), (x_center, self.height)), width=1, fill=1)

        self.disp.image(self.screen_image)
        self.disp.display()

    # Uses the
    # added in 1.3
    def loop_text(self, direction, string='', line=25, font_size=16):

        length = len(string)
        image = Image.new('1', (self.width, self.height))
        font = ImageFont.truetype(self.TEXT_FILE_PATH, font_size)
        draw = ImageDraw.Draw(image)
        size = draw.textsize(string, font=font)

        if 0 <= line <= 45:

            if size[0] > 128:
                print("The string entered is too long.")
            else:
                if direction == 'R':
                    self.stop_scroll()
                    draw.text((1, line), string, font=font, fill=255)
                    self.disp.image(image)
                    self.disp.display()
                    self.right_scroll()

                elif direction == 'L':
                    self.stop_scroll()
                    draw.text((1, line), string, font=font, fill=255)
                    self.disp.image(image)
                    self.disp.display()
                    self.left_scroll()

                elif direction == 'S':
                    self.stop_scroll()
        else:
            print('Lines can be entered from 0 to 45.')

    def clock(self, hour, minute, string='', font_size=18):
        """
        draws a clock on the screen, redraws on whatever image was on before
        this method needs to be given the current hour, minute and a string
        that will be displayed below the time set
        the font size can be modified, default is 18
        added in 1.3
        in 1.41 fixed the font size parameter to 18 and modifies time text
        :param hour: the hour
        :param minute: the minute
        :param string: a short message 8 char, to display
        :param font_size: the font size of the time
        :return: nothing
        """
        # this method will wipe the image canvas and draw a clock
        image = Image.new('1', (self.width, self.height))
        font = ImageFont.truetype(self.TEXT_FILE_PATH, font_size)
        draw = ImageDraw.Draw(image)
        size = draw.textsize(string, font=font)
        width = self.width
        height = self.height

        # Clock
        draw.rectangle((0, 0, width, height), outline=0, fill=0)

        # Draw an ellipse.
        draw.ellipse((1, 1, 64, 64), outline=255, fill=0)
        draw.ellipse((6, 6, 58, 58), outline=255, fill=0)
        draw.ellipse((31, 31, 33, 33), outline=255, fill=0)

        timer_x = 32
        timer_y = height / 2

        # minute    
        basic_min_hand = 19
        amp_min_hand = 0.1
        min_hand = None
        hour_hand = None

        if minute <= 15:
            min_hand = basic_min_hand + (minute * amp_min_hand)
        elif minute <= 30:
            min_hand = basic_min_hand + ((15 - (minute - 15)) * amp_min_hand)
        elif minute <= 45:
            min_hand = basic_min_hand + ((minute - 30) * amp_min_hand)
        elif minute <= 60:
            min_hand = basic_min_hand + ((15 - (minute - 45)) * amp_min_hand)

        cla_minute = minute * 6 + 270
        if cla_minute > 360:
            cla_minute = cla_minute - 360

        rad_min = 3.14159 / 180 * cla_minute

        x = min_hand * math.cos(rad_min) + timer_x
        y = min_hand * math.sin(rad_min) + timer_y
        draw.line((timer_x, timer_y, x, y), fill=255)

        # hour  
        ampm_hour = hour

        if hour > 12:
            hour = hour - 12

        basic_hour_hand = 15
        amp_hour_hand = 0.1

        if hour <= 3:
            hour_hand = basic_hour_hand + (hour * amp_hour_hand)
        elif hour <= 6:
            hour_hand = basic_hour_hand + ((3 - (hour - 3)) * amp_hour_hand)
        elif hour <= 9:
            hour_hand = basic_hour_hand + ((hour - 6) * amp_hour_hand)
        elif hour <= 12:
            hour_hand = basic_hour_hand + ((3 - (hour - 9)) * amp_hour_hand)

        cal_hour = (hour * 30 + 270)  # + (6*claMinute/60)
        if cal_hour > 360:
            cal_hour = cal_hour - 360

        rad_min = 3.14159 / 180 * cal_hour

        x = hour_hand * math.cos(rad_min) + timer_x
        y = hour_hand * math.sin(rad_min) + timer_y

        draw.line((timer_x, timer_y, x, y), fill=255)

        font = ImageFont.truetype(self.TEXT_FILE_PATH, font_size)
        x_pos = 67
        y_pos = 18
        if ampm_hour >= 10:
            if minute >= 10:
                draw.text((x_pos, y_pos), str(ampm_hour) + ' : ' + str(minute), font=font, fill=255)
            else:
                draw.text((x_pos, y_pos), str(ampm_hour) + ' : 0' + str(minute), font=font, fill=255)
        else:
            if minute >= 10:
                draw.text((x_pos, y_pos), '  ' + str(ampm_hour) + ' : ' + str(minute), font=font, fill=255)
            else:
                draw.text((x_pos, y_pos), '  ' + str(ampm_hour) + ' : 0' + str(minute), font=font, fill=255)

        length = len(string)
        if length > 8:
            print('Please enter 8 characters or fewer than 8 characters.')
        else:
            font = ImageFont.truetype(self.TEXT_FILE_PATH, 14)
            draw.text((69, 18 + 25), string, font=font, fill=255)

        # Display image.
        self.disp.image(image)
        self.disp.display()

    # turns off the display does not clear the image however
    # in order to make OLED display again, need to call screen.on()
    def off(self):
        # self.SSD1306_DISPLAYOFF = 0xAE
        self.disp.command(self.SSD1306_DISPLAYOFF)

    # turns on the display useful after using screen.off()
    def on(self):
        # self.SSD1306_DISPLAYALLON = 0xA5
        self.disp.command(self.SSD1306_DISPLAYON)

    # makes all pixels invert in color, so black
    # become white and white becomes black
    def invert(self):
        # self.SSD1306_INVERTDISPLAY = 0xA7
        self.disp.command(self.SSD1306_INVERTDISPLAY)

    def normal(self):
        # self.SSD1306_NORMALDISPLAY = 0xA6
        self.disp.command(self.SSD1306_NORMALDISPLAY)

    def stop_scroll(self):
        # self.SSD1306_DEACTIVATE_SCROLL = 0x2E
        self.disp.command(self.SSD1306_DEACTIVATE_SCROLL)

    def start_scroll(self):
        # self.SSD1306_ACTIVATE_SCROLL = 0x2F
        self.disp.command(self.SSD1306_ACTIVATE_SCROLL)

    def right_scroll(self, start_row=0, end_row=7, time_interval=7):
        self.stop_scroll()
        self.disp.command(self.SSD1306_RIGHT_HORIZONTAL_SCROLL)
        self.disp.command(0x00)  # dummy byte
        self.disp.command(start_row)  # define page 0 as start page address
        self.disp.command(time_interval)  # set time interval between each scroll step as 5 frames 
        self.disp.command(end_row)  # define page 7 as end page address
        self.disp.command(0x00)  # dummy byte
        self.disp.command(0xFF)  # dummy byte
        self.start_scroll()

    def left_scroll(self, start_row=0, end_row=7, time_interval=7):
        # row is a number from 0-7 it is also the "pages"
        # col is the column in pixels of the area that will move (1-128)
        # time interval can be selected from the following 8 states
        # 0 = 5 frames, 1= 64 frames, 2 = 128 frames, 3 = 256 frames
        # 4 = 3 frames, 5 = 4 frames, 6= 25 frames, 7 = 2 frames
        self.stop_scroll()
        self.disp.command(self.SSD1306_LEFT_HORIZONTAL_SCROLL)
        self.disp.command(0x00)  # A dummy byte
        self.disp.command(start_row)  # B define page 0 as start page address
        self.disp.command(time_interval)  # C set time interval between each scroll step as 5 frames 
        self.disp.command(end_row)  # D define page 15 as end page address
        self.disp.command(0x00)  # E dummy byte
        self.disp.command(0xFF)  # F dummy byte
        self.start_scroll()

    def up_and_left_scroll(self, start_row=0, end_row=7, vert_offset=1, time_interval=7):
        # row is a number from 0-7 it is also the "pages"
        # time interval can be selected from the following 8 states
        # 0 = 5 frames, 1= 64 frames, 2 = 128 frames, 3 = 256 frames
        # 4 = 3 frames, 5 = 4 frames, 6= 25 frames, 7 = 2 frames
        self.stop_scroll()
        self.disp.command(self.SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL)  # SSD1306_LEFT_HORIZONTAL_SCROLL
        self.disp.command(0x00)  # A dummy byte
        self.disp.command(start_row)  # B define page 0 as start page address
        self.disp.command(time_interval)  # C set time interval between each scroll step as 5 frames
        self.disp.command(end_row)  # D define page 7 as end page address
        self.disp.command(vert_offset)  # E vertical scrolling effect ex: 1 = 1 row offset
        self.start_scroll()

    def up_and_right_scroll(self, start_row=0, end_row=7, vert_offset=1, time_interval=7):
        self.stop_scroll()
        self.disp.command(self.SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL)
        self.disp.command(0x00)  # dummy byte
        self.disp.command(start_row)  # define page 0 as start page address
        self.disp.command(time_interval)  # set time interval between each scroll step as 5 frames 
        self.disp.command(end_row)  # define page 7 as end page address
        self.disp.command(vert_offset)
        self.start_scroll()

    def set_contrast(self, value):
        self.disp.set_contrast(value)

    # added in 1.3 #TODO protection in case student stops program
    # and the oled stays off
    def flicker_text(self, string, count=10, delay=0.5):
        if len(string) > 8:
            print('Please enter 8 characters or fewer than 8 characters.')
        else:
            message = string

            self.clear_display()
            self.draw_text_center(message)
            for i in range(0, count):
                # self.clear_display()
                # self.draw_text_center(message)
                self.on()
                time.sleep(delay + 0.01)
                self.off()
                time.sleep(delay + 0.01)
            # make sure to turn display back on
            self.on()

            # added in 1.3

    def size_text(self, count, size, string):
        if len(string) > 8:
            print('Please enter 8 characters or fewer than 8 characters.')
        else:
            message = string
            self.draw_text_center(message, font_size=16)

            for i in range(0, count):

                for j in range(16, size, 2):
                    self.draw_text_center(message, font_size=j)

                for j in range(size, 16, -2):
                    self.draw_text_center(message, font_size=j)

    # added in 1.3
    def moving_text(self, direction, string, line=25, speed=5, font_size=16):

        image = Image.new('1', (self.width, self.height))
        font = ImageFont.truetype(self.TEXT_FILE_PATH, font_size)
        draw = ImageDraw.Draw(image)

        size = draw.textsize(string, font=font)
        pos = 128 - size[0]
        length = len(string)

        if 0 <= line <= 45:

            if length > 8:
                print('Please enter 8 characters or fewer than 8 characters.')
            else:
                if direction == 'R':
                    for i in range(0, pos, speed):
                        image = Image.new('1', (self.width, self.height))
                        draw = ImageDraw.Draw(image)
                        self.disp.display()
                        draw.text((0 + i, line), string, font=font, fill=255)
                        self.disp.image(image)
                        self.disp.display()

                if direction == 'L':
                    for i in range(pos, 0, -speed):
                        image = Image.new('1', (self.width, self.height))
                        draw = ImageDraw.Draw(image)
                        self.disp.display()
                        draw.text((0 + i, line), string, font=font, fill=255)
                        self.disp.image(image)
                        self.disp.display()
        else:
            print('Lines can be entered from 0 to 45.')

    def clear_display(self):
        self.disp.clear()
        self.disp.display()

    def clear_drawing(self):
        self.disp.clear()
        # self.disp.display()
        # reset the screen image
        self.screen_image = Image.new('1', (self.width, self.height))
        # reset the drawing as well using a new black image
        self.draw = ImageDraw.Draw(self.screen_image)

    def draw_text(self, string, x=1, y=1, display=0, image=0, font_size=16, clear=True):
        # if display does not exist
        if display == 0:
            # grab display object
            # and give it a local name
            display = self.disp
        if image == 0:
            # if no image object, make one
            image = Image.new('1', (self.width, self.height))
        font = ImageFont.truetype(self.TEXT_FILE_PATH, font_size)
        # this will make a new blank draw object
        draw = ImageDraw.Draw(image)
        max_x = 0
        max_y = 0
        # TODO: This doesnt actually do anything ---------
        if clear:
            # draw a black rectangle to clear out the previous draw image
            draw.rectangle((0, 0, self.width, self.height), outline=0, fill=0)
        current_x = x
        current_y = y
        # ------------------------------------------------

        for char in string:
            char_width, char_height = draw.textsize(char, font=font)
            max_x = char_width if max_x < char_width else max_x
            max_y = char_height if max_y < char_height else max_y
            draw.text((current_x, current_y), char, font=font, fill=255)
            current_x += char_width
            if current_x > self.width - max_x:
                current_x = x
                current_y += max_y + 1
        display.image(image)
        display.display()

        self.screen_image = image

    def draw_text_center(self, string, display=0, image=0, font_size=16, clear=True):

        words = string.split(' ')
        split_lines = []
        text = ""
        current_h = font_size
        font = ImageFont.truetype(self.TEXT_FILE_PATH, font_size)

        if display == 0:
            display = self.disp

        if image == 0:
            image = Image.new('1', (self.width, self.height))

        draw = ImageDraw.Draw(image)

        # TODO: FIX this clear does nothing as well,
        # rectangle has 4 points parameters
        if clear:
            draw.rectangle((0, 0, self.width, self.height), outline=0, fill=0)

        for word in words:
            new_line = False
            new_line_next_word = ""

            if "\n" in word:
                try:
                    word, new_line_next_word = word.split("\n")
                    new_line = True
                except:
                    print("You should use '\\n' only once in one word")
                    return

            text += word + " "
            text_width, text_height = draw.textsize(text, font=font)

            if word == words[0]:
                current_h = text_height

            if text_width >= 124:
                text = text[:-len(word) - 2]
                split_lines.append(text)
                text = word + " "
                current_h += text_height

            if new_line:
                split_lines.append(text[:-1])
                current_h += text_height
                text = new_line_next_word + " "

            if current_h >= 60:
                print("Sentence is too long")
                return

        split_lines.append(text[:-1])

        current_y = (self.height - 4 - current_h) / 2

        for text in split_lines:
            text_width, text_height = draw.textsize(text, font=font)
            current_x = (self.width - text_width) / 2
            draw.text((current_x, current_y), text, font=font, fill=255)
            current_y += text_height

        display.image(image)
        display.display()

        self.screen_image = image

    def path_to_image(self, path):
        # convert a file path into an PIL image object if it exists
        return Image.open(path).convert('1')

    def show_image(self, image):
        # this method will take a photo taken with pi camera and display it
        import cv2
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  # Convert it to gray
        small = cv2.resize(gray, (128, 64))  # Resize it to fit the screen
        self.draw_image(Image.fromarray(small).convert('1'))  # show the picture!

    def draw_image(self, img, display=0):
        if display == 0:
            display = self.disp
        display.image(img)
        display.display()

    def draw_image_by_path(self, path):
        """
        Will draw an image with a given path
        will take png or ppm files, will resize
        if necessary to fit in the
        128 by 64 pixel screen
        """
        try:
            img = self.path_to_image(path)
            self.draw_image(img)
        except ValueError:
            # if the image is not correct
            # dimensions change it
            img = self.path_to_image(path)
            im = img.resize((128, 64))
            self.draw_image(im)

    def draw_image_by_name(self, name):
        self.draw_image(self.path_to_image(self.EYE_IMAGE_FOLDER_PATH + name + ".ppm"))

    def animate(self, preset=None, custom=False):
        preset = self.EXCITED if preset is None else preset
        if not custom:
            for item in preset:
                self.draw_image(self.path_to_image(self.EYE_IMAGE_FOLDER_PATH + item + ".ppm"))
        else:
            for item in preset:
                self.draw_image(item)

    def calibrating(self):
        self.draw_image(self.path_to_image(self.EYE_IMAGE_FOLDER_PATH + "calibrating.ppm"))

    def calibrated(self):
        self.draw_image(self.path_to_image(self.EYE_IMAGE_FOLDER_PATH + "calibrated.ppm"))

    def close_eyes(self):
        self.draw_image(self.path_to_image(self.EYE_IMAGE_FOLDER_PATH + "close.ppm"))

    def sleepy_eyes(self):
        self.draw_image(self.path_to_image(self.EYE_IMAGE_FOLDER_PATH + "sleep.ppm"))

    def sleepy_left(self):
        self.draw_image(self.path_to_image(self.EYE_IMAGE_FOLDER_PATH + "sleepyleft1.ppm"))

    def sleepy_right(self):
        self.draw_image(self.path_to_image(self.EYE_IMAGE_FOLDER_PATH + "sleepyright1.ppm"))

    def blink(self):
        self.draw_image(self.path_to_image(self.EYE_IMAGE_FOLDER_PATH + "neutral2.ppm"))
        time.sleep(.25)
        self.draw_image(self.path_to_image(self.EYE_IMAGE_FOLDER_PATH + "close.ppm"))
        time.sleep(.25)
        self.draw_image(self.path_to_image(self.EYE_IMAGE_FOLDER_PATH + "neutral1.ppm"))

    def look_around_open(self):
        self.draw_image_by_name("lookright1")
        time.sleep(2)
        self.close_eyes()
        self.draw_image_by_name("lookleft1")
        time.sleep(1)
        self.close_eyes()
        self.draw_image_by_name("lookright1")
        time.sleep(1)
        self.close_eyes()
        self.hello()
        time.sleep(1)

    def sleeping(self):
        self.draw_image_by_name("close")
        time.sleep(.6)
        self.draw_image_by_name("sleep_z1")
        time.sleep(.6)
        self.draw_image_by_name("sleep_z2")
        time.sleep(.6)
        self.draw_image_by_name("sleep_z3")
        time.sleep(.6)
        self.draw_image_by_name("close")
        time.sleep(.6)

    def look_around(self):
        self.sleepy_eyes()
        time.sleep(2)
        self.close_eyes()
        self.sleepy_left()
        time.sleep(1)
        self.close_eyes()
        self.sleepy_right()
        time.sleep(1)
        self.close_eyes()
        self.sleepy_eyes()
        time.sleep(1)

    def glimmer(self):
        glimmer = ["neutral1", "neutral2", "neutral3"]
        self.animate(glimmer)

    def sad(self):
        sad = ["sad1"]
        self.animate(sad)

    def happy(self):
        happy = ["neutral1", "neutral2"]
        wink = ["happy_left2", "happy_right1"]
        self.animate(happy)
        for i in range(3):
            self.animate(wink)
        self.hello()

    def hello(self):
        self.draw_image(self.path_to_image(self.EYE_IMAGE_FOLDER_PATH + "neutral1.ppm"))

    def angry(self):
        self.draw_image(self.path_to_image(self.EYE_IMAGE_FOLDER_PATH + "focus.ppm"))

    def connection_success(self):
        self.draw_image_by_name("connected")

    def connection_fail(self):
        self.draw_image_by_name("onlinefail")


def run():
    print("test screen.py script")
    eye = Screen()
    print(eye.TEXT_FILE_PATH)
    print(os.path.dirname(os.path.abspath(__file__)))
    eye.draw_text("hello world")
    time.sleep(2)
    eye.close_eyes()
    time.sleep(2)
    eye.blink()
    time.sleep(2)
    eye.glimmer()
    time.sleep(2)
    eye.sad()
    time.sleep(2)
    eye.happy()
    time.sleep(2)
    eye.hello()
    print("end test screen.py")


if __name__ == '__main__':
    run()
