import Adafruit_SSD1306
import time
import os
from PIL import Image, ImageFont, ImageDraw
import random
from random import shuffle, randrange


class Maze:
    # Creates a pixel art map on the OLED screen
    # creates 13 by 9 Blocks, useful for creating maze games
    # on the Zumi OLED screen
    # 맵을 입력하지 않는 경우 사용하는 기본 맵
    basicMap = [
        [1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1],
        [1, 0, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1],
        [1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1],
        [1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1],
        [1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1],
        [1, 0, 0, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1]
    ]

    def __init__(self, screen_obj):

        self.disp = screen_obj.disp
        self.width = self.disp.width
        self.height = self.disp.height

        self.image = Image.new('1', (self.width, self.height))
        self.draw = ImageDraw.Draw(self.image)

        self.size = 4
        self.spacingX = 10
        self.spacingY = 7

        self.oldCharX = 255
        self.oldCharY = 255

    def map_generator(self, map=basicMap):

        self.oldCharX = 255
        self.oldCharY = 255

        count_x = 0
        count_y = 0

        for j in range(0, 60, self.spacingY):
            for i in range(1, 130, self.spacingX):
                if map[count_y][count_x] == 1:
                    self.draw.rectangle((i - 1, j, i + self.size, j + self.size), outline=1, fill=map[count_y][count_x])
                else:
                    self.draw.rectangle((i - 1, j, i + self.size, j + self.size), outline=0, fill=map[count_y][count_x])
                count_x = count_x + 1
            count_x = 0
            count_y = count_y + 1

        self.disp.image(self.image)
        self.disp.display()

    def move_xy(self, posX=0, posY=0):

        now_x = (posX * (self.spacingX)) + 1
        now_y = posY * (self.spacingY)

        self.draw.rectangle((self.oldCharX - 1, self.oldCharY, self.oldCharX + self.size, self.oldCharY + self.size),
                            outline=0, fill=0)
        self.draw.ellipse((now_x - 1, now_y, now_x + self.size, now_y + self.size), outline=1, fill=0)

        self.disp.image(self.image)
        self.disp.display()

        self.oldCharX = now_x
        self.oldCharY = now_y

    def random_map(self, w=6, h=4):
        vis = [[0] * w + [1] for _ in range(h)] + [[1] * (w + 1)]
        ver = [["|  "] * w + ['|'] for _ in range(h)] + [[]]
        hor = [["+--"] * w + ['+'] for _ in range(h + 1)]

        def walk(x, y):
            vis[y][x] = 1

            d = [(x - 1, y), (x, y + 1), (x + 1, y), (x, y - 1)]
            shuffle(d)
            for (xx, yy) in d:
                if vis[yy][xx]: continue
                if xx == x: hor[max(y, yy)][x] = "z  "
                if yy == y: ver[y][max(x, xx)] = "xxx"
                walk(xx, yy)

        walk(randrange(w), randrange(h))
        map_data = ""
        for (a, b) in zip(hor, ver):
            map_data += ''.join(a + ['\n'] + b + ['\n'])

        list(map_data)
        map_len = len(map_data)

        string_map = ""
        mapText = []
        halfWall = False
        halfRoom = 0

        for i in range(0, map_len):

            if map_data[i] == "+":
                mapText.append(1)

            if map_data[i] == "\n":
                mapText.append("a")

            if map_data[i] == "|":
                if map_data[i + 1] == "\n":
                    mapText.append(1)
                else:
                    mapText.append(1)
                    mapText.append(0)

            if map_data[i] == "-":

                if halfWall == False:
                    mapText.append(1)
                    halfWall = True
                else:
                    halfWall = False

                # print(str(i) + "+")
            if map_data[i] == "x":

                if halfRoom == 0:
                    mapText.append(0)
                    mapText.append(0)
                    halfRoom = 1
                elif halfRoom == 1:
                    halfRoom = 2

                elif halfRoom == 2:
                    halfRoom = 0

            if map_data[i] == "z":
                mapText.append(1)
                mapText.append(0)

        makeMap = [
            [1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
            [1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1],
            [1, 0, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1],
            [1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1],
            [1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1],
            [1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1],
            [1, 0, 0, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1],
            [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1]
        ]
        nowNum = 0
        nowLine = 0
        for i in mapText:
            if i == 'a':
                nowLine = nowLine + 1
                nowNum = 0
            else:
                makeMap[nowLine][nowNum] = i
                nowNum = nowNum + 1

        maze_entranceY = 0
        maze_entranceX = 0
        while True:
            maze_entranceX = random.choice([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            if makeMap[maze_entranceY + 1][maze_entranceX] == 0:
                break  # not wall

        maze_exitY = 8
        maze_exitX = 0
        while True:
            maze_exitX = random.choice([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
            if makeMap[maze_exitY - 1][maze_exitX] == 0:
                break  # not wall

        makeMap[maze_entranceY][maze_entranceX] = 0
        makeMap[maze_exitY][maze_exitX] = 0

        return makeMap
