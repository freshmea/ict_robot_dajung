import pygame
from pg_local import *

class Cloud:
    def __init__(self, x, y, root) -> None:
        self.x = x
        self.y = y
        self.game = root
        self.load_data()
        self.dir = 1

    def load_data(self):
        self.image = pygame.image.load('image/cloud.png')
        self.image = pygame.transform.scale(self.image, (200, 100))

    def update(self):
        self.x += self.dir
        if self.x > SCREEN_X:
            self.dir = -1
        if self.x < 0:
            self.dir = 1

    def draw(self):
        self.game.screen.blit(self.image, (self.x, self.y))