import pygame
import random
from pg_local import *

class Rain:
    def __init__(self, x, y, root) -> None:
        self.x = x
        self.y = y
        self.game = root
        self.color = pygame.Color('blue')
        self.bold = 3
        self.speed = random.randint(3, 10)
        self.wind = 9

    def update(self):
        self.y += self.speed
        self.x += self.wind/10
        if self.y > SCREEN_Y:
            for i, rain in enumerate(self.game.rains):
                if self == rain:
                    del self.game.rains[i]
        del self

    def draw(self):
        pygame.draw.line(self.game.screen, self.color, (self.x, self.y), (self.x, self.y+10), self.bold)
