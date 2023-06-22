import pygame, random
from pg_local import *

class Pizza(pygame.sprite.Sprite):
    def __init__(self, root):
        super().__init__()
        self.game = root
        self.image = pygame.Surface((GRID,GRID))
        self.rect = self.image.get_rect()
        self.image.fill(pygame.Color('red'))
        self.rect.topleft = (random.randint(0,GRID_WIDTH-1)*GRID ,random.randint(0,GRID_HEIGHT-1)*GRID+SPAN)
