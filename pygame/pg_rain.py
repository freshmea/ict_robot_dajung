import pygame
import random
from pg_local import *

class Rain(pygame.sprite.Sprite):
    def __init__(self, x, y, root) -> None:
        self.x = x
        self.y = y
        self.game = root
        self.color = pygame.Color('red')
        self.bold = 3
        self.speed = random.randint(3, 10)
        self.wind = 9
        self.image = pygame.Surface((self.bold, 10))
        self.image.fill(self.color)
        self.rect = self.image.get_rect()
        self.rect.topleft = (self.x, self.y)
        self.groups = self.game.all_sprites
        pygame.sprite.Sprite.__init__(self, self.groups)
        

    def update(self):
        self.y += self.speed
        self.x += self.wind/10
        self.rect = (self.x, self.y)
        if self.y > SCREEN_Y:
            self.game.all_sprites.remove(self)
        if pygame.Rect((self.x, self.y),(10,3)).colliderect(self.game.saram.rect):
            self.game.all_sprites.remove(self)
