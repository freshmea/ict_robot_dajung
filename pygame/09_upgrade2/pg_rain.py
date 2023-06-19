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
        self.mask = pygame.mask.from_surface(self.image)

    def update(self):
        self.y += self.speed + self.y/SCREEN_Y * 20
        self.x += self.wind/10
        self.rect.topleft = (self.x, self.y)
        if self.y > SCREEN_Y:
            self.game.all_sprites.remove(self)
        if pygame.sprite.collide_mask(self, self.game.player_sprites.sprites()[0]):
            self.game.hit_rain += 1
            print(self.game.hit_rain)
            self.game.all_sprites.remove(self)
            del self
