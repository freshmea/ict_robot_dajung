import pygame
from pg_local import *
vec = pygame.math.Vector2

class Bullet(pygame.sprite.Sprite):
    def __init__(self, root, vel):
        super().__init__()
        self.game = root
        self.image = pygame.Surface((25,25))
        self.rect = self.image.get_rect()
        self.image.fill((255,0,0))
        self.pos = vec(0,SCREEN_Y)
        self.rect.center = self.pos
        self.vel = vel*100
        self.gravity = 0.05
    
    def update(self):
        self.vel.y += self.gravity
        self.pos += self.vel
        self.rect.center = self.pos