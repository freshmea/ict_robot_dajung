import pygame
from pg_local import *
import random
from pg_rain import Rain


class Cloud(pygame.sprite.Sprite):
    def __init__(self, x, root) -> None:
        self.x = x
        self.y = 30
        self.game = root
        self.load_data()
        self.rect = self.image.get_rect()
        self.groups = [self.game.all_sprites, self.game.cloud_sprites]
        pygame.sprite.Sprite.__init__(self, self.groups)
        self.rect.topleft = (self.x, self.y)
        self.dir = 1
        self.speed = random.randint(2, 5)

    def load_data(self):
        self.image = pygame.image.load('image/cloud.png')
        self.image = pygame.transform.scale(self.image, (200, 100))

    def rain(self):
        # 비 객체 생성.
        for _ in range(QUENTITY):
            self.game.all_sprites.add(Rain(random.randint(self.x, self.x+200), 120, self.game))

    def update(self):
        self.rain()
        self.x += self.dir * self.speed
        # 화면 밖으로 나가지 않게 바운더리 적용.
        if self.x > SCREEN_X:
            self.dir = -1
        if self.x < 0:
            self.dir = 1
        self.rect.topleft = (self.x, self.y)
        
    def is_click(self):
        #마우스 클릭시 collision bool 반환
        pos = pygame.mouse.get_pos()
        return self.rect.collidepoint(pos)