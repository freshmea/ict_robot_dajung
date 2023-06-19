import pygame
from pg_local import *
import random
from pg_cloud import Cloud

class Saram(pygame.sprite.Sprite):
    def __init__(self, x, y, root) -> None:
        self.x = x
        self.y = y
        self.game = root
        self.color = pygame.Color('black')
        self.image = pygame.Surface((200, 50))
        self.image.fill(self.color)
        self.rect = self.image.get_rect()
        self.rect.topleft = (self.x, self.y)
        self.groups = self.game.saram_sprites
        pygame.sprite.Sprite.__init__(self, self.groups)

class Game:
    def __init__(self) -> None:
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_X, SCREEN_Y))
        self.clock = pygame.time.Clock()
        self.playing = True
        self.load_data()
        self.all_sprites = pygame.sprite.Group()
        self.saram_sprites = pygame.sprite.Group()
        self.saram_sprites.add(Saram(300,SCREEN_Y-100, self))
        self.cloud_count = 0
        
    
    def load_data(self):
        self.image = pygame.image.load('image/background01.png').convert_alpha()
        self.image = pygame.transform.scale(self.image, (SCREEN_X, SCREEN_Y))

    def run(self):
        while self.playing:
            self.clock.tick(FPS)
            self.event()
            self.update()
            self.draw()

    def event(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.playing = False
        if self.cloud_count < CLOUD_NUMBER:
            self.all_sprites.add(Cloud(random.randint(0, SCREEN_X), self))
            self.cloud_count += 1
            
    
    def update(self):
        self.all_sprites.update()
        self.saram_sprites.update()
            
    def draw(self):
        self.screen.fill((255,255,255))
        self.screen.blit(self.image, (0,0))
        self.all_sprites.draw(self.screen)
        self.saram_sprites.draw(self.screen)
        pygame.display.update()
        
    def quit(self):
        pygame.quit()