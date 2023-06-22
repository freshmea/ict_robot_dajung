import pygame, time
from pg_local import *
from pg_bullet import Bullet
vec = pygame.math.Vector2

class Game:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_X, SCREEN_Y))
        self.clock = pygame.time.Clock()
        self.playing = True
        self.all_sprite = pygame.sprite.Group()
        self.check = False
        self.bul = False
        self.time = time.time()

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
            if event.type == pygame.MOUSEBUTTONDOWN:
                self.check = True
            if event.type == pygame.MOUSEBUTTONUP:
                self.check = False

    def update(self):
        self.all_sprite.update()
        if self.check:
            self.time = time.time()
            self.bul = True
        elif self.bul:
            print('make bullet')
            dir = (vec(pygame.mouse.get_pos()) - vec(0, SCREEN_Y)).normalize()
            print(dir)
            magnitute = (time.time()-self.time)*5
            self.all_sprite.add(Bullet(self, dir * magnitute))
            self.bul = False
    
    def draw(self):
        self.screen.fill((255,255,255))
        self.all_sprite.draw(self.screen)
        pygame.display.update()
        
    def quit(self):
        pygame.quit()