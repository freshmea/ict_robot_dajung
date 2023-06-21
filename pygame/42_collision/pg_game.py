import pygame, random
from pg_local import *
from pg_ball import Ball
class Game:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_X, SCREEN_Y))
        self.clock = pygame.time.Clock()
        self.playing = True
        self.all_sprites = pygame.sprite.Group()

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
        if len(self.all_sprites) < NUMBER:
            ball = Ball(self, random.randint(3,5))
            self.all_sprites.add(ball)

    def update(self):
        self.all_sprites.update()
    
    def draw(self):
        self.screen.fill((255,255,255))
        self.all_sprites.draw(self.screen)
        pygame.display.update()
        
    def quit(self):
        pygame.quit()