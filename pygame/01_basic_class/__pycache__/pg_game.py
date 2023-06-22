import pygame
from pg_local import *

class Game:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_X, SCREEN_Y))
        self.clock = pygame.time.Clock()
        self.playing = True
        self.all_sprite = pygame.sprite.Group()

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

    def update(self):
        self.all_sprite.update()
    
    def draw(self):
        self.all_sprite.draw(self.screen)
        pygame.display.update()
        
    def quit(self):
        pygame.quit()