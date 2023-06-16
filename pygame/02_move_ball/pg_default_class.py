import pygame
from pg_local import *

class Game:
    def __init__(self) -> None:
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_X, SCREEN_Y))
        self.clock = pygame.time.Clock()
        self.playing = True
        self.color = pygame.Color('blue')
        self.position_x = 0

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
        self.position_x += 1
        if (self.position_x > SCREEN_X):
            self.position_x = 0
    
    def draw(self):
        self.screen.fill((255,255,255))
        pygame.draw.circle(self.screen,self.color, (self.position_x, SCREEN_Y/2), 100)
        pygame.display.update()

    def quit(self):
        pygame.quit()