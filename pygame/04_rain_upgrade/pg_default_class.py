import pygame, random
from pg_local import *
from pg_rain import Rain

class Game:
    def __init__(self) -> None:
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_X, SCREEN_Y))
        self.clock = pygame.time.Clock()
        self.playing = True
        self.color = pygame.Color('blue')
        self.position_x = 0
        self.rains = []
        self.load_data()
        
    def load_data(self):
        self.image = pygame.image.load('image/background01.png')
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
        for _ in range(QUENTITY):
            self.rains.append(Rain(random.randint(0, SCREEN_X), 0, self))
    
    def update(self):
        for rain in self.rains:
            rain.update()
    
    def draw(self):
        self.screen.fill((255,255,255))
        self.screen.blit(self.image, (0,0))
        for rain in self.rains:
            rain.draw()
        pygame.display.update()

    def quit(self):
        pygame.quit()