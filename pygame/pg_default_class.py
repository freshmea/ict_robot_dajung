import pygame
from pg_local import *
import random
from py_rain import Rain

class Clound:
    def __init__(self, x, root) -> None:
        self.x = x
        self.y = 30
        self.game = root
        self.load_data()
        self.dir = 1
    
    def load_data(self):
        self.image = pygame.image.load('image/cloud.png')
        self.image = pygame.transform.scale(self.image, (200, 100))
        
    def update(self):
        self.x += self.dir
        if self.x > SCREEN_X:
            self.dir = -1
        if self.x < 0:
            self.dir = 1

    def draw(self):
        self.game.screen.blit(self.image, (self.x, self.y))
    
class Game:
    def __init__(self) -> None:
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_X, SCREEN_Y))
        self.clock = pygame.time.Clock()
        self.playing = True
        self.rains = []
        self.clouds = []
        self.load_data()
    
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
        for i in range(QUENTITY):
            self.rains.append(Rain(random.randint(0, SCREEN_X), 0, self))
        if len(self.clouds) < 5:
            self.clouds.append(Clound(random.randint(0, SCREEN_X), self))
    
    def update(self):
        for rain in self.rains:
            rain.update()
        for cloud in self.clouds:
            cloud.update()
            
    def draw(self):
        self.screen.fill((255,255,255))
        self.screen.blit(self.image, (0,0))
        for rain in self.rains:
            rain.draw()
        for cloud in self.clouds:
            cloud.draw()
        pygame.display.update()
        
    def quit(self):
        pygame.quit()