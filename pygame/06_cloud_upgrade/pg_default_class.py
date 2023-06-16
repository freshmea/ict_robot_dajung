import pygame, random
from pg_local import *
from pg_rain import Rain
from pg_cloud import Cloud


class Game:
    def __init__(self) -> None:
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_X, SCREEN_Y))
        self.clock = pygame.time.Clock()
        self.playing = True
        self.color = pygame.Color('blue')
        self.position_x = 0
        self.rains = []
        self.clouds = []
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
        if len(self.clouds) < CLOUD_NUMBER:
            self.clouds.append(Cloud(random.randint(0, SCREEN_X), 30, self))

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