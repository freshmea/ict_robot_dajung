import pygame
from pg_local import *

class Game:
    def __init__(self) -> None:
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_X, SCREEN_Y))
        self.clock = pygame.time.Clock()
        self.playing = True

    def run(self):
        while self.playing:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.playing = False
            pygame.display.update()

    def quit(self):
        pygame.quit()