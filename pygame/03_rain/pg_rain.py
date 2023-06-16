import pygame
from pg_local import *

class Rain:
    def __init__(self, x, y, root) -> None:
        self.x = x
        self.y = y
        self.game = root
        self.color = pygame.Color('blue')
        self.bold = 3
    
    def update(self):
        self.y += 1
    
    def draw(self):
        pygame.draw.line(self.game.screen, self.color, (self.x, self.y), (self.x, self.y+10), self.bold)