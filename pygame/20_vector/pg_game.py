import pygame, random
from pg_local import *

class Rectagle(pygame.sprite.Sprite):
    def __init__(self, x, y, root):
        super().__init__()
        self.game = root
        self.image = pygame.Surface((30, 30))
        self.image.fill(pygame.Color('crimson'))
        self.rect = self.image.get_rect()
        self.rect.center = x, y

    def update(self):
        pass
    
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
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_q:
                        self.playing = False
        if len(self.all_sprites)< 20:
            self.all_sprites.add(Rectagle(random.randint(0, SCREEN_X), random.randint(0, SCREEN_Y), self))
    def update(self):
        pass

    def draw(self):
        self.all_sprites.draw(self.screen)
        pygame.display.update()
    def quit(self):
        pygame.quit()