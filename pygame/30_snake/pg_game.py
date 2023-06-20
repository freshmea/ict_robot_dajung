import pygame, sys
from pg_local import *
from pg_text import Text

class Game:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_X, SCREEN_Y))
        self.clock = pygame.time.Clock()
        self.playing = True
        self.replay = True

    def run(self):
        self.opening()
        self.playing = True
        while self.playing:
            self.clock.tick(FPS)
            self.event()
            self.update()
            self.draw()
        self.playing = True
        self.ending()

    def opening(self):
        while self.playing:
            self.clock.tick(FPS)
            self.event()
            self.screen.fill(pygame.Color('black'))
            text1 = Text('이것은 스네이크 게임 입니다.', 50, pygame.Color('white'),30, 100, self)
            self.screen.blit(text1.image, text1.rect)
            pygame.display.update()

    def ending(self):
        self.replay = False
        while self.playing:
            self.pressed_key = pygame.key.get_pressed()
            self.clock.tick(FPS)
            self.event()
            self.screen.fill(pygame.Color('black'))
            text1 = Text('이것은 엔딩 화면 입니다.', 50, pygame.Color('white'),30, 100, self)
            self.screen.blit(text1.image, text1.rect)
            pygame.display.update()
            if self.pressed_key[pygame.K_SPACE]:
                print('replay', self.replay)
                self.replay = True
                break

    def event(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.playing = False
                sys.exit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:
                    self.playing = False

    def update(self):
        pass

    def draw(self):
        self.screen.fill(pygame.Color('white'))
        self.draw_grid()
        pygame.display.update()
        
    def draw_grid(self):
        for row in range(0, int(GRID_HEIGHT)):
            for col in range(0, int(GRID_WIDTH)):
                if (row+col) % 2 == 0:
                    rect = pygame.Rect((col*GRID, row*GRID), (GRID, GRID))
                    pygame.draw.rect(self.screen, pygame.Color('pink1'), rect)
                else:
                    rect = pygame.Rect((col*GRID, row*GRID), (GRID, GRID))
                    pygame.draw.rect(self.screen, pygame.Color('pink3'), rect)

    def quit(self):
        pygame.quit()