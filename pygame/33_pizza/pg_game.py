import pygame, sys, random
from pg_local import *
from pg_text import Text
from pg_snake import Snake
from pg_pizza import Pizza
vec = pygame.math.Vector2


class Game:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_X, SCREEN_Y))
        self.clock = pygame.time.Clock()
        self.playing = True
        self.replay = True
        self.color_check = True

    def run(self):
        self.opening()
        self.playing = True
        while self.playing:
            self.clock.tick(FPS + self.snake_sprites.sprites()[0].length/2)
            self.update()
            self.event()
            self.make_snake()
            self.make_pizza()
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
            text2 = Text('q를 누르면 게임이 시작 됩니다.', 50, pygame.Color('white'),30, 170, self)
            self.screen.blit(text2.image, text2.rect)
            pygame.display.update()
        self.score = 0
        self.all_sprites = pygame.sprite.Group()
        self.snake_sprites = pygame.sprite.Group()
        self.pizza_sprites = pygame.sprite.Group()
        snake = Snake(self, True)
        self.all_sprites.add(snake)
        self.snake_sprites.add(snake)
        self.t_rect = [pygame.Rect(GRID,GRID,GRID,GRID)]

    def ending(self):
        self.replay = False
        while self.playing:
            self.pressed_key = pygame.key.get_pressed()
            self.clock.tick(FPS)
            self.event()
            self.screen.fill(pygame.Color('black'))
            text1 = Text('이것은 엔딩 화면 입니다.', 50, pygame.Color('white'),30, 100, self)
            self.screen.blit(text1.image, text1.rect)
            text2 = Text(f'점수 : {self.score}뱀의 길이 : {self.snake_sprites.sprites()[0].length} 게임 속도:{self.snake_sprites.sprites()[0].length//4}', 50, pygame.Color('white'), 30, SCREEN_Y/2, self)
            self.screen.blit(text2.image, text2.rect)
            if self.pressed_key[pygame.K_SPACE]:
                self.replay = True
                break
            pygame.display.update()

    def event(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.playing = False
                sys.exit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:
                    self.playing = False
                if event.key == pygame.K_UP:
                    self.all_sprites.sprites()[0].change_dir((0,-1))
                if event.key == pygame.K_DOWN:
                    self.all_sprites.sprites()[0].change_dir((0,1))
                if event.key == pygame.K_RIGHT:
                    self.all_sprites.sprites()[0].change_dir((1,0))
                if event.key == pygame.K_LEFT:
                    self.all_sprites.sprites()[0].change_dir((-1,0))
                    
    def make_pizza(self):
            if len(self.pizza_sprites) < 15:
                pizza = Pizza(self)
                self.all_sprites.add(pizza)
                self.pizza_sprites.add(pizza)

    def make_snake(self):
        # 스네이크 만드는 클래스.
        if len(self.snake_sprites) <= self.snake_sprites.sprites()[0].length:
            snake = Snake(self, False)
            self.all_sprites.add(snake)
            self.snake_sprites.add(snake)
            self.t_rect.append(pygame.Rect(GRID,GRID,GRID,GRID))

    def update(self):
        self.all_sprites.update()

    def draw(self):
        self.screen.fill(pygame.Color('white'))
        self.draw_grid()
        self.all_sprites.draw(self.screen)
        text1 = Text(f'점수 : {self.score}뱀의 길이 : {self.snake_sprites.sprites()[0].length} 게임 속도:{self.snake_sprites.sprites()[0].length//4}', 50, pygame.Color('black'), 30, 20, self)
        self.screen.blit(text1.image, text1.rect)
        pygame.display.update()
        
    def draw_grid(self):
        for row in range(0, int(GRID_HEIGHT)):
            for col in range(0, int(GRID_WIDTH)):
                if (row+col) % 2 == 0:
                    rect = pygame.Rect((col*GRID, row*GRID+SPAN), (GRID, GRID))
                    pygame.draw.rect(self.screen, pygame.Color('pink1'), rect)
                else:
                    rect = pygame.Rect((col*GRID, row*GRID+SPAN), (GRID, GRID))
                    pygame.draw.rect(self.screen, pygame.Color('pink3'), rect)

    def quit(self):
        pygame.quit()