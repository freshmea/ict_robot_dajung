import pygame, random
from pg_local import *
vec = pygame.math.Vector2

class Snake(pygame.sprite.Sprite):
    length = 5
    score = 0
    def __init__(self, root, head):
        super().__init__()
        self.game = root
        self.length = 1
        self.image = pygame.Surface((GRID,GRID))
        self.rect = self.image.get_rect()
        self.color1 = pygame.Color('green')
        self.color2 = pygame.Color('green4')
        self.head_color = pygame.Color('crimson')
        self.head = head
        self.create_snake()

    def create_snake(self):
        self.rect.topleft =(GRID*15, GRID*15+SPAN)
        self.direction = random.choice([(-1,0),(1,0),(0, -1),(0, 1)])
        self.game.score = 0
        if self.head:
            self.image.fill(self.head_color)
        elif self.game.color_check:
            self.image.fill(self.color1)
            self.game.color_check = False
        else:
            self.image.fill(self.color2)
            self.game.color_check = True

    def change_dir(self, dir):
        self.direction = dir

    def update(self):
        if self.head:
            self.game.t_rect[0] = self.rect.copy()
            self.rect.topleft += vec(self.direction)*GRID
            if self.rect.x >= GRID*GRID_WIDTH:
                self.rect.x = 0
            if self.rect.x < 0:
                self.rect.x = GRID*(GRID_WIDTH-1)
            if self.rect.y >= GRID*GRID_HEIGHT+SPAN:
                self.rect.y = SPAN
            if self.rect.y < SPAN:
                self.rect.y = GRID*(GRID_HEIGHT-1)+SPAN
        else:
            self.game.t_rect[list(self.game.snake_sprites.sprites()).index(self)]=self.rect.copy()
            self.rect = self.game.t_rect[list(self.game.snake_sprites.sprites()).index(self)-1]
        self.eat_pizza()
        self.self_coliide()

    def eat_pizza(self):
        if self.head:
            if pygame.sprite.spritecollide(self, self.game.pizza_sprites, True):
                Snake.length += 1
                Snake.score += 10

    def self_coliide(self):
        if self.head:
            if len(pygame.sprite.spritecollide(self, self.game.snake_sprites, False)) > 1:
                self.game.playing = False