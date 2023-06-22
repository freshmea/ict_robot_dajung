import pygame, sys, random
from pg_local import *
from pg_text import Text
vec = pygame.math.Vector2

class Pizza(pygame.sprite.Sprite):
    def __init__(self, root):
        super().__init__()
        self.image = pygame.Surface((GRID,GRID))
        self.rect = self.image.get_rect()
        self.image.fill(pygame.Color('red'))
        self.rect.topleft = (random.randint(0, GRID_WIDTH-1)*GRID, random.randint(0, GRID_HEIGHT-1)*GRID+SPAN)

class Snake(pygame.sprite.Sprite):
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
        self.length = 3
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
                self.rect.x = GRID*GRID_WIDTH
            if self.rect.y >= GRID*GRID_WIDTH+SPAN:
                self.rect.y = SPAN
            if self.rect.y < SPAN:
                self.rect.y = GRID*(GRID_HEIGHT-1)+SPAN
        else:
            self.game.t_rect[self.game.snakes.index(self)]=self.rect.copy()
            self.rect = self.game.t_rect[self.game.snakes.index(self)-1]
        if self.eat_pizza() and self.head:
            self.length += 1
            self.game.score += 10
    
    def eat_pizza(self):
        return pygame.sprite.spritecollide(self, self.game.pizza_sprites, True)

class Game:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_X, SCREEN_Y))
        self.clock = pygame.time.Clock()
        self.playing = True
        self.replay = True
        self.score = 0
        self.color_check = True
        self.all_sprites = pygame.sprite.Group()
        self.pizza_sprites = pygame.sprite.Group()
        snake = Snake(self, True)
        self.all_sprites.add(snake)
        self.snakes =[snake]
        self.t_rect = [pygame.Rect(GRID,GRID,GRID,GRID)]

    def run(self):
        self.opening()
        self.playing = True
        while self.playing:
            self.clock.tick(FPS+self.snakes[0].length)
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
                if event.key == pygame.K_UP:
                    self.all_sprites.sprites()[0].change_dir((0,-1))
                if event.key == pygame.K_DOWN:
                    self.all_sprites.sprites()[0].change_dir((0,1))
                if event.key == pygame.K_RIGHT:
                    self.all_sprites.sprites()[0].change_dir((1,0))
                if event.key == pygame.K_LEFT:
                    self.all_sprites.sprites()[0].change_dir((-1,0))

    def make_snake(self):
        # 스네이크 만드는 클래스.
        if len(self.snakes) <= self.snakes[0].length:
            snake = Snake(self, False)
            self.all_sprites.add(snake)
            self.snakes.append(snake)
            self.t_rect.append(pygame.Rect(GRID,GRID,GRID,GRID))

    def make_pizza(self):
        if len(self.pizza_sprites) <= 10:
            pizza = Pizza(self)
            self.all_sprites.add(pizza)
            self.pizza_sprites.add(pizza)

    def update(self):
        self.all_sprites.update()

    def draw(self):
        self.screen.fill(pygame.Color('white'))
        self.draw_grid()
        self.all_sprites.draw(self.screen)
        self.ui()
        pygame.display.update()
    
    # 배경 칸 만듬.
    def draw_grid(self):
        for row in range(0, int(GRID_HEIGHT)):
            for col in range(0, int(GRID_WIDTH)):
                if (row+col) % 2 == 0:
                    rect = pygame.Rect((col*GRID, row*GRID+SPAN), (GRID, GRID))
                    pygame.draw.rect(self.screen, pygame.Color('pink1'), rect)
                else:
                    rect = pygame.Rect((col*GRID, row*GRID+SPAN), (GRID, GRID))
                    pygame.draw.rect(self.screen, pygame.Color('pink3'), rect)
    
    def ui(self):
        text1 = Text(f' 점수 : {self.score} 뱀길이: {self.snakes[0].length} LV: {int(self.snakes[0].length // 4)} ',50, pygame.Color('black'), 20, 20, self)
        self.screen.blit(text1.image, text1.rect)
        

    def quit(self):
        pygame.quit()