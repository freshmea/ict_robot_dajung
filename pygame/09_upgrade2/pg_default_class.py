import pygame, time
from pg_local import *
import random
from pg_cloud import Cloud
from pg_player import Player

class Text(pygame.sprite.Sprite):
    def __init__(self, text, size, color, x, y, root) -> None:
        self.x = x
        self.y = y
        self.game = root
        self.color = color
        self.font = pygame.font.SysFont('Noto Sans Mono CJK KR', size)
        self.image = self.font.render(text, True, self.color)
        self.rect = self.image.get_rect()
        self.rect.topleft = (self.x, self.y)
        self.groups = [self.game.all_sprites, self.game.text_sprites]
        pygame.sprite.Sprite.__init__(self, self.groups)
 
    def change_text(self, text):
        self.image = self.font.render(text, True, self.color)
        self.rect = self.image.get_rect()

    def update(self):
        self.change_text(f'맞은 비의 수: {self.game.hit_rain} 우산 펴기까지 남은 시간{20-(time.time()-self.game.player_sprites.sprites()[0].utime)}')

class Game:
    def __init__(self) -> None:
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_X, SCREEN_Y))
        self.clock = pygame.time.Clock()
        self.playing = True
        self.time = time.time()
        self.load_data()
        self.all_sprites = pygame.sprite.Group()
        self.player_sprites = pygame.sprite.Group()
        self.cloud_sprites = pygame.sprite.Group()
        self.text_sprites = pygame.sprite.Group()
        # 플레이어 생성
        player = Player(300,SCREEN_Y-200, self)
        self.player_sprites.add(player)
        self.all_sprites.add(player)
        # 텍스트 생성
        text = Text('this is test', 50, pygame.Color('red'), 10, 10 ,self)
        self.text_sprites.add(text)
        self.all_sprites.add(text)
        self.pressed_key = pygame.key.get_pressed()
        self.hit_rain = int()

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
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:
                    self.playing = False
            if event.type == pygame.MOUSEBUTTONDOWN:
                for cloud in self.cloud_sprites:
                    if cloud.is_click():
                        self.cloud_sprites.remove(cloud)
                        self.all_sprites.remove(cloud)
                        del cloud
        # hit_rain 종료 조건 체크
        if self.hit_rain >= 30:
            print(f'플레이 시간은: {(time.time() - self.time):.2f} s')
            self.playing = False
        
        # 구름 만들어 지는 코드
        if len(self.cloud_sprites) < CLOUD_NUMBER:
            cloud = Cloud(random.randint(0, SCREEN_X), self)
            self.all_sprites.add(cloud)
            self.cloud_sprites.add(cloud)
        self.pressed_key = pygame.key.get_pressed()
    
    def update(self):
        global CLOUD_NUMBER
        self.all_sprites.update()
        if self.pressed_key[pygame.K_a]:
            CLOUD_NUMBER += 1

    def draw(self):
        self.screen.fill((255,255,255))
        self.screen.blit(self.image, (0,0))
        self.all_sprites.draw(self.screen)
        # text 를 한번 더 그림.
        self.text_sprites.draw(self.screen)
        pygame.display.update()

    def quit(self):
        pygame.quit()