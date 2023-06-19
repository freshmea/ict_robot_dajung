import pygame, time
from pg_local import *
import random
from pg_cloud import Cloud
from pg_player import Player
from pg_text import Text

class Game:
    def __init__(self) -> None:
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_X, SCREEN_Y))
        self.clock = pygame.time.Clock()
        self.playing = True
        # 게임 시간
        self.time = time.time()
        self.hit_rain = int()
        # 배경 음악
        pygame.mixer.music.load('wave/bgmusic.wav')
        pygame.mixer.music.set_volume(0.4)
        pygame.mixer.music.play(-1)
        # 효과음
        self.hit_sound = pygame.mixer.Sound('wave/explosion2.wav')
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
        # 눌린 키 체크
        self.pressed_key = pygame.key.get_pressed()

    def load_data(self):
        self.image = pygame.image.load('image/background01.png').convert_alpha()
        self.image = pygame.transform.scale(self.image, (SCREEN_X, SCREEN_Y))

    def run(self):
        self.opening()
        while self.playing:
            self.clock.tick(FPS)
            self.event()
            self.update()
            self.draw()

    def opening(self):
        text1 = Text('빨간 비를 맞지 않고 오래 버텨 보세요.', 50, pygame.Color('red'), 100, 10+50 ,self)
        text2 = Text('스페이스 바를 누르면 1초 동안 우산이 펴집니다.', 50, pygame.Color('red'), 100, 10+110 ,self)
        text3 = Text('s를 누르면 게임이 시작됩니다.', 50, pygame.Color('red'), 100, 10+170 ,self)
        while True:
            self.event()
            self.screen.fill((255,255,255))
            self.screen.blit(text1.image, text1.rect)
            self.screen.blit(text2.image, text2.rect)
            self.screen.blit(text3.image, text3.rect)
            pygame.display.update()
            if self.pressed_key[pygame.K_s]:
                break
            
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