import pygame, random
from pg_local import *
vec = pygame.math.Vector2
class Rectagle(pygame.sprite.Sprite):
    def __init__(self, x, y, root):
        super().__init__()
        self.game = root
        self.image = pygame.Surface((30, 30))
        self.image.fill(pygame.Color('crimson'))
        self.rect = self.image.get_rect()
        self.rect.center = vec(x, y)
        self.speed = 3
        self.dir = vec(random.random()*2-1, random.random()*2-1).normalize()

    def update(self):
        # 속도 적용.
        self.rect.center += self.dir * self.speed
        
        # 바운더리 제한.
        if self.rect.x < 0 or self.rect.x > SCREEN_X:
            self.dir *= -1
        if self.rect.y < 0 or self.rect.y > SCREEN_Y:
            self.dir *= -1
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
                # q 를 입력 하면 게임 종료
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_q:
                        self.playing = False
        # Rectangle 클래스로 객체 20개 만들기.
        if len(self.all_sprites) <= 20:
            self.all_sprites.add(Rectagle(random.randint(0, SCREEN_X), random.randint(0, SCREEN_Y), self))

    def update(self):
        self.all_sprites.update()

    def draw(self):
        self.screen.fill(pygame.Color('navajowhite1'))
        self.all_sprites.draw(self.screen)
        pygame.display.update()
    def quit(self):
        pygame.quit()