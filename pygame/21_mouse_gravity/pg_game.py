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
        self.ispeed = 3
        self.friction = FRICTION
        self.acc = vec(0,0)
        self.dir = vec(random.random()*2-1, random.random()*2-1).normalize()
        self.vel = self.dir * self.ispeed

    def update(self):
        # 가속도 적용(중력)
        self.vel.y += GRAVITY
        self.vel += self.acc
        # 마찰 추가
        if self.vel.y > 0:
            self.vel.y -= self.friction
        else:
            self.vel.y += self.friction
        # 속도 적용.
        self.rect.center += self.vel
        # 바운더리 제한.
        if self.rect.x < 0 or self.rect.x > SCREEN_X:
            self.vel *= -1
        if self.rect.y < 0 or self.rect.y > SCREEN_Y:
            self.vel *= -1
        if self.rect.y > SCREEN_Y:
            self.rect.y -= 30
        # 속도 제한
        if self.vel.magnitude() > 20:
            self.vel = self.vel.normalize() * 20
            
        if self.game.mouse_check:
            self.clicked()
        else:
            self.acc = vec(0,0)
    
    def clicked(self):
        try:
            self.dir =  (vec(pygame.mouse.get_pos()) - vec(self.rect.center)).normalize()
        except:
            pass
        self.acc = self.dir * GRAVITY
class Game:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_X, SCREEN_Y))
        self.clock = pygame.time.Clock()
        self.playing = True
        self.mouse_check = False
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
                if event.type == pygame.MOUSEBUTTONDOWN:
                    self.mouse_check = True
                if event.type == pygame.MOUSEBUTTONUP:
                    self.mouse_check = False
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