import pygame, random
from pg_local import *
vec = pygame.math.Vector2

class Ball(pygame.sprite.Sprite):
    def __init__(self, root, size):
        super().__init__()
        self.game = root
        self.size = size
        self.image = pygame.Surface((self.size*2, self.size*2))
        self.color = (random.randint(0,255),random.randint(0,255),random.randint(0,255))
        pygame.draw.circle(self.image, self.color, (self.size, self.size), self.size)
        self.image.set_colorkey((0,0,0))
        self.rect = self.image.get_rect()
        self.rect.center = vec(random.randint(0, SCREEN_X),random.randint(0, SCREEN_Y))
        self.vel = vec(random.randint(-100, 100)/10, random.randint(-100, 100)/10)
        self.acc = vec(0,0)
    
    def update(self):
        self.gravity()
        self.vel += self.acc
        self.rect.center += self.vel
        if self.rect.x < 0:
            self.rect.x = SCREEN_X
        if self.rect.x > SCREEN_X:
            self.rect.x  = 0
        if self.rect.y < 0:
            self.rect.y = SCREEN_Y
        if self.rect.y > SCREEN_Y:
            self.rect.y = 0
        
    def gravity(self):
        self.acc = vec(0,0)
        for ball in self.game.all_sprites.sprites():
            if not (ball == self):
                try:
                    distance = (vec(self.rect.center)-vec(ball.rect.center)).length()
                    direction =(vec(self.rect.center)-vec(ball.rect.center)).normalize()
                    self.acc += GCONSTANT * direction * self.size * ball.size/(distance*distance)
                except:
                    pass
                    
        print(self.acc)