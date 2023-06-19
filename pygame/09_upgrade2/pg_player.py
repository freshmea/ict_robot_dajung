import pygame

class Player(pygame.sprite.Sprite):
    def __init__(self, x, y, root) -> None:
        self.x = x
        self.y = y
        self.speed = 10
        self.game = root
        self.images = [pygame.image.load('image/player1.png'),pygame.image.load('image/player2.png')]
        self.image = self.images[0]
        self.rect = self.image.get_rect()
        self.rect.center = (self.x, self.y)
        self.mask = pygame.mask.from_surface(self.image)
        self.groups = [self.game.all_sprites, self.game.player_sprites]
        pygame.sprite.Sprite.__init__(self, self.groups)
        
    def update(self):
        if self.game.pressed_key[pygame.K_UP]:
            self.y += -self.speed
        if self.game.pressed_key[pygame.K_DOWN]:
            self.y += self.speed
        if self.game.pressed_key[pygame.K_LEFT]:
            self.x += -self.speed
        if self.game.pressed_key[pygame.K_RIGHT]:
            self.x += self.speed
        if self.game.pressed_key[pygame.K_SPACE]:
            self.image = self.images[1]
            self.rect = self.image.get_rect()
            self.mask = pygame.mask.from_surface(self.image)
        else:
            self.image = self.images[0]
            self.rect = self.image.get_rect()
            self.mask = pygame.mask.from_surface(self.image)
        self.rect.center = (self.x, self.y)
