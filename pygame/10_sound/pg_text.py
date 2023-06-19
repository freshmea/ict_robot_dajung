import pygame, time

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
        self.change_text(f'맞은 비의 수: {self.game.hit_rain} 우산 펴기까지 남은 시간: {20-(time.time()-self.game.player_sprites.sprites()[0].utime):.2f}')