import pygame, time

class Text(pygame.sprite.Sprite):
    def __init__(self, text, size, color, x, y, root) -> None:
        pygame.sprite.Sprite.__init__(self)
        self.x = x
        self.y = y
        self.game = root
        self.color = color
        self.font = pygame.font.SysFont('Noto Sans Mono CJK KR', size)
        self.image = self.font.render(text, True, self.color)
        self.rect = self.image.get_rect()
        self.rect.topleft = (self.x, self.y)
 
    def change_text(self, text):
        self.image = self.font.render(text, True, self.color)
        self.rect = self.image.get_rect()

    def update(self):
        self.change_text(f'updating')