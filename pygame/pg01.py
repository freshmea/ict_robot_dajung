import pygame

FPS = 60
SCREEN_X = 600
SCREEN_Y = 400

pygame.init()

screen = pygame.display.set_mode((SCREEN_X, SCREEN_Y))
clock = pygame.time.Clock()

playing = True
while playing:
    clock.tick(FPS)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            playing = False
    pygame.display.update()

pygame.quit()