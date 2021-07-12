#!/usr/bin/env python
import pygame

pygame.init()

white = (255, 255, 255)
black = (0, 0, 0)
red = (255, 0, 0)

dis = pygame.display.set_mode((800, 600))
pygame.display.set_caption('Duckiebot Trace')

game_over = False

x1 = 300
y1 = 300

x1_change = 0
y1_change = 0

clock = pygame.time.Clock()

while not game_over:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            game_over = True
    if event.type == pygame.KEYDOWN:
        if pygame.key.get_pressed()[pygame.K_LEFT]:
            x1_change = -10
            y1_change = 0
            x1 += x1_change
            y1 += y1_change
        elif pygame.key.get_pressed()[pygame.K_RIGHT]:
            x1_change = 10
            y1_change = 0
            x1 += x1_change
            y1 += y1_change
        elif pygame.key.get_pressed()[pygame.K_UP]:
            y1_change = -10
            x1_change = 0
            x1 += x1_change
            y1 += y1_change
        elif pygame.key.get_pressed()[pygame.K_DOWN]:
            y1_change = 10
            x1_change = 0
            x1 += x1_change
            y1 += y1_change



    pygame.draw.rect(dis, white, [x1, y1, 10, 10])

    pygame.display.update()

    clock.tick(30)

pygame.quit()
quit()
