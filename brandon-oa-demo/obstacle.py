import pygame
import random
from settings import SCREEN_WIDTH, SCREEN_HEIGHT, RED
import math


class Obstacle:
    def __init__(self):
        self.width = 50
        self.height = 50
        self.color = RED
        self.x = random.randint(0, SCREEN_WIDTH - self.width)
        self.y = random.randint(0, SCREEN_HEIGHT - self.height)
        self.speed = random.randint(3, 7)
        self.rect = pygame.Rect(self.x, self.y, self.width, self.height)
        self.DIAM = 50
        self.CHARGE = -1
        self.MASS = 10
        
    

    def move(self):
        self.rect.y += self.speed
        
    def distanceSquared(self, player):
        dx = self.rect.centerx - player.rect.centerx
        dy = self.rect.centery - player.rect.centery
        return dx * dx + dy * dy

    def distance(self, player):
        return math.hypot(self.rect.centerx - player.rect.centerx, self.rect.centery - player.rect.centery)

        

    def draw(self, surface):
        pygame.draw.circle(surface, self.color, self.rect.center, self.width // 2)
