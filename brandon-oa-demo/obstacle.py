import pygame
import random
from settings import SCREEN_WIDTH, SCREEN_HEIGHT, RED
import math


class Obstacle:
    def __init__(self, x, y, charge=500.0, diam=20.0):
        self.p = pygame.Vector2(x, y)
        self.charge = charge
        self.diam = diam
        self.mass = 1.0 

    def distance_sqr(self, robot):
        return self.p.distance_sq_to(robot.pos)

    def distance(self, robot):
        return self.p.distance_to(robot.pos)

    def draw(self, surface):
        pygame.draw.circle(surface, (200, 50, 50), (int(self.p.x), int(self.p.y)), int(self.diam / 2))