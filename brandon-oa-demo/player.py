import pygame
import math
from settings import SCREEN_WIDTH, SCREEN_HEIGHT, BLUE

class Player:
    def __init__(self):
        self.width = 50
        self.height = 50
        self.color = BLUE
        self.x = SCREEN_WIDTH // 2 - self.width // 2
        self.y = SCREEN_HEIGHT - self.height - 10
        self.speed = 5
        self.rect = pygame.Rect(self.x, self.y, self.width, self.height)
        self.target_index = 0

    def follow_course(self, course, course_is_closed=False):
        if not course:
            self.target_index = 0
            return

        if self.target_index >= len(course):
            # If course was changed/cleared, restart from the beginning.
            self.target_index = 0
            if not course: # check again in case course was emptied
                return

        target_pos = course[self.target_index]
        dx = target_pos[0] - self.rect.centerx
        dy = target_pos[1] - self.rect.centery
        distance = math.hypot(dx, dy)

        if distance < self.speed:
            self.rect.center = target_pos
            self.target_index += 1
            if self.target_index >= len(course) and course_is_closed:
                self.target_index = 0
        else:
            angle = math.atan2(dy, dx)
            self.rect.x += self.speed * math.cos(angle)
            self.rect.y += self.speed * math.sin(angle)

    def draw(self, surface):
        pygame.draw.circle(surface, self.color, self.rect.center, self.width // 2)
