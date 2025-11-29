import pygame
import sys
import math
from settings import SCREEN_WIDTH, SCREEN_HEIGHT, WHITE, FPS, BLACK, GREEN
from player import Player
from obstacle import Obstacle

class Game:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption("Object Avoidance Game")
        self.clock = pygame.time.Clock()
        self.running = True
        
        self.player = Player()
        self.obstacles = [Obstacle() for _ in range(10)]
        self.course = []
        self.course_is_closed = False

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            if event.type == pygame.MOUSEBUTTONDOWN:
                if not self.course_is_closed:
                    # Check if the click is close to the start to close the path
                    if len(self.course) > 2 and math.hypot(event.pos[0] - self.course[0][0], event.pos[1] - self.course[0][1]) < 20:
                        self.course_is_closed = True
                    else:
                        self.course.append(event.pos)
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_c:
                    self.course = []
                    self.course_is_closed = False


    def update(self):
        self.player.follow_course(self.course, self.course_is_closed)
        # for obstacle in self.obstacles:
            # obstacle.move()
        # self.check_collisions()

    def check_collisions(self):
        for obstacle in self.obstacles:
            distance = math.hypot(self.player.rect.centerx - obstacle.rect.centerx,
                                 self.player.rect.centery - obstacle.rect.centery)
            if distance < self.player.width / 2 + obstacle.width / 2:
                self.running = False


    def draw(self):
        self.screen.fill(WHITE)
        if len(self.course) > 1:
            if self.course_is_closed:
                pygame.draw.polygon(self.screen, GREEN, self.course, 2)
            else:
                pygame.draw.lines(self.screen, BLACK, False, self.course, 2)
        self.player.draw(self.screen)
        for obstacle in self.obstacles:
            obstacle.draw(self.screen)

        pygame.display.flip()

    def run(self):
        while self.running:
            self.handle_events()
            self.update()
            self.draw()
            self.clock.tick(FPS)
        
        pygame.quit()
        sys.exit()
