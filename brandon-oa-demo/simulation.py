import pygame
import sys
import math
from settings import SCREEN_WIDTH, SCREEN_HEIGHT, WHITE, FPS, BLACK, GREEN
from robot import Robot
from obstacle import Obstacle

class Simulation:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption("Robot Virtual Force Sim")
        self.clock = pygame.time.Clock()
        
        self.obstacles = []
        self.course = []
        # Initial Robot
        self.robot = Robot(100, 100, self.obstacles, self.course)
        self.running = True

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            
            if event.type == pygame.MOUSEBUTTONUP:
                x, y = pygame.mouse.get_pos()
                if event.button == 1:  # Left click to add a waypoint
                    waypoint = pygame.Vector2(x, y)
                    self.course.append(waypoint)
                    if not self.robot.target:
                        self.robot.target = waypoint
                elif event.button == 3:  # Right click to add an obstacle
                    new_obs = Obstacle(x, y)
                    self.obstacles.append(new_obs)

    def update(self):
        self.robot.update_position()

    def draw(self):
        self.screen.fill((30, 30, 30))  # Dark background
        
        # Draw course
        if len(self.course) > 0:
            for point in self.course:
                pygame.draw.circle(self.screen, GREEN, (int(point.x), int(point.y)), 5)
        
        if len(self.course) > 1:
            pygame.draw.lines(self.screen, GREEN, False, self.course, 2)

        for obs in self.obstacles:
            obs.draw(self.screen)
            
        self.robot.draw(self.screen)
        pygame.display.flip()

    def run(self):
        while self.running:
            self.handle_events()
            self.update()
            self.draw()
            self.clock.tick(FPS)
        pygame.quit()
