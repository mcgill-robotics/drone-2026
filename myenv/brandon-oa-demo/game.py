import pygame
import sys
import math
from settings import SCREEN_WIDTH, SCREEN_HEIGHT, WHITE, FPS, BLACK, GREEN
from player import Player
from obstacle import Obstacle
from field import Field

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
        
        # Field visualization settings
        self.show_fields = True  # Toggle with 'F' key
        self.field_resolution = 30  # Grid spacing for visualization

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
                if event.key == pygame.K_f:
                    self.show_fields = not self.show_fields

    def update(self):
        # Determine goal: last waypoint or first waypoint if closed
        goal = None
        if self.course:
            if self.course_is_closed:
                goal = self.course[0]  # Return to start if closed
            else:
                goal = self.course[-1]  # Move to last clicked point
        
        # Update player with field forces
        if goal:
            self.player.update_with_field(goal, self.obstacles)
        
        # Optional: Check if player reached goal
        if goal:
            distance_to_goal = math.hypot(self.player.x - goal[0], self.player.y - goal[1])
            if distance_to_goal < 10:  # Within 10 pixels of goal
                # Could trigger something here (next waypoint, level complete, etc.)
                pass
        
        # for obstacle in self.obstacles:
            # obstacle.move()
        # self.check_collisions()

    def check_collisions(self):
        for obstacle in self.obstacles:
            distance = math.hypot(self.player.rect.centerx - obstacle.rect.centerx,
                                 self.player.rect.centery - obstacle.rect.centery)
            if distance < self.player.width / 2 + obstacle.width / 2:
                self.running = False

    def draw_field_visualization(self):
        """Draw arrow field to visualize forces"""
        if not self.show_fields:
            return
        
        # Only draw if we have a goal (last course point)
        if not self.course:
            return
        
        goal = self.course[-1] if not self.course_is_closed else self.course[0]
        
        for x in range(0, SCREEN_WIDTH, self.field_resolution):
            for y in range(0, SCREEN_HEIGHT, self.field_resolution):
                position = (x, y)
                
                # Calculate total force at this point
                total_fx, total_fy = 0, 0
                
                # Attractive force from goal
                fx, fy = Field.calculate_attractive_force(position, goal, strength=50)
                total_fx += fx
                total_fy += fy
                
                # Repulsive forces from obstacles
                for obstacle in self.obstacles:
                    fx, fy = Field.calculate_repulsive_force(
                        position, 
                        obstacle.rect.center, 
                        obstacle.width / 2,
                        strength=5000,
                        influence_distance=100
                    )
                    total_fx += fx
                    total_fy += fy
                
                # Draw arrow showing force direction
                magnitude = math.hypot(total_fx, total_fy)
                if magnitude > 0.1:  # Only draw if force is significant
                    # Normalize and scale for visualization
                    scale = min(self.field_resolution * 0.4, magnitude * 0.5)
                    arrow_dx = (total_fx / magnitude) * scale
                    arrow_dy = (total_fy / magnitude) * scale
                    
                    # Draw arrow
                    end_x = x + arrow_dx
                    end_y = y + arrow_dy
                    
                    # Color based on magnitude (blue = weak, red = strong)
                    color_intensity = min(255, int(magnitude * 2))
                    color = (color_intensity, 0, 255 - color_intensity)
                    
                    pygame.draw.line(self.screen, color, (x, y), (end_x, end_y), 1)
                    # Arrowhead (simple)
                    if magnitude > 1:
                        pygame.draw.circle(self.screen, color, (int(end_x), int(end_y)), 2)

    def draw(self):
        self.screen.fill(WHITE)
        
        # Draw field visualization first (under everything)
        self.draw_field_visualization()
        
        if len(self.course) > 1:
            if self.course_is_closed:
                pygame.draw.polygon(self.screen, GREEN, self.course, 2)
            else:
                pygame.draw.lines(self.screen, BLACK, False, self.course, 2)
        
        # Draw course waypoints
        for i, point in enumerate(self.course):
            color = GREEN if i == 0 else BLACK
            pygame.draw.circle(self.screen, color, point, 5)
        
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