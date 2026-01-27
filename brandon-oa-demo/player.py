import pygame
import math
from settings import SCREEN_WIDTH, SCREEN_HEIGHT, BLACK
from field import Field

class Player:
    def __init__(self):
        self.width = 20
        self.height = 20
        self.x = SCREEN_WIDTH // 2
        self.y = SCREEN_HEIGHT // 2
        self.rect = pygame.Rect(self.x - self.width // 2, self.y - self.height // 2, 
                                self.width, self.height)
        
        # Velocity
        self.vx = 0
        self.vy = 0
        
        # Physics parameters
        self.max_speed = 5.0  # Maximum velocity
        self.damping = 0.85  # Velocity damping (0-1, lower = more damping)
        
        # Field force parameters
        self.attractive_strength = 50
        self.repulsive_strength = 10000
        self.repulsive_influence_distance = 100

    def calculate_field_force(self, goal, obstacles):
        """Calculate the total force from attractive and repulsive fields"""
        if goal is None:
            return (0, 0)
        
        position = (self.x, self.y)
        total_fx, total_fy = 0, 0
        
        # Attractive force from goal
        fx, fy = Field.calculate_attractive_force(
            position, 
            goal, 
            strength=self.attractive_strength
        )
        total_fx += fx
        total_fy += fy
        
        # Repulsive forces from all obstacles
        for obstacle in obstacles:
            # Handle rectangular trap walls differently from circular obstacles
            if hasattr(obstacle, 'is_trap') and obstacle.is_trap:
                fx, fy = Field.calculate_repulsive_force_rect(
                    position,
                    obstacle.rect,
                    strength=self.repulsive_strength,
                    influence_distance=self.repulsive_influence_distance
                )
            else:
                fx, fy = Field.calculate_repulsive_force(
                    position,
                    obstacle.rect.center,
                    obstacle.width / 2,
                    strength=self.repulsive_strength,
                    influence_distance=self.repulsive_influence_distance
                )
            total_fx += fx
            total_fy += fy
        
        return (total_fx, total_fy)

    def update_with_field(self, goal, obstacles):
        """Update player position based on field forces"""
        if goal is None:
            return
        
        # Calculate force from field
        fx, fy = self.calculate_field_force(goal, obstacles)
        
        # Update velocity with force (treating force as acceleration)
        self.vx += fx * 0.01  # Scale factor to control responsiveness
        self.vy += fy * 0.01
        
        # Apply damping to simulate friction/air resistance
        self.vx *= self.damping
        self.vy *= self.damping
        
        # Limit to max speed
        speed = math.hypot(self.vx, self.vy)
        if speed > self.max_speed:
            self.vx = (self.vx / speed) * self.max_speed
            self.vy = (self.vy / speed) * self.max_speed
        
        # Update position
        self.x += self.vx
        self.y += self.vy
        
        # Keep player on screen (optional boundary)
        self.x = max(self.width // 2, min(SCREEN_WIDTH - self.width // 2, self.x))
        self.y = max(self.height // 2, min(SCREEN_HEIGHT - self.height // 2, self.y))
        
        # Update rect for drawing and collision
        self.rect.centerx = int(self.x)
        self.rect.centery = int(self.y)

    def follow_course(self, course, course_is_closed):
        """Legacy method - keeping for compatibility but not used with field-based movement"""
        pass

    def draw(self, screen):
        # Draw player as circle
        pygame.draw.circle(screen, BLACK, (int(self.x), int(self.y)), self.width // 2)
        
        # Draw velocity vector (optional - helpful for debugging)
        if abs(self.vx) > 0.1 or abs(self.vy) > 0.1:
            end_x = int(self.x + self.vx * 5)
            end_y = int(self.y + self.vy * 5)
            pygame.draw.line(screen, (255, 0, 0), (int(self.x), int(self.y)), (end_x, end_y), 2)