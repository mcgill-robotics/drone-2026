import pygame
import math
import random
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
        
        # Virtual force (from Java implementation, but improved)
        self.virtualforce = False  # Flag like in Java
        self.virtual_force_boost = 0.5  # Strength per frame
        self.virtual_force_direction = 0  # Store the direction to apply force
        self.virtual_force_duration = 2.0  # DURATION: How long to apply force (seconds)
        self.virtual_force_timer = 0.0  # Track how long force has been active
        
        # Local minima detection
        self.position_history = []
        self.stuck_threshold = 3.0  # TIME: seconds before considering stuck
        self.stuck_distance = 20.0  # If moved less than this distance, might be stuck
        self.stuck_timer = 0.0  # Total time stuck
        self.total_time = 0.0  # Total elapsed time (for timestamps)
        self.is_stuck = False

    def find_random_safe_direction(self, obstacles):
        """
        Pick a random direction, but reject it if it points directly into an obstacle.
        Tries multiple times to find a safe direction.
        """
        max_attempts = 10
        safe_distance = 30  # Direction should have at least this much clearance
        
        for attempt in range(max_attempts):
            # Generate random direction
            angle = random.uniform(0, 2 * math.pi)
            
            # Check if this direction is safe
            clearance = self.measure_clearance(angle, obstacles, max_distance=50)
            
            if clearance > safe_distance:
                print(f"    Random safe direction found: {math.degrees(angle):.1f}° (clearance: {clearance:.1f}px)")
                return angle
        
        # If no safe direction found after attempts, just pick random
        # (this shouldn't happen often)
        fallback_angle = random.uniform(0, 2 * math.pi)
        print(f"    No safe direction found, using random: {math.degrees(fallback_angle):.1f}°")
        return fallback_angle

    def measure_clearance(self, angle, obstacles, max_distance=150):
        """
        Measure how far we can go in a given direction before hitting an obstacle.
        """
        step_size = 10
        clearance = max_distance
        
        for dist in range(step_size, int(max_distance) + step_size, step_size):
            test_x = self.x + dist * math.cos(angle)
            test_y = self.y + dist * math.sin(angle)
            
            # Check if this point collides with any obstacle
            for obs in obstacles:
                if hasattr(obs, 'is_trap') and obs.is_trap:
                    # Rectangular obstacle collision
                    margin = 10  # Safety margin
                    if (obs.rect.left - margin <= test_x <= obs.rect.right + margin and
                        obs.rect.top - margin <= test_y <= obs.rect.bottom + margin):
                        clearance = min(clearance, dist)
                        return clearance
                else:
                    # Circular obstacle collision
                    dist_to_obs = math.hypot(test_x - obs.rect.centerx, test_y - obs.rect.centery)
                    if dist_to_obs <= obs.width / 2 + 10:  # +10 margin
                        clearance = min(clearance, dist)
                        return clearance
        
        return clearance

    def detect_local_minima(self, goal, obstacles, dt):
        """Detect if player is stuck in a local minimum"""
        # Increment total time
        self.total_time += dt
        
        current_pos = (self.x, self.y)
        
        # Add current position to history with timestamp
        self.position_history.append((current_pos, self.total_time))
        
        # Keep only recent history (last stuck_threshold seconds)
        self.position_history = [(pos, t) for pos, t in self.position_history 
                                  if self.total_time - t <= self.stuck_threshold]
        
        # Check if we're far from goal
        distance_to_goal = math.hypot(self.x - goal[0], self.y - goal[1])
        
        # If we're at the goal, we're not stuck
        if distance_to_goal < 15:
            self.stuck_timer = 0.0
            self.is_stuck = False
            self.virtualforce = False
            return False
        
        # Check if we've been moving very little
        if len(self.position_history) > 1:
            oldest_pos, _ = self.position_history[0]
            distance_moved = math.hypot(current_pos[0] - oldest_pos[0], 
                                       current_pos[1] - oldest_pos[1])
            
            # Check speed
            speed = math.hypot(self.vx, self.vy)
            
            # If we've barely moved and we're moving slowly, we might be stuck
            if distance_moved < self.stuck_distance and speed < 1.0:
                self.stuck_timer += dt
                
                # If stuck for long enough, activate virtual force
                if self.stuck_timer >= self.stuck_threshold:
                    self.is_stuck = True
                    if not self.virtualforce:
                        self.virtualforce = True
                        self.virtual_force_timer = 0.0  # Reset timer
                        # Find random but safe direction
                        self.virtual_force_direction = self.find_random_safe_direction(obstacles)
                        print(f"!!! [Virtual Force] ACTIVATED for {self.virtual_force_duration}s")
                    return True
            else:
                # Reset timer if we're moving
                self.stuck_timer = max(0, self.stuck_timer - dt * 2)
                self.is_stuck = False
        
        return False

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

    def update_with_field(self, goal, obstacles, dt):
        """Update player position based on field forces"""
        if goal is None:
            return
        
        # Detect if stuck in local minimum (now passes obstacles)
        self.detect_local_minima(goal, obstacles, dt)
        
        # Calculate force from field
        fx, fy = self.calculate_field_force(goal, obstacles)
        
        # Update velocity with force (treating force as acceleration)
        self.vx += fx * 0.01  # Scale factor to control responsiveness
        self.vy += fy * 0.01
        
        # Virtual force component - applies over time in random safe direction
        if self.virtualforce:
            # Increment timer
            self.virtual_force_timer += dt
            
            # Check if still within duration
            if self.virtual_force_timer < self.virtual_force_duration:
                # Optional: Decay force over time
                # force_multiplier = 1.0 - (self.virtual_force_timer / self.virtual_force_duration)
                force_multiplier = 1.0  # Constant force
                
                # Apply boost in the random safe direction
                boost_x = self.virtual_force_boost * math.cos(self.virtual_force_direction) * force_multiplier
                boost_y = self.virtual_force_boost * math.sin(self.virtual_force_direction) * force_multiplier
                
                self.vx += boost_x
                self.vy += boost_y
                
                # Print progress every 0.5 seconds
                if int(self.virtual_force_timer * 2) != int((self.virtual_force_timer - dt) * 2):
                    print(f"Virtual Force ACTIVE ({self.virtual_force_timer:.1f}s / {self.virtual_force_duration}s)")
            else:
                # Duration expired - deactivate
                self.virtualforce = False
                self.stuck_timer = 0.0
                self.is_stuck = False
                print("!!! Virtual Force DEACTIVATED")
        
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
        # Draw player as circle - orange if stuck, black otherwise
        player_color = (255, 165, 0) if self.is_stuck else BLACK
        pygame.draw.circle(screen, player_color, (int(self.x), int(self.y)), self.width // 2)
        
        # Draw velocity vector (optional - helpful for debugging)
        if abs(self.vx) > 0.1 or abs(self.vy) > 0.1:
            end_x = int(self.x + self.vx * 5)
            end_y = int(self.y + self.vy * 5)
            pygame.draw.line(screen, (255, 0, 0), (int(self.x), int(self.y)), (end_x, end_y), 2)
        
        # Draw virtual force direction when active
        if self.virtualforce:
            # Green arrow showing escape direction
            arrow_length = 40
            end_x = int(self.x + arrow_length * math.cos(self.virtual_force_direction))
            end_y = int(self.y + arrow_length * math.sin(self.virtual_force_direction))
            pygame.draw.line(screen, (0, 255, 0), (int(self.x), int(self.y)), 
                           (end_x, end_y), 3)
            pygame.draw.circle(screen, (0, 255, 0), (end_x, end_y), 5)