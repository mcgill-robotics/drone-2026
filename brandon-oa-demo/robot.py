import pygame
import math
from settings import SCREEN_WIDTH, SCREEN_HEIGHT, BLUE
import random

class Robot:
    def __init__(self, x, y, obstacles, course, dt=0.1, m=1.0, f_max=55.0, diam=15.0):
        self.pos = pygame.Vector2(x, y)
        self.vel = pygame.Vector2(0, 0)
        self.dt = dt
        self.m = m
        self.f_max = f_max
        self.max_speed = 30.0
        self.obstacles = obstacles
        self.course = course
        self.current_waypoint_index = 0
        self.diam = diam
        self.target = None
        self.virtualforce = True
        self.trail = []
        self.trail_length = 100
        self.wall_mode_direction = 0
        self.last_force = pygame.Vector2(0, 0)

    def add_noise(self, value, mean, stddev):
        return value + random.gauss(mean, stddev)

    def _update_target(self):
        if not self.target and self.course:
            self.target = self.course[0]
            
        arrival_radius = 10.0 
        corner_radius = 40.0

        if self.target:
            dist_to_target = self.pos.distance_to(self.target)
            is_last_point = self.current_waypoint_index == len(self.course) - 1
            check_radius = arrival_radius if is_last_point else corner_radius
            
            if dist_to_target < check_radius:
                if not is_last_point:
                    self.current_waypoint_index += 1
                    self.target = self.course[self.current_waypoint_index]
                else:
                    self.target = None

    def _scan_for_obstacles(self):
        # 1. Determine Safety Thresholds (Schmitt Trigger)
        if self.wall_mode_direction != 0:
            scan_radius = 110.0 # Exit Threshold (Keep mode active longer)
        else:
            scan_radius = 60.0  # Entry Threshold

        # 2. Calculate "Mind" (Future) Position
        look_ahead_dist = self.vel.length() * 1.5 
        if look_ahead_dist < 40: look_ahead_dist = 40 
        future_pos = self.pos + (self.vel.normalize() * look_ahead_dist) if self.vel.length() > 0 else self.pos

        wall_normal_sum = pygame.Vector2(0, 0)
        obs_count = 0
        
        for obs in self.obstacles:
            # --- THE FIX: DUAL CHECK ---
            # Check if the obstacle is blocking our path (Future)
            dist_future = (obs.p - future_pos).length()
            # Check if the obstacle is right next to us (Present)
            dist_present = (obs.p - self.pos).length()
            
            # If EITHER is unsafe, we consider the obstacle "active"
            if dist_future < scan_radius or dist_present < scan_radius:
                
                # Determine which vector to use for the math
                # If the future is blocked, use that (Predictive Steering)
                # If only the present is blocked (Side wall), use that (Reactive Safety)
                if dist_future < scan_radius:
                    weight = (scan_radius - dist_future) / scan_radius
                    push_vec = obs.p - future_pos
                else:
                    weight = (scan_radius - dist_present) / scan_radius
                    push_vec = obs.p - self.pos
                
                if push_vec.length() > 0:
                    wall_normal_sum += (-push_vec.normalize() * weight)
                
                obs_count += 1
                
        return wall_normal_sum, obs_count

    def _calculate_wall_following_force(self, wall_normal_sum):
        force = pygame.Vector2(0,0)
        
        urgency = wall_normal_sum.length()
        if urgency > 1.0: urgency = 1.0 
        
        if urgency > 0:
            wall_normal = wall_normal_sum.normalize()
        else:
            return pygame.Vector2(0,0)

        # Decision Logic (Hysteresis)
        if self.wall_mode_direction == 0:
            tangent_left = pygame.Vector2(-wall_normal.y, wall_normal.x)
            tangent_right = pygame.Vector2(wall_normal.y, -wall_normal.x)
            
            to_target = (self.target - self.pos).normalize() if self.target else pygame.Vector2(0,0)
            if tangent_left.dot(to_target) > tangent_right.dot(to_target):
                self.wall_mode_direction = -1
            else:
                self.wall_mode_direction = 1

        tangent_left = pygame.Vector2(-wall_normal.y, wall_normal.x)
        tangent_right = pygame.Vector2(wall_normal.y, -wall_normal.x)
        
        desired_dir = tangent_left if self.wall_mode_direction == -1 else tangent_right
        slide_strength = 250.0 + (150.0 * urgency)
        
        if self.vel.length() < 10.0:
            force += desired_dir * slide_strength * 1.5 
        else:
            desired_velocity = desired_dir * self.max_speed
            steer = desired_velocity - self.vel
            force += steer * 4.0 

        repulsion_strength = 50.0 + (450.0 * urgency)
        force += wall_normal * repulsion_strength
        
        return force

    def _calculate_free_flight_force(self):
        force = pygame.Vector2(0,0)
        if not self.target: return force

        is_last_point = self.current_waypoint_index == len(self.course) - 1
        
        desired = (self.target - self.pos)
        dist = desired.length()
        if dist > 0: desired.normalize_ip()
        
        slow_radius = 150.0 
        if is_last_point and dist < slow_radius:
            desired *= self.max_speed * (dist / slow_radius)
        else:
            desired *= self.max_speed

        steer = desired - self.vel
        force += steer
        return force

    def _apply_physics(self, force):
        if force.length() > self.f_max:
            force.scale_to_length(self.f_max)

        acceleration = force / self.m
        self.vel += acceleration * self.dt
        self.vel *= 0.975 # Air resistance
        
        if self.vel.length() > self.max_speed:
            self.vel.scale_to_length(self.max_speed)

        self.pos += self.vel * self.dt

    def _update_trail(self):
        self.trail.append(self.pos.copy())
        if len(self.trail) > self.trail_length:
            self.trail.pop(0)

    def update_position(self):
        self._update_target()
        
        new_force = pygame.Vector2(0, 0)
        
        if self.virtualforce and self.target:
            wall_normal_sum, obs_count = self._scan_for_obstacles()
            if obs_count > 0:
                new_force = self._calculate_wall_following_force(wall_normal_sum)
            else:
                self.wall_mode_direction = 0 
                new_force = self._calculate_free_flight_force()
        
        # Smoothing (0.6 = 40% old, 60% new)
        smoothing_factor = 0.6 
        smoothed_force = self.last_force.lerp(new_force, smoothing_factor)
        self.last_force = smoothed_force 
        
        self._apply_physics(smoothed_force)
        self._update_trail()

    def draw(self, surface):
        t = pygame.time.get_ticks() / 1000.0
        pulse_radius = 60.0 * (1.0 + 0.1 * math.sin(t * 5))
        
        radar_surf = pygame.Surface((pulse_radius * 2, pulse_radius * 2), pygame.SRCALPHA)
        pygame.draw.circle(radar_surf, (0, 200, 255, 40), (int(pulse_radius), int(pulse_radius)), int(pulse_radius))
        pygame.draw.circle(radar_surf, (0, 200, 255, 100), (int(pulse_radius), int(pulse_radius)), int(pulse_radius), 2)
        
        surface.blit(radar_surf, (self.pos.x - pulse_radius, self.pos.y - pulse_radius))

        if len(self.trail) > 1:
            pygame.draw.lines(surface, (100, 100, 255), False, self.trail, 2)
            
        look_ahead_dist = self.vel.length() * 1.5 
        if look_ahead_dist < 40: look_ahead_dist = 40 
        future_pos = self.pos + (self.vel.normalize() * look_ahead_dist) if self.vel.length() > 0 else self.pos

        if self.target:
            for obs in self.obstacles:
                # DEBUG: Visualize both checks
                to_future = obs.p - future_pos
                to_present = obs.p - self.pos
                dist_future = to_future.length()
                dist_present = to_present.length()
                
                avoid_dist = 60.0 
                # Check if either is active
                if dist_future < avoid_dist or dist_present < avoid_dist:
                    
                    # Choose which one to draw (closest)
                    if dist_future < dist_present:
                         outward_dir = -to_future.normalize()
                         repulsion_mag = (avoid_dist - dist_future) / avoid_dist
                    else:
                         outward_dir = -to_present.normalize()
                         repulsion_mag = (avoid_dist - dist_present) / avoid_dist

                    v_scale = 40.0 
                    # Red: Repulsion
                    pygame.draw.line(surface, (255, 50, 50), self.pos, 
                                    self.pos + outward_dir * (repulsion_mag * v_scale), 2)
                    # Green: Vortex/Slide
                    vortex_dir = pygame.Vector2(-outward_dir.y, outward_dir.x)
                    pygame.draw.line(surface, (50, 255, 50), self.pos, 
                                    self.pos + vortex_dir * (repulsion_mag * v_scale), 2)

        pygame.draw.circle(surface, (255, 255, 255), (int(self.pos.x), int(self.pos.y)), int(self.diam / 2))
        for angle in [45, 135, 225, 315]:
            rad = math.radians(angle)
            prop_pos = self.pos + pygame.Vector2(math.cos(rad), math.sin(rad)) * (self.diam * 0.8)
            pygame.draw.circle(surface, (50, 150, 250), (int(prop_pos.x), int(prop_pos.y)), 4)