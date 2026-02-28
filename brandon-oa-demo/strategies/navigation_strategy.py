from abc import ABC, abstractmethod
import pygame

class NavigationStrategy(ABC):
    @abstractmethod
    def calculate_goal_force(self, position, velocity, course, config, current_waypoint_index):
        pass

class SimpleNavigationStrategy(NavigationStrategy):
    def calculate_goal_force(self, position, velocity, course, config, current_waypoint_index):
        if not course or current_waypoint_index >= len(course):
            return pygame.Vector2(0, 0)
        target = course[current_waypoint_index]
        desired = (target - position)
        dist = desired.length()
        if dist > 0:
            desired.normalize_ip()
        slow_radius = getattr(config, 'slow_radius', 150.0)
        max_speed = getattr(config, 'max_speed', 30.0)
        if dist < slow_radius:
            desired *= max_speed * (dist / slow_radius)
        else:
            desired *= max_speed
        steer = desired - velocity
        return steer
