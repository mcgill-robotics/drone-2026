import pygame

from strategies.navigation_strategy import NavigationStrategy, SimpleNavigationStrategy

class NavigationController:
    def __init__(self, course, config, strategy: NavigationStrategy = None):
        self.course = course
        self.config = config
        self.current_waypoint_index = 0
        self.current_target = None
        self.course_completed = False
        self.strategy = strategy if strategy else SimpleNavigationStrategy()

    def update_target(self, position):
        if not self.current_target and self.course and not self.course_completed:
            self.current_target = self.course[0]
        arrival_radius = getattr(self.config, 'arrival_radius', 10.0)
        corner_radius = getattr(self.config, 'corner_radius', 40.0)
        if self.current_target:
            dist_to_target = position.distance_to(self.current_target)
            is_last_point = self.current_waypoint_index == len(self.course) - 1
            check_radius = arrival_radius if is_last_point else corner_radius
            if dist_to_target < check_radius:
                if not is_last_point:
                    self.current_waypoint_index += 1
                    self.current_target = self.course[self.current_waypoint_index]
                else:
                    if self.is_closed_loop():
                        self.current_waypoint_index = 0
                        self.current_target = self.course[0]
                    else:
                        self.current_target = None
                        self.course_completed = True

    def is_closed_loop(self):
        if len(self.course) < 3:
            return False
        loop_threshold = getattr(self.config, 'loop_threshold', 20.0)
        return self.course[0].distance_to(self.course[-1]) < loop_threshold

    def calculate_goal_force(self, position, velocity):
        # Delegate to strategy
        return self.strategy.calculate_goal_force(position, velocity, self.course, self.config, self.current_waypoint_index)
