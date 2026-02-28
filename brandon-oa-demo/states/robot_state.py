from abc import ABC, abstractmethod
import math
import random
import pygame


class RobotState(ABC):
    @abstractmethod
    def handle(self, robot) -> pygame.Vector2:
        """Calculate the desired force for this frame and handle state transitions.
        Returns the desired force vector."""
        pass


class MovingState(RobotState):
    """Normal navigation toward waypoints with no obstacles in range."""

    def handle(self, robot) -> pygame.Vector2:
        if not robot.navigation.current_target:
            return pygame.Vector2(0, 0)

        env_type, density, _ = robot._classify_environment()
        robot.current_env_type = robot._get_stable_env_type(env_type)

        wall_normal, obs_count = robot._scan_for_obstacles(robot.current_env_type)

        if obs_count > 0:
            robot.steps_since_obstacle = 0
            robot.set_state(AvoidingState())
            return robot._calculate_adaptive_avoidance_force(wall_normal, robot.current_env_type, density)

        robot.steps_since_obstacle += 1
        mem_thresh = robot.avoidance_config.memory_thresholds.get(robot.current_env_type, 20)
        if robot.steps_since_obstacle > mem_thresh:
            robot.wall_mode_direction = 0

        return robot.navigation.calculate_goal_force(robot.physics.position, robot.physics.velocity)


class AvoidingState(RobotState):
    """Active obstacle avoidance using virtual force fields."""

    def handle(self, robot) -> pygame.Vector2:
        if not robot.navigation.current_target:
            return pygame.Vector2(0, 0)

        env_type, density, _ = robot._classify_environment()
        robot.current_env_type = robot._get_stable_env_type(env_type)

        wall_normal, obs_count = robot._scan_for_obstacles(robot.current_env_type)

        if obs_count == 0:
            robot.steps_since_obstacle += 1
            mem_thresh = robot.avoidance_config.memory_thresholds.get(robot.current_env_type, 20)
            if robot.steps_since_obstacle > mem_thresh:
                robot.wall_mode_direction = 0
                robot.set_state(MovingState())
            return robot.navigation.calculate_goal_force(robot.physics.position, robot.physics.velocity)

        robot.steps_since_obstacle = 0
        return robot._calculate_adaptive_avoidance_force(wall_normal, robot.current_env_type, density)


class StuckState(RobotState):
    """Emergency recovery: applies a random escape force to break out of local minima."""

    def handle(self, robot) -> pygame.Vector2:
        escape_angle = random.uniform(-math.pi, math.pi)
        return pygame.Vector2(
            math.cos(escape_angle) * robot.physics_config.max_force * robot.stuck_config.escape_force_mult,
            math.sin(escape_angle) * robot.physics_config.max_force * robot.stuck_config.escape_force_mult,
        )


class IdleState(RobotState):
    """Course completed; apply braking until the robot comes to rest."""

    def handle(self, robot) -> pygame.Vector2:
        if robot.physics.velocity.length() > 0.5:
            return -robot.physics.velocity.normalize() * robot.physics_config.max_force * 0.8
        return pygame.Vector2(0, 0)
