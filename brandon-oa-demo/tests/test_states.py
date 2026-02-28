"""Tests for all robot states and their transition logic.

Each state's handle() method is tested by:
  1. Monkeypatching the robot's sensor methods so tests are deterministic.
  2. Asserting the returned force vector and any state transitions.
"""
import pygame
import pytest

from states.robot_state import AvoidingState, IdleState, MovingState, StuckState


@pytest.fixture
def robot(pygame_session):
    """Robot with one waypoint so current_target is set."""
    from robot import Robot
    course = [pygame.Vector2(500, 300)]
    r = Robot(100, 100, obstacles=[], course=course)
    r.navigation.current_target = pygame.Vector2(500, 300)
    return r


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _patch_no_obstacles(monkeypatch, robot):
    """Make sensor methods report a clear environment."""
    monkeypatch.setattr(robot, '_classify_environment', lambda: ('SPARSE', 0.0, 0.0))
    monkeypatch.setattr(robot, '_get_stable_env_type', lambda t: t)
    monkeypatch.setattr(robot, '_scan_for_obstacles', lambda env=None: (pygame.Vector2(0, 0), 0))


def _patch_with_obstacles(monkeypatch, robot, avoidance_force=None):
    """Make sensor methods report obstacles directly ahead."""
    if avoidance_force is None:
        avoidance_force = pygame.Vector2(50, 0)
    wall_normal = pygame.Vector2(1, 0)
    monkeypatch.setattr(robot, '_classify_environment', lambda: ('MODERATE', 0.1, 0.0))
    monkeypatch.setattr(robot, '_get_stable_env_type', lambda t: t)
    monkeypatch.setattr(robot, '_scan_for_obstacles', lambda env=None: (wall_normal, 3))
    monkeypatch.setattr(
        robot, '_calculate_adaptive_avoidance_force',
        lambda *_: avoidance_force
    )


# ---------------------------------------------------------------------------
# MovingState
# ---------------------------------------------------------------------------

class TestMovingState:
    def test_returns_goal_force_when_no_obstacles(self, robot, monkeypatch):
        _patch_no_obstacles(monkeypatch, robot)
        robot.set_state(MovingState())
        force = robot.state.handle(robot)
        assert force.length() > 0

    def test_returns_zero_force_when_no_target(self, robot, monkeypatch):
        _patch_no_obstacles(monkeypatch, robot)
        robot.navigation.current_target = None
        robot.set_state(MovingState())
        force = robot.state.handle(robot)
        assert force.length() == pytest.approx(0.0)

    def test_transitions_to_avoiding_when_obstacles_detected(self, robot, monkeypatch):
        avoidance_force = pygame.Vector2(30, 10)
        _patch_with_obstacles(monkeypatch, robot, avoidance_force)
        robot.set_state(MovingState())
        force = robot.state.handle(robot)
        assert isinstance(robot.state, AvoidingState)
        assert force == avoidance_force

    def test_resets_wall_mode_direction_after_memory_threshold(self, robot, monkeypatch):
        _patch_no_obstacles(monkeypatch, robot)
        robot.set_state(MovingState())
        robot.wall_mode_direction = 1
        robot.current_env_type = 'SPARSE'
        # steps_since_obstacle beyond SPARSE threshold (10)
        robot.steps_since_obstacle = 100
        robot.state.handle(robot)
        assert robot.wall_mode_direction == 0


# ---------------------------------------------------------------------------
# AvoidingState
# ---------------------------------------------------------------------------

class TestAvoidingState:
    def test_returns_avoidance_force_when_obstacles_present(self, robot, monkeypatch):
        avoidance_force = pygame.Vector2(40, 5)
        _patch_with_obstacles(monkeypatch, robot, avoidance_force)
        robot.set_state(AvoidingState())
        force = robot.state.handle(robot)
        assert isinstance(robot.state, AvoidingState)
        assert force == avoidance_force

    def test_transitions_to_moving_after_obstacles_clear(self, robot, monkeypatch):
        _patch_no_obstacles(monkeypatch, robot)
        robot.set_state(AvoidingState())
        robot.current_env_type = 'SPARSE'
        robot.steps_since_obstacle = 100   # well past SPARSE threshold (10)
        robot.state.handle(robot)
        assert isinstance(robot.state, MovingState)

    def test_stays_in_avoiding_if_not_past_memory_threshold(self, robot, monkeypatch):
        _patch_no_obstacles(monkeypatch, robot)
        robot.set_state(AvoidingState())
        robot.current_env_type = 'SPARSE'
        robot.steps_since_obstacle = 0    # below threshold
        robot.state.handle(robot)
        assert isinstance(robot.state, AvoidingState)

    def test_resets_wall_mode_direction_on_transition(self, robot, monkeypatch):
        _patch_no_obstacles(monkeypatch, robot)
        robot.set_state(AvoidingState())
        robot.wall_mode_direction = -1
        robot.current_env_type = 'SPARSE'
        robot.steps_since_obstacle = 100
        robot.state.handle(robot)
        assert robot.wall_mode_direction == 0

    def test_returns_zero_force_when_no_target(self, robot, monkeypatch):
        _patch_no_obstacles(monkeypatch, robot)
        robot.navigation.current_target = None
        robot.set_state(AvoidingState())
        force = robot.state.handle(robot)
        assert force.length() == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# StuckState
# ---------------------------------------------------------------------------

class TestStuckState:
    def test_returns_nonzero_escape_force(self, robot):
        robot.set_state(StuckState())
        # Run a few times — random angle, so occasionally might be near-zero on one axis.
        # Check magnitude over several calls.
        forces = [robot.state.handle(robot) for _ in range(10)]
        assert all(f.length() > 0 for f in forces)

    def test_escape_force_scaled_by_config_multiplier(self, robot):
        robot.stuck_config.escape_force_mult = 0.5
        robot.set_state(StuckState())
        force = robot.state.handle(robot)
        expected_max = robot.physics_config.max_force * 0.5
        assert force.length() == pytest.approx(expected_max, rel=1e-6)


# ---------------------------------------------------------------------------
# IdleState
# ---------------------------------------------------------------------------

class TestIdleState:
    def test_applies_braking_force_when_moving(self, robot):
        robot.physics.velocity = pygame.Vector2(10, 0)
        robot.set_state(IdleState())
        force = robot.state.handle(robot)
        assert force.length() > 0
        assert force.x < 0   # braking opposes +x motion

    def test_returns_zero_force_when_already_stopped(self, robot):
        robot.physics.velocity = pygame.Vector2(0, 0)
        robot.set_state(IdleState())
        force = robot.state.handle(robot)
        assert force.length() == pytest.approx(0.0)

    def test_braking_force_magnitude(self, robot):
        robot.physics.velocity = pygame.Vector2(10, 0)
        robot.set_state(IdleState())
        force = robot.state.handle(robot)
        expected = robot.physics_config.max_force * 0.8
        assert force.length() == pytest.approx(expected, rel=1e-5)
