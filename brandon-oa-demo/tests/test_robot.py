"""Integration tests for Robot: properties, update loop, stuck detection, and trail."""
import pygame
import pytest

from config.robot_config import RobotConfig, StuckDetectionConfig, VisualizationConfig
from states.robot_state import AvoidingState, IdleState, MovingState, StuckState


def make_robot(x=100, y=100, obstacles=None, course=None, config=None):
    from robot import Robot
    return Robot(x, y, obstacles=obstacles or [], course=course or [], config=config)


# ---------------------------------------------------------------------------
# Properties — all must delegate to the correct subsystem
# ---------------------------------------------------------------------------

class TestRobotProperties:
    def test_pos_delegates_to_physics_position(self, simple_robot):
        assert simple_robot.pos is simple_robot.physics.position

    def test_vel_delegates_to_physics_velocity(self, simple_robot):
        assert simple_robot.vel is simple_robot.physics.velocity

    def test_m_delegates_to_physics_config(self, simple_robot):
        assert simple_robot.m == simple_robot.physics_config.mass

    def test_f_max_delegates_to_physics_config(self, simple_robot):
        assert simple_robot.f_max == simple_robot.physics_config.max_force

    def test_max_speed_delegates_to_physics_config(self, simple_robot):
        assert simple_robot.max_speed == simple_robot.physics_config.max_speed

    def test_dt_delegates_to_physics_config(self, simple_robot):
        assert simple_robot.dt == simple_robot.physics_config.dt

    def test_course_delegates_to_navigation(self, simple_robot):
        assert simple_robot.course is simple_robot.navigation.course

    def test_course_completed_delegates_to_navigation(self, simple_robot):
        assert simple_robot.course_completed == simple_robot.navigation.course_completed

    def test_current_waypoint_index_delegates_to_navigation(self, simple_robot):
        assert simple_robot.current_waypoint_index == simple_robot.navigation.current_waypoint_index

    def test_target_property_reads_navigation(self, robot_with_waypoint):
        assert robot_with_waypoint.target is robot_with_waypoint.navigation.current_target

    def test_target_setter_writes_navigation(self, simple_robot):
        wp = pygame.Vector2(300, 200)
        simple_robot.target = wp
        assert simple_robot.navigation.current_target == wp


# ---------------------------------------------------------------------------
# Initial state
# ---------------------------------------------------------------------------

class TestRobotInitialization:
    def test_starts_in_moving_state(self, simple_robot):
        assert isinstance(simple_robot.state, MovingState)

    def test_initial_trail_is_empty(self, simple_robot):
        assert simple_robot.trail == []

    def test_initial_position_matches_constructor(self, pygame_session):
        r = make_robot(x=200, y=150)
        assert r.pos.x == pytest.approx(200.0)
        assert r.pos.y == pytest.approx(150.0)

    def test_set_state_changes_state(self, simple_robot):
        simple_robot.set_state(IdleState())
        assert isinstance(simple_robot.state, IdleState)


# ---------------------------------------------------------------------------
# update_position — state transitions
# ---------------------------------------------------------------------------

class TestUpdatePositionTransitions:
    def test_transitions_to_idle_when_course_completed(self, robot_with_waypoint):
        robot_with_waypoint.navigation.course_completed = True
        robot_with_waypoint.update_position()
        assert isinstance(robot_with_waypoint.state, IdleState)

    def test_does_not_re_enter_idle_if_already_idle(self, robot_with_waypoint):
        idle = IdleState()
        robot_with_waypoint.navigation.course_completed = True
        robot_with_waypoint.set_state(idle)
        robot_with_waypoint.update_position()
        assert robot_with_waypoint.state is idle    # same object, not re-constructed

    def test_stuck_detection_triggers_stuck_state(self, pygame_session):
        """With a very low stuck_counter_threshold, stuck state is reached quickly."""
        config = RobotConfig()
        config.stuck_detection = StuckDetectionConfig(
            velocity_threshold=9999.0,   # always considered stuck
            stuck_counter_threshold=2,
            stuck_reset_threshold=1000,
        )
        course = [pygame.Vector2(500, 300)]
        r = make_robot(course=course, config=config)
        r.navigation.current_target = pygame.Vector2(500, 300)

        for _ in range(4):
            r.update_position()

        assert isinstance(r.state, StuckState)

    def test_stuck_resets_to_moving_after_reset_threshold(self, pygame_session):
        """After reset_threshold frames in StuckState, the robot returns to MovingState.

        Trace (velocity_threshold=9999 so robot is always "slow"):
          stuck_counter starts at 2, reset_threshold=3.
          Update 1: counter → 3,  3 > 3 is False  → still StuckState
          Update 2: counter → 4,  4 > 3 is True   → MovingState, counter = 0
        """
        config = RobotConfig()
        config.stuck_detection = StuckDetectionConfig(
            velocity_threshold=9999.0,
            stuck_counter_threshold=1,
            stuck_reset_threshold=3,
        )
        course = [pygame.Vector2(500, 300)]
        r = make_robot(course=course, config=config)
        r.navigation.current_target = pygame.Vector2(500, 300)
        r.set_state(StuckState())
        r.stuck_counter = 2

        r.update_position()                          # counter=3, still StuckState
        assert isinstance(r.state, StuckState)

        r.update_position()                          # counter=4, 4>3 → MovingState
        assert isinstance(r.state, MovingState)
        assert r.stuck_counter == 0

    def test_stuck_counter_resets_when_speed_recovers(self, robot_with_waypoint):
        robot_with_waypoint.stuck_counter = 10
        robot_with_waypoint.set_state(StuckState())
        # Give robot a high velocity so the stuck check clears
        robot_with_waypoint.physics.velocity = pygame.Vector2(100, 0)
        robot_with_waypoint.update_position()
        assert robot_with_waypoint.stuck_counter == 0
        assert isinstance(robot_with_waypoint.state, MovingState)


# ---------------------------------------------------------------------------
# Trail
# ---------------------------------------------------------------------------

class TestTrail:
    def test_trail_grows_by_one_per_update(self, robot_with_waypoint):
        robot_with_waypoint.navigation.current_target = pygame.Vector2(500, 300)
        before = len(robot_with_waypoint.trail)
        robot_with_waypoint.update_position()
        assert len(robot_with_waypoint.trail) == before + 1

    def test_trail_capped_at_viz_config_trail_length(self, pygame_session):
        config = RobotConfig()
        config.visualization = VisualizationConfig(trail_length=5)
        course = [pygame.Vector2(500, 300)]
        r = make_robot(course=course, config=config)
        r.navigation.current_target = pygame.Vector2(500, 300)

        for _ in range(20):
            r.update_position()

        assert len(r.trail) <= 5

    def test_trail_entries_are_position_copies(self, robot_with_waypoint):
        robot_with_waypoint.navigation.current_target = pygame.Vector2(500, 300)
        robot_with_waypoint.update_position()
        # Modifying current position should not affect previously recorded trail entry
        recorded = robot_with_waypoint.trail[-1].copy()
        robot_with_waypoint.physics.position += pygame.Vector2(100, 100)
        assert robot_with_waypoint.trail[-1] == recorded
