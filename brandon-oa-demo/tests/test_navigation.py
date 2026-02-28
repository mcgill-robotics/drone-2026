"""Tests for NavigationController, SimpleNavigationStrategy, and waypoint logic."""
import pygame
import pytest

from config.navigation_config import NavigationConfig
from core.navigation_controller import NavigationController
from strategies.navigation_strategy import SimpleNavigationStrategy


def make_controller(course, arrival_radius=10.0, corner_radius=40.0, loop_threshold=20.0):
    config = NavigationConfig(
        arrival_radius=arrival_radius,
        corner_radius=corner_radius,
        loop_threshold=loop_threshold,
    )
    return NavigationController(course, config, SimpleNavigationStrategy())


# ---------------------------------------------------------------------------
# update_target: target initialization
# ---------------------------------------------------------------------------

def test_update_target_does_nothing_with_empty_course():
    ctrl = make_controller([])
    ctrl.update_target(pygame.Vector2(0, 0))
    assert ctrl.current_target is None


def test_update_target_sets_first_waypoint_on_first_call():
    course = [pygame.Vector2(100, 0), pygame.Vector2(200, 0)]
    ctrl = make_controller(course)
    ctrl.update_target(pygame.Vector2(0, 0))
    assert ctrl.current_target == pygame.Vector2(100, 0)


def test_update_target_ignores_completed_course():
    course = [pygame.Vector2(100, 0)]
    ctrl = make_controller(course)
    ctrl.course_completed = True
    ctrl.update_target(pygame.Vector2(0, 0))
    assert ctrl.current_target is None


# ---------------------------------------------------------------------------
# update_target: waypoint advancement
# ---------------------------------------------------------------------------

def test_advances_to_next_waypoint_within_corner_radius():
    course = [pygame.Vector2(100, 0), pygame.Vector2(300, 0)]
    ctrl = make_controller(course, corner_radius=40.0)
    ctrl.update_target(pygame.Vector2(0, 0))          # init → target = (100,0)
    ctrl.update_target(pygame.Vector2(95, 0))          # dist=5 < corner_radius=40 → advance
    assert ctrl.current_target == pygame.Vector2(300, 0)
    assert ctrl.current_waypoint_index == 1


def test_does_not_advance_outside_corner_radius():
    course = [pygame.Vector2(100, 0), pygame.Vector2(300, 0)]
    ctrl = make_controller(course, corner_radius=40.0)
    ctrl.update_target(pygame.Vector2(0, 0))
    ctrl.update_target(pygame.Vector2(50, 0))          # dist=50 > corner_radius=40 → stay
    assert ctrl.current_target == pygame.Vector2(100, 0)


def test_completes_course_at_final_waypoint():
    course = [pygame.Vector2(100, 0)]
    ctrl = make_controller(course, arrival_radius=10.0)
    ctrl.update_target(pygame.Vector2(0, 0))
    ctrl.update_target(pygame.Vector2(95, 0))          # dist=5 < arrival_radius=10
    assert ctrl.course_completed is True
    assert ctrl.current_target is None


def test_uses_arrival_radius_not_corner_radius_for_last_point():
    # corner_radius=80, arrival_radius=10 — should NOT complete if dist=15 (>arrival but <corner)
    course = [pygame.Vector2(100, 0)]
    ctrl = make_controller(course, arrival_radius=10.0, corner_radius=80.0)
    ctrl.update_target(pygame.Vector2(0, 0))
    ctrl.update_target(pygame.Vector2(85, 0))          # dist=15 < corner but > arrival
    assert ctrl.course_completed is False


# ---------------------------------------------------------------------------
# is_closed_loop
# ---------------------------------------------------------------------------

def test_closed_loop_detected_when_endpoints_close():
    # first=(0,0), last=(5,0) → dist=5 < loop_threshold=20
    course = [pygame.Vector2(0, 0), pygame.Vector2(100, 0), pygame.Vector2(5, 0)]
    ctrl = make_controller(course, loop_threshold=20.0)
    assert ctrl.is_closed_loop() is True


def test_closed_loop_not_detected_when_endpoints_far():
    course = [pygame.Vector2(0, 0), pygame.Vector2(100, 0), pygame.Vector2(500, 500)]
    ctrl = make_controller(course, loop_threshold=20.0)
    assert ctrl.is_closed_loop() is False


def test_closed_loop_requires_at_least_3_points():
    course = [pygame.Vector2(0, 0), pygame.Vector2(5, 0)]    # 2 points, dist=5 < 20
    ctrl = make_controller(course, loop_threshold=20.0)
    assert ctrl.is_closed_loop() is False


# ---------------------------------------------------------------------------
# Closed-loop wrap-around
# ---------------------------------------------------------------------------

def test_closed_loop_wraps_back_to_first_waypoint():
    p0, p1, p2 = pygame.Vector2(0, 0), pygame.Vector2(200, 0), pygame.Vector2(5, 0)
    course = [p0, p1, p2]
    ctrl = make_controller(course, arrival_radius=10.0, corner_radius=40.0, loop_threshold=20.0)

    # Manually place controller at the last waypoint about to arrive
    ctrl.current_target = p2
    ctrl.current_waypoint_index = 2

    ctrl.update_target(pygame.Vector2(4, 0))            # dist=1 < arrival_radius=10, is_last → loop
    assert ctrl.current_waypoint_index == 0
    assert ctrl.current_target == p0
    assert ctrl.course_completed is False


# ---------------------------------------------------------------------------
# SimpleNavigationStrategy
# ---------------------------------------------------------------------------

def make_strategy():
    return SimpleNavigationStrategy()


def test_strategy_returns_zero_for_empty_course():
    config = NavigationConfig()
    force = make_strategy().calculate_goal_force(
        pygame.Vector2(0, 0), pygame.Vector2(0, 0), [], config, 0
    )
    assert force.length() == pytest.approx(0.0)


def test_strategy_steers_toward_target():
    config = NavigationConfig(slow_radius=150.0)
    course = [pygame.Vector2(1000, 0)]                  # far to the right
    force = make_strategy().calculate_goal_force(
        pygame.Vector2(0, 0), pygame.Vector2(0, 0), course, config, 0
    )
    assert force.x > 0                                   # must push in +x direction
    assert abs(force.y) < 1e-6                           # no y component for aligned target


def test_strategy_force_smaller_near_target_than_far():
    config = NavigationConfig(slow_radius=150.0)
    pos = pygame.Vector2(0, 0)
    vel = pygame.Vector2(0, 0)

    # Within slow_radius
    near_course = [pygame.Vector2(50, 0)]
    force_near = make_strategy().calculate_goal_force(pos, vel, near_course, config, 0)

    # Outside slow_radius
    far_course = [pygame.Vector2(500, 0)]
    force_far = make_strategy().calculate_goal_force(pos, vel, far_course, config, 0)

    assert force_near.length() < force_far.length()


def test_strategy_out_of_bounds_index_returns_zero():
    config = NavigationConfig()
    course = [pygame.Vector2(100, 0)]
    force = make_strategy().calculate_goal_force(
        pygame.Vector2(0, 0), pygame.Vector2(0, 0), course, config, 5  # index out of range
    )
    assert force.length() == pytest.approx(0.0)
