"""Tests for mission_controller.pathfinding"""
import math
from unittest.mock import patch

import pytest

from mission_controller.pathfinding import (
    PathfindingStrategy,
    PathPrinting,
    PotentialFieldPathfinding,
)
from mission_controller.types import Point


class TestPathfindingStrategyBase:
    def test_abstract_class_cannot_be_instantiated(self):
        with pytest.raises(TypeError):
            PathfindingStrategy()

    def test_get_next_waypoint_returns_first_and_pops(self):
        pf = PathPrinting(waypoints=[Point(1, 1), Point(2, 2), Point(3, 3)])
        assert pf.get_next_waypoint() == Point(1, 1)
        assert pf.get_next_waypoint() == Point(2, 2)
        assert len(pf.waypoints) == 1

    def test_get_next_waypoint_returns_none_when_empty(self):
        pf = PathPrinting(waypoints=[])
        assert pf.get_next_waypoint() is None

    def test_default_waypoints_is_empty_list(self):
        pf = PathPrinting()
        assert pf.waypoints == []

    def test_is_path_clear_returns_true(self):
        pf = PathPrinting()
        assert pf.is_path_clear(Point(0, 0)) is True


class TestPathPrinting:
    def test_calculate_path_delegates_to_generate_print_pattern(self):
        start, goal = Point(0, 0), Point(10, 10)
        fake_waypoints = [Point(1, 1), Point(5, 5), Point(10, 10)]

        with patch(
            "mission_controller.pathfinding.generate_print_pattern",
            return_value=fake_waypoints,
        ) as mock_gen:
            pf = PathPrinting()
            result = pf.calculate_path(start, goal)

        mock_gen.assert_called_once_with(start, goal)
        assert result == fake_waypoints
        assert pf.waypoints == fake_waypoints

    def test_heuristic_returns_euclidean_distance(self):
        pf = PathPrinting()
        assert math.isclose(pf.heuristic(Point(0, 0), Point(3, 4)), 5.0)


class TestPotentialFieldPathfinding:
    def test_init_default_obstacle_radius(self):
        pf = PotentialFieldPathfinding()
        assert pf.obstacle_radius == 10.0
        assert pf.obstacles == []
        assert pf.attractive_force_gain == 1.0
        assert pf.repulsive_force_gain == 1.0

    def test_init_custom_obstacle_radius(self):
        pf = PotentialFieldPathfinding(obstacle_radius=25.5)
        assert pf.obstacle_radius == 25.5

    def test_add_obstacle(self):
        pf = PotentialFieldPathfinding()
        pf.add_obstacle(Point(5, 5))
        pf.add_obstacle(Point(10, 10))
        assert pf.obstacles == [Point(5, 5), Point(10, 10)]

    def test_calculate_path_delegates_to_potential_field(self):
        start, goal = Point(0, 0), Point(10, 10)
        fake_path = [start, Point(5, 5), goal]

        with patch(
            "mission_controller.pathfinding.generate_potential_field_path",
            return_value=fake_path,
        ) as mock_gen:
            pf = PotentialFieldPathfinding()
            pf.add_obstacle(Point(3, 3))
            result = pf.calculate_path(start, goal)

        mock_gen.assert_called_once_with(start, goal, [Point(3, 3)])
        assert result == fake_path
        assert pf.waypoints == fake_path

    def test_attractive_force_points_toward_goal(self):
        pf = PotentialFieldPathfinding()
        # Goal is directly east -> force should be (1, 0) normalized
        fx, fy = pf.attractive_force(Point(0, 0), Point(10, 0))
        assert math.isclose(fx, 1.0)
        assert math.isclose(fy, 0.0)

    def test_attractive_force_magnitude_scales_with_gain(self):
        pf = PotentialFieldPathfinding()
        pf.attractive_force_gain = 3.0
        fx, fy = pf.attractive_force(Point(0, 0), Point(0, 5))
        assert math.isclose(fx, 0.0)
        assert math.isclose(fy, 3.0)

    def test_repulsive_force_no_obstacles_is_zero(self):
        pf = PotentialFieldPathfinding()
        assert pf.repulsive_force(Point(0, 0)) == (0, 0)

    def test_repulsive_force_ignores_distant_obstacles(self):
        pf = PotentialFieldPathfinding(obstacle_radius=5.0)
        pf.add_obstacle(Point(100, 100))  # Outside radius
        assert pf.repulsive_force(Point(0, 0)) == (0, 0)

    def test_repulsive_force_pushes_away_from_nearby_obstacle(self):
        pf = PotentialFieldPathfinding(obstacle_radius=10.0)
        pf.add_obstacle(Point(1, 0))
        # current at origin, obstacle at (1,0): direction = (-1, 0)
        fx, fy = pf.repulsive_force(Point(0, 0))
        assert fx < 0  # pushed in -x direction
        assert math.isclose(fy, 0.0)

    def test_repulsive_force_sums_multiple_obstacles(self):
        pf = PotentialFieldPathfinding(obstacle_radius=10.0)
        pf.add_obstacle(Point(1, 0))
        pf.add_obstacle(Point(-1, 0))
        # Equal and opposite contributions should cancel
        fx, fy = pf.repulsive_force(Point(0, 0))
        assert math.isclose(fx, 0.0, abs_tol=1e-9)
        assert math.isclose(fy, 0.0, abs_tol=1e-9)

    def test_repulsive_force_skips_obstacle_at_same_point(self):
        """A zero-distance obstacle would cause division by zero; must be skipped."""
        pf = PotentialFieldPathfinding(obstacle_radius=10.0)
        pf.add_obstacle(Point(0, 0))
        fx, fy = pf.repulsive_force(Point(0, 0))
        assert fx == 0
        assert fy == 0
