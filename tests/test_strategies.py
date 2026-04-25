"""Tests for mission_controller.strategies"""
from unittest.mock import patch, MagicMock

import pytest

from mission_controller.strategies import MissionStrategy, MissionOne, MissionTwo
from mission_controller.types import Point


class TestMissionStrategyBase:
    def test_abstract_class_cannot_be_instantiated(self):
        with pytest.raises(TypeError):
            MissionStrategy({})

    def test_update_battery(self):
        m = MissionOne({})
        m.update_battery(42.5)
        assert m.battery_level == 42.5

    def test_add_visited_objective(self):
        m = MissionOne({})
        m.add_visited_objective("goal_a")
        m.add_visited_objective("goal_b")
        assert m.visited_objectives == ["goal_a", "goal_b"]

    def test_get_current_mission_returns_class_name(self):
        assert MissionOne({}).get_current_mission() == "MissionOne"
        assert MissionTwo({}).get_current_mission() == "MissionTwo"

    def test_switch_pathfinding_strategy(self):
        m = MissionOne({})
        fake_pf = MagicMock()
        m.switch_pathfinding_strategy(fake_pf)
        assert m.pathfinding is fake_pf


class TestMissionOne:
    def test_init_defaults(self):
        m = MissionOne({})
        assert m.lap_waypoints == []
        assert m.item_count == 0
        assert m.items_found == 0
        assert m.num_laps == 0
        assert m.battery_level == 100.0

    def test_init_with_waypoints_and_item_count(self):
        wps = [Point(1, 1), Point(2, 2)]
        m = MissionOne({"min_lat": 0}, lap_waypoints=wps, item_count=5)
        assert m.lap_waypoints == wps
        assert m.item_count == 5

    def test_execute_runs_lap_and_increments(self):
        m = MissionOne({})
        with patch("mission_controller.strategies.run_lap_algorithm") as mock_lap:
            m.execute()
            m.execute()
        assert mock_lap.call_count == 2
        assert m.num_laps == 2

    def test_nearest_potential_field_returns_first_waypoint(self):
        wps = [Point(1, 1), Point(2, 2)]
        m = MissionOne({}, lap_waypoints=wps)
        assert m.nearest_potential_field(Point(0, 0)) == Point(1, 1)

    def test_nearest_potential_field_no_waypoints_returns_query_point(self):
        m = MissionOne({})
        query = Point(7, 7)
        assert m.nearest_potential_field(query) is query

    def test_navigate_to_point_calls_goto_drone(self):
        m = MissionOne({})
        target = Point(5, 5)
        with patch("mission_controller.strategies.goto_drone") as mock_goto:
            m.navigate_to_point(target)
        mock_goto.assert_called_once_with(target)

    def test_mark_visited_appends_to_objectives(self):
        m = MissionOne({})
        m.mark_visited(Point(1, 2))
        assert m.visited_objectives == [Point(1, 2)]

    def test_increment_lap(self):
        m = MissionOne({})
        m.increment_lap()
        m.increment_lap()
        assert m.num_laps == 2


class TestMissionTwo:
    def test_init_defaults(self):
        m = MissionTwo({})
        assert m.water_tank_capacity == 100.0
        assert m.current_water_level == 100.0
        assert m.objectives == []

    def test_init_custom_capacity(self):
        m = MissionTwo({}, water_tank_capacity=50.0)
        assert m.water_tank_capacity == 50.0
        assert m.current_water_level == 50.0

    def test_add_objective(self):
        m = MissionTwo({})
        obj = MagicMock()
        obj.location = Point(1, 1)
        m.add_objective(obj)
        assert m.objectives == [obj]

    def test_decrement_water_default_10(self):
        m = MissionTwo({})
        m.decrement_water_tank_capacity()
        assert m.current_water_level == 90.0

    def test_decrement_water_custom_amount(self):
        m = MissionTwo({})
        m.decrement_water_tank_capacity(25.0)
        assert m.current_water_level == 75.0

    def test_decrement_water_clamped_at_zero(self):
        m = MissionTwo({})
        m.decrement_water_tank_capacity(150.0)
        assert m.current_water_level == 0

    def test_increment_water_refills_but_clamps_at_capacity(self):
        m = MissionTwo({}, water_tank_capacity=100.0)
        m.decrement_water_tank_capacity(30.0)  # 70 remaining
        m.increment_water_tank_capacity(50.0)  # tries 120, clamps to 100
        assert m.current_water_level == 100.0

    def test_spray_water_decrements_5_liters(self):
        m = MissionTwo({})
        m.spray_water(Point(1, 2))
        assert m.current_water_level == 95.0

    def test_spray_water_noop_when_empty(self):
        m = MissionTwo({})
        m.decrement_water_tank_capacity(100.0)  # empty
        m.spray_water(Point(1, 2))
        assert m.current_water_level == 0

    def test_execute_aborts_when_empty_tank(self):
        m = MissionTwo({})
        m.current_water_level = 0
        obj1, obj2 = MagicMock(), MagicMock()
        m.objectives = [obj1, obj2]
        m.execute()
        obj1.execute.assert_not_called()
        obj2.execute.assert_not_called()

    def test_execute_runs_objectives_and_decrements_water(self):
        m = MissionTwo({})
        obj1, obj2 = MagicMock(), MagicMock()
        m.objectives = [obj1, obj2]
        m.execute()
        obj1.execute.assert_called_once()
        obj2.execute.assert_called_once()
        # default decrement is 10 per call, 2 objectives = 20 used
        assert m.current_water_level == 80.0

    def test_nearest_potential_field_returns_first_objective(self):
        m = MissionTwo({})
        obj = MagicMock()
        m.objectives = [obj]
        assert m.nearest_potential_field(Point(0, 0)) is obj

    def test_nearest_potential_field_no_objectives_returns_query(self):
        m = MissionTwo({})
        query = Point(9, 9)
        assert m.nearest_potential_field(query) is query
