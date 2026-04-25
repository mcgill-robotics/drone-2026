"""Tests for mission_controller.controller"""
import time
from unittest.mock import patch, MagicMock

import pytest

from mission_controller.controller import MissionController
from mission_controller.strategies import MissionOne, MissionTwo
from mission_controller.types import Mode, MissionState, MissionType, Point


BOUNDARY = {"min_lat": -10, "max_lat": 10, "min_lon": -10, "max_lon": 10}


def make_controller(**overrides):
    """Helper that builds a MissionController with sensible defaults."""
    defaults = dict(
        mission_number=1,
        site_gps=Point(5, 5),
        mission_boundary=BOUNDARY,
        home_position=Point(0, 0),
    )
    defaults.update(overrides)
    return MissionController(**defaults)


class TestMissionControllerInit:
    def test_defaults_to_mission_one(self):
        c = make_controller()
        assert c.mission_type == MissionType.MISSION_ONE
        assert isinstance(c.mission_strategy, MissionOne)
        assert c.state == MissionState.INIT
        assert c.lap_target == 3
        assert c.laps_completed == 0
        assert c.payload_available is True
        assert c.detected_pad is None
        assert c.building_interior is False

    def test_mission_two_detected_from_strategy(self):
        c = make_controller(mission_strategy=MissionTwo(BOUNDARY))
        assert c.mission_type == MissionType.MISSION_TWO

    def test_custom_num_laps(self):
        c = make_controller(num_laps=7)
        assert c.lap_target == 7

    def test_home_position_is_initial_location(self):
        home = Point(1, 2)
        c = make_controller(home_position=home)
        assert c.current_location is home

    def test_mission_duration_is_30_minutes(self):
        c = make_controller()
        assert c.mission_duration_limit == 30 * 60


class TestGetMissionStatus:
    def test_returns_expected_fields(self):
        c = make_controller(mission_number=42)
        status = c.get_mission_status()
        assert status["mission_number"] == 42
        assert status["state"] == "INIT"
        assert status["laps_completed"] == 0
        assert status["battery"] == 100.0
        assert status["strategy"] == "MissionOne"


class TestAddObjectiveAndStrategy:
    def test_add_objective(self):
        c = make_controller()
        c.add_objective("obj_a")
        c.add_objective("obj_b")
        assert c.objectives == ["obj_a", "obj_b"]

    def test_set_mission_strategy_swaps(self):
        c = make_controller()
        new_strategy = MissionTwo(BOUNDARY)
        c.set_mission_strategy(new_strategy)
        assert c.mission_strategy is new_strategy


class TestInitialize:
    def test_transitions_to_takeoff_and_sets_start_time(self):
        c = make_controller()
        assert c.mission_start_time is None
        c.initialize()
        assert c.state == MissionState.TAKEOFF
        assert c.mission_start_time is not None


class TestTakeoff:
    def test_mission_one_transitions_to_laps(self):
        c = make_controller()
        with patch("mission_controller.controller.takeoff_drone") as mock_to:
            c.takeoff()
        mock_to.assert_called_once()
        assert c.state == MissionState.LAPS
        assert c.current_mode == Mode.AIRBORNE

    def test_mission_two_transitions_to_enter_building(self):
        c = make_controller(mission_strategy=MissionTwo(BOUNDARY))
        with patch("mission_controller.controller.takeoff_drone"):
            c.takeoff()
        assert c.state == MissionState.ENTER_BUILDING


class TestDoLaps:
    def test_remaining_laps_stay_in_laps_state(self):
        c = make_controller(num_laps=3)
        c.state = MissionState.LAPS
        c.mission_strategy = MagicMock()
        c.do_laps()
        assert c.laps_completed == 1
        assert c.state == MissionState.LAPS
        c.mission_strategy.execute.assert_called_once()

    def test_final_lap_transitions_to_transit(self):
        c = make_controller(num_laps=2)
        c.state = MissionState.LAPS
        c.mission_strategy = MagicMock()
        c.do_laps()  # 1
        c.do_laps()  # 2, should transition
        assert c.laps_completed == 2
        assert c.state == MissionState.TRANSIT_TO_SITE


class TestGoToSite:
    def test_stays_transit_if_not_arrived(self):
        c = make_controller()
        c.state = MissionState.TRANSIT_TO_SITE
        with patch("mission_controller.controller.goto_drone"), \
             patch("mission_controller.controller.inside_boundary", return_value=True), \
             patch("mission_controller.controller.at_position", return_value=False):
            c.go_to_site()
        assert c.state == MissionState.TRANSIT_TO_SITE

    def test_transitions_to_search_when_arrived(self):
        c = make_controller()
        c.state = MissionState.TRANSIT_TO_SITE
        with patch("mission_controller.controller.goto_drone"), \
             patch("mission_controller.controller.inside_boundary", return_value=True), \
             patch("mission_controller.controller.at_position", return_value=True):
            c.go_to_site()
        assert c.state == MissionState.SEARCH_SITE


class TestSearchSite:
    def test_pad_detected_goes_to_drop(self):
        c = make_controller()
        pad = Point(5, 5)
        with patch(
            "mission_controller.controller.boustrophedon_search", return_value=pad
        ):
            c.search_site()
        assert c.state == MissionState.DROP_PAYLOAD
        assert c.detected_pad == pad

    def test_no_pad_goes_home(self):
        c = make_controller()
        with patch(
            "mission_controller.controller.boustrophedon_search", return_value=None
        ):
            c.search_site()
        assert c.state == MissionState.RETURN_HOME
        assert c.detected_pad is None


class TestHandleDrop:
    def test_no_payload_skips_to_home(self):
        c = make_controller()
        c.payload_available = False
        c.handle_drop()
        assert c.state == MissionState.RETURN_HOME

    def test_drops_when_pad_empty(self):
        c = make_controller()
        c.detected_pad = Point(5, 5)
        with patch(
            "mission_controller.controller.pad_has_extinguisher", return_value=False
        ), patch("mission_controller.controller.drop_payload") as mock_drop:
            c.handle_drop()
        mock_drop.assert_called_once_with(Point(5, 5))
        assert c.payload_available is False
        assert c.state == MissionState.RETURN_HOME

    def test_skips_drop_when_pad_already_has_extinguisher(self):
        c = make_controller()
        c.detected_pad = Point(5, 5)
        with patch(
            "mission_controller.controller.pad_has_extinguisher", return_value=True
        ), patch("mission_controller.controller.drop_payload") as mock_drop:
            c.handle_drop()
        mock_drop.assert_not_called()
        assert c.payload_available is True
        assert c.state == MissionState.RETURN_HOME


class TestCheckTimeout:
    def test_noop_before_mission_start(self):
        c = make_controller()
        c.check_timeout()  # start_time is None
        assert c.state == MissionState.INIT

    def test_within_limit_does_nothing(self):
        c = make_controller()
        c.mission_start_time = time.time()
        c.state = MissionState.LAPS
        c.check_timeout()
        assert c.state == MissionState.LAPS

    def test_over_limit_transitions_home(self):
        c = make_controller()
        c.mission_start_time = time.time() - (c.mission_duration_limit + 1)
        c.state = MissionState.LAPS
        c.check_timeout()
        assert c.state == MissionState.RETURN_HOME


class TestReturnHomeAndLand:
    def test_return_home_stays_if_not_arrived(self):
        c = make_controller()
        c.state = MissionState.RETURN_HOME
        with patch("mission_controller.controller.goto_drone"), \
             patch("mission_controller.controller.inside_boundary", return_value=True), \
             patch("mission_controller.controller.at_position", return_value=False):
            c.return_home()
        assert c.state == MissionState.RETURN_HOME

    def test_return_home_transitions_to_land_when_arrived(self):
        c = make_controller()
        c.state = MissionState.RETURN_HOME
        with patch("mission_controller.controller.goto_drone"), \
             patch("mission_controller.controller.inside_boundary", return_value=True), \
             patch("mission_controller.controller.at_position", return_value=True):
            c.return_home()
        assert c.state == MissionState.LAND

    def test_land_sets_complete(self):
        c = make_controller()
        with patch("mission_controller.controller.land_drone") as mock_land:
            c.land()
        mock_land.assert_called_once()
        assert c.state == MissionState.COMPLETE
        assert c.current_mode == Mode.HOVER


class TestSafeGoto:
    def test_raises_when_outside_boundary(self):
        c = make_controller()
        with patch(
            "mission_controller.controller.inside_boundary", return_value=False
        ):
            with pytest.raises(Exception, match="outside mission boundary"):
                c.safe_goto(Point(100, 100), BOUNDARY)

    def test_calls_goto_when_inside_boundary(self):
        c = make_controller()
        target = Point(5, 5)
        with patch(
            "mission_controller.controller.inside_boundary", return_value=True
        ), patch("mission_controller.controller.goto_drone") as mock_goto:
            c.safe_goto(target, BOUNDARY)
        mock_goto.assert_called_once_with(target)


class TestMissionTwoStates:
    def _m2_controller(self):
        return make_controller(
            mission_strategy=MissionTwo(BOUNDARY),
            building_entry_point=Point(1, 1),
            building_exit_point=Point(2, 2),
        )

    def test_enter_building_transitions_when_arrived(self):
        c = self._m2_controller()
        c.state = MissionState.ENTER_BUILDING
        with patch("mission_controller.controller.goto_drone"), \
             patch("mission_controller.controller.inside_boundary", return_value=True), \
             patch("mission_controller.controller.at_position", return_value=True):
            c.enter_building()
        assert c.state == MissionState.SEARCH_BUILDING
        assert c.building_interior is True

    def test_search_building_finds_pads_transitions_to_spray(self):
        c = self._m2_controller()
        with patch(
            "mission_controller.controller.boustrophedon_search",
            return_value=[Point(1, 1)],
        ):
            c.search_building()
        assert c.state == MissionState.SPRAY_PADS

    def test_search_building_no_pads_transitions_to_exit(self):
        c = self._m2_controller()
        with patch(
            "mission_controller.controller.boustrophedon_search", return_value=None
        ):
            c.search_building()
        assert c.state == MissionState.EXIT_BUILDING

    def test_spray_pads_empty_tank_exits(self):
        c = self._m2_controller()
        c.mission_strategy.current_water_level = 0
        c.spray_pads()
        assert c.state == MissionState.EXIT_BUILDING

    def test_spray_pads_uses_mission_strategy_spray(self):
        c = self._m2_controller()
        c.detected_pad = Point(2, 2)
        c.mission_strategy = MagicMock(spec=MissionTwo)
        c.mission_strategy.current_water_level = 100
        c.spray_pads()
        c.mission_strategy.spray_water.assert_called_once_with(Point(2, 2))
        assert c.state == MissionState.EXIT_BUILDING

    def test_spray_pads_with_wrong_strategy_still_transitions(self):
        c = make_controller()  # MissionOne strategy
        c.spray_pads()
        assert c.state == MissionState.EXIT_BUILDING

    def test_exit_building_transitions_home_when_arrived(self):
        c = self._m2_controller()
        c.building_interior = True
        c.state = MissionState.EXIT_BUILDING
        with patch("mission_controller.controller.goto_drone"), \
             patch("mission_controller.controller.inside_boundary", return_value=True), \
             patch("mission_controller.controller.at_position", return_value=True):
            c.exit_building()
        assert c.state == MissionState.RETURN_HOME
        assert c.building_interior is False


class TestRun:
    def test_run_mission_one_full_happy_path(self):
        c = make_controller(num_laps=1)
        c.mission_strategy = MagicMock()

        with patch("mission_controller.controller.time.sleep"), \
             patch("mission_controller.controller.takeoff_drone"), \
             patch("mission_controller.controller.land_drone"), \
             patch("mission_controller.controller.goto_drone"), \
             patch("mission_controller.controller.inside_boundary", return_value=True), \
             patch("mission_controller.controller.at_position", return_value=True), \
             patch(
                 "mission_controller.controller.boustrophedon_search",
                 return_value=Point(5, 5),
             ), \
             patch(
                 "mission_controller.controller.pad_has_extinguisher",
                 return_value=False,
             ), \
             patch("mission_controller.controller.drop_payload"):
            c.run()

        assert c.state == MissionState.COMPLETE
        assert c.laps_completed == 1
        assert c.payload_available is False
