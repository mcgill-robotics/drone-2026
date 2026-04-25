"""Tests for mission_controller.types"""
import math

import pytest

from mission_controller.types import MissionState, MissionType, Mode, Point


class TestPoint:
    def test_2d_init_defaults_z_to_zero(self):
        p = Point(1.0, 2.0)
        assert p.x == 1.0
        assert p.y == 2.0
        assert p.z == 0

    def test_3d_init(self):
        p = Point(1.0, 2.0, 3.0)
        assert (p.x, p.y, p.z) == (1.0, 2.0, 3.0)

    def test_repr(self):
        assert repr(Point(1, 2, 3)) == "Point(1, 2, 3)"

    def test_equality_same_values(self):
        assert Point(1, 2, 3) == Point(1, 2, 3)

    def test_equality_different_values(self):
        assert Point(1, 2, 3) != Point(1, 2, 4)

    def test_equality_rejects_non_point(self):
        assert Point(1, 2, 3) != (1, 2, 3)
        assert Point(1, 2, 3) != "not a point"
        assert Point(1, 2, 3) != None

    def test_distance_to_self_is_zero(self):
        p = Point(5, 10, 15)
        assert p.distance_to(p) == 0.0

    def test_distance_to_3d(self):
        a = Point(0, 0, 0)
        b = Point(1, 2, 2)  # sqrt(1 + 4 + 4) = 3
        assert math.isclose(a.distance_to(b), 3.0)

    def test_distance_to_2d_pythagorean(self):
        assert math.isclose(Point(0, 0).distance_to(Point(3, 4)), 5.0)

    def test_distance_is_symmetric(self):
        a, b = Point(1, 2, 3), Point(4, 6, 8)
        assert math.isclose(a.distance_to(b), b.distance_to(a))

    def test_to_dict(self):
        assert Point(1, 2, 3).to_dict() == {"x": 1, "y": 2, "z": 3}

    def test_from_dict_with_z(self):
        assert Point.from_dict({"x": 1, "y": 2, "z": 3}) == Point(1, 2, 3)

    def test_from_dict_missing_z_defaults_to_zero(self):
        assert Point.from_dict({"x": 5, "y": 10}) == Point(5, 10, 0)

    def test_dict_roundtrip(self):
        original = Point(3.14, 2.71, 1.41)
        assert Point.from_dict(original.to_dict()) == original


class TestMissionState:
    EXPECTED = {
        "INIT", "TAKEOFF", "LAPS", "TRANSIT_TO_SITE", "SEARCH_SITE",
        "DROP_PAYLOAD", "ENTER_BUILDING", "SEARCH_BUILDING", "SPRAY_PADS",
        "EXIT_BUILDING", "RETURN_HOME", "LAND", "COMPLETE",
    }

    def test_all_expected_states_exist(self):
        for name in self.EXPECTED:
            assert hasattr(MissionState, name), f"MissionState.{name} missing"

    def test_values_are_unique(self):
        values = [s.value for s in MissionState]
        assert len(values) == len(set(values))


class TestMissionType:
    def test_has_mission_one_and_two(self):
        assert hasattr(MissionType, "MISSION_ONE")
        assert hasattr(MissionType, "MISSION_TWO")

    def test_values_are_unique(self):
        values = [t.value for t in MissionType]
        assert len(values) == len(set(values))


class TestMode:
    EXPECTED = {"HOVER", "LAND", "ASCEND", "RETURN", "AIRBORNE"}

    def test_all_expected_modes_exist(self):
        for name in self.EXPECTED:
            assert hasattr(Mode, name), f"Mode.{name} missing"

    def test_values_are_unique(self):
        values = [m.value for m in Mode]
        assert len(values) == len(set(values))
