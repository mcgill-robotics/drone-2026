"""Tests for mission_controller.objectives"""
from unittest.mock import patch

import pytest

from mission_controller.objectives import (
    ExtinguishObjective,
    Objective,
    PayloadDeliveryObjective,
    SurveyObjective,
)
from mission_controller.types import Point


class TestObjectiveBase:
    def test_abstract_class_cannot_be_instantiated(self):
        with pytest.raises(TypeError):
            Objective(Point(0, 0))

    def test_initial_state_not_complete(self):
        obj = ExtinguishObjective(Point(1, 2))
        assert obj.complete is False
        assert obj.is_complete() is False

    def test_set_complete_marks_done(self):
        obj = ExtinguishObjective(Point(1, 2))
        obj.set_complete()
        assert obj.complete is True
        assert obj.is_complete() is True

    def test_to_dict_serialization(self):
        obj = ExtinguishObjective(Point(1, 2, 3))
        d = obj.to_dict()
        assert d["type"] == "ExtinguishObjective"
        assert d["location"] == {"x": 1, "y": 2, "z": 3}
        assert d["complete"] is False


class TestExtinguishObjective:
    def test_execute_calls_extinguish_fire_and_marks_complete(self):
        obj = ExtinguishObjective(Point(5, 5))
        with patch("mission_controller.objectives.extinguish_fire") as mock_ext:
            obj.execute()
        mock_ext.assert_called_once_with(Point(5, 5))
        assert obj.is_complete() is True

    def test_detect_fire_sets_flag_and_returns_true(self):
        obj = ExtinguishObjective(Point(0, 0))
        assert obj.fire_detected is False
        assert obj.detect_fire() is True
        assert obj.fire_detected is True


class TestSurveyObjective:
    def test_init_photos_taken_zero(self):
        obj = SurveyObjective(Point(0, 0))
        assert obj.photos_taken == 0

    def test_execute_calls_take_survey_and_stores_count(self):
        obj = SurveyObjective(Point(10, 10))
        with patch(
            "mission_controller.objectives.take_survey_photos", return_value=7
        ) as mock_survey:
            obj.execute()
        mock_survey.assert_called_once_with(Point(10, 10))
        assert obj.photos_taken == 7
        assert obj.is_complete() is True


class TestPayloadDeliveryObjective:
    def test_init_payload_type_is_none(self):
        obj = PayloadDeliveryObjective(Point(0, 0))
        assert obj.payload_type is None

    def test_set_payload_type(self):
        obj = PayloadDeliveryObjective(Point(0, 0))
        obj.set_payload_type("extinguisher")
        assert obj.payload_type == "extinguisher"

    def test_execute_calls_release_payload_and_marks_complete(self):
        obj = PayloadDeliveryObjective(Point(2, 3, 4))
        with patch("mission_controller.objectives.release_payload") as mock_release:
            obj.execute()
        mock_release.assert_called_once_with(Point(2, 3, 4))
        assert obj.is_complete() is True
