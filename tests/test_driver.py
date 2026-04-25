"""Tests for mission_controller.driver"""
import json
from unittest.mock import patch, MagicMock

import pytest

from mission_controller.driver import Driver
from mission_controller.types import MissionState, Point


BOUNDARY = {"min_lat": -10, "max_lat": 10, "min_lon": -10, "max_lon": 10}


class TestDriverInit:
    def test_empty_state_on_init(self):
        d = Driver()
        assert d.missions == {}
        assert d.current_mission is None
        assert d.mission_logs == []


class TestCreateMission:
    def test_creates_and_registers_mission(self):
        d = Driver()
        mission = d.create_mission(
            mission_id="m1",
            site_gps=Point(5, 5),
            mission_boundary=BOUNDARY,
            home_position=Point(0, 0),
        )
        assert "m1" in d.missions
        assert d.missions["m1"] is mission
        assert mission.mission_number == "m1"


class TestStartMission:
    def test_unknown_mission_returns_false(self):
        d = Driver()
        assert d.start_mission("nope") is False

    def test_successful_run_logs_completed(self):
        d = Driver()
        mock_mission = MagicMock()
        d.missions["m1"] = mock_mission
        assert d.start_mission("m1") is True
        mock_mission.run.assert_called_once()
        assert len(d.mission_logs) == 1
        assert d.mission_logs[0]["status"] == "COMPLETED"

    def test_failing_run_logs_failed(self):
        d = Driver()
        mock_mission = MagicMock()
        mock_mission.run.side_effect = RuntimeError("boom")
        d.missions["m1"] = mock_mission
        assert d.start_mission("m1") is False
        assert d.mission_logs[0]["status"].startswith("FAILED")
        assert "boom" in d.mission_logs[0]["status"]


class TestAbortMission:
    def test_abort_sets_state_to_return_home(self):
        d = Driver()
        mock_mission = MagicMock()
        d.missions["m1"] = mock_mission
        d.abort_mission("m1")
        assert mock_mission.state == MissionState.RETURN_HOME
        assert d.mission_logs[-1]["status"] == "ABORTED"

    def test_abort_unknown_is_noop(self):
        d = Driver()
        d.abort_mission("unknown")  # should not raise
        assert d.mission_logs == []


class TestGetMissionStatus:
    def test_returns_status_for_known_mission(self):
        d = Driver()
        mock_mission = MagicMock()
        mock_mission.get_mission_status.return_value = {"state": "INIT"}
        d.missions["m1"] = mock_mission
        assert d.get_mission_status("m1") == {"state": "INIT"}

    def test_returns_none_for_unknown_mission(self):
        d = Driver()
        assert d.get_mission_status("unknown") is None


class TestLogMission:
    def test_entry_has_id_status_and_timestamp(self):
        d = Driver()
        d.log_mission("m1", "TESTING")
        entry = d.mission_logs[0]
        assert entry["mission_id"] == "m1"
        assert entry["status"] == "TESTING"
        assert "timestamp" in entry


class TestExportLogs:
    def test_writes_json_to_file(self, tmp_path):
        d = Driver()
        d.log_mission("m1", "COMPLETED")
        d.log_mission("m2", "FAILED: oops")
        out = tmp_path / "logs.json"
        d.export_logs(str(out))
        loaded = json.loads(out.read_text())
        assert len(loaded) == 2
        assert loaded[0]["mission_id"] == "m1"
        assert loaded[1]["mission_id"] == "m2"

    def test_handles_write_error_gracefully(self):
        d = Driver()
        d.log_mission("m1", "COMPLETED")
        # invalid path should not raise
        d.export_logs("/nonexistent_dir/logs.json")
