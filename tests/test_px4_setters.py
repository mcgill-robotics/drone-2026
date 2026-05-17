"""
Tests for mission_controller.px4_setters.

PX4Setters is a mixin - it assumes the host class provides attributes like
`connected`, `arming_client`, `setpoint_pub`, etc. We build a fake host here
and drive the mixin with mocked MAVROS service/publisher clients.
"""
from unittest.mock import MagicMock, patch

import pytest

from mission_controller.px4_setters import PX4Setters


class FakePX4(PX4Setters):
    """Minimal host class for the PX4Setters mixin."""

    def __init__(self, connected=True, armed=False):
        self.connected = connected
        self._armed = armed
        self.current_position = MagicMock()
        self.current_position.pose.position.x = 0.0
        self.current_position.pose.position.y = 0.0
        self.current_position.pose.position.z = 0.0
        self.param_get_client = MagicMock()
        self.arming_client = MagicMock()
        self.set_mode_client = MagicMock()
        self.takeoff_client = MagicMock()
        self.land_client = MagicMock()
        self.setpoint_pub = MagicMock()
        self.velocity_setpoint_pub = MagicMock()

    def is_armed(self):
        return self._armed

    def get_clock(self):
        clk = MagicMock()
        clk.now.return_value.to_msg.return_value = "fake_stamp"
        return clk


def _future_with_result(result):
    """Helper to build a MagicMock future that's done with a given result."""
    f = MagicMock()
    f.done.return_value = True
    f.result.return_value = result
    return f


def _future_pending():
    f = MagicMock()
    f.done.return_value = False
    f.result.return_value = None
    return f


# ---------------------------------------------------------------------------
# arm / disarm
# ---------------------------------------------------------------------------
class TestArmVehicle:
    def test_not_connected_returns_false(self):
        px4 = FakePX4(connected=False)
        assert px4.arm_vehicle() is False

    def test_already_armed_returns_true(self):
        px4 = FakePX4(armed=True)
        assert px4.arm_vehicle() is True
        px4.arming_client.call_async.assert_not_called()

    def test_successful_arm(self):
        px4 = FakePX4(armed=False)
        result = MagicMock(success=True)
        px4.arming_client.call_async.return_value = _future_with_result(result)
        with patch("mission_controller.px4_setters.rclpy.spin_once"), \
             patch("mission_controller.px4_setters.time.sleep"):
            assert px4.arm_vehicle() is True
        px4.arming_client.call_async.assert_called_once()

    def test_failed_arm_returns_false(self):
        px4 = FakePX4(armed=False)
        result = MagicMock(success=False)
        px4.arming_client.call_async.return_value = _future_with_result(result)
        with patch("mission_controller.px4_setters.rclpy.spin_once"), \
             patch("mission_controller.px4_setters.time.sleep"):
            assert px4.arm_vehicle() is False

    def test_arm_timeout_returns_false(self):
        px4 = FakePX4(armed=False)
        px4.arming_client.call_async.return_value = _future_pending()
        with patch("mission_controller.px4_setters.rclpy.spin_once"), \
             patch("mission_controller.px4_setters.time.sleep"):
            assert px4.arm_vehicle(timeout=0) is False

    def test_arm_exception_returns_false(self):
        px4 = FakePX4(armed=False)
        px4.arming_client.call_async.side_effect = RuntimeError("boom")
        assert px4.arm_vehicle() is False


class TestDisarmVehicle:
    def test_not_connected_returns_false(self):
        px4 = FakePX4(connected=False)
        assert px4.disarm_vehicle() is False

    def test_already_disarmed_returns_true(self):
        px4 = FakePX4(armed=False)
        assert px4.disarm_vehicle() is True

    def test_successful_disarm(self):
        px4 = FakePX4(armed=True)
        result = MagicMock(success=True)
        px4.arming_client.call_async.return_value = _future_with_result(result)
        with patch("mission_controller.px4_setters.rclpy.spin_once"), \
             patch("mission_controller.px4_setters.time.sleep"):
            assert px4.disarm_vehicle() is True

    def test_failed_disarm_returns_false(self):
        px4 = FakePX4(armed=True)
        result = MagicMock(success=False)
        px4.arming_client.call_async.return_value = _future_with_result(result)
        with patch("mission_controller.px4_setters.rclpy.spin_once"), \
             patch("mission_controller.px4_setters.time.sleep"):
            assert px4.disarm_vehicle() is False


# ---------------------------------------------------------------------------
# change_mode
# ---------------------------------------------------------------------------
class TestChangeMode:
    def test_not_connected_returns_false(self):
        px4 = FakePX4(connected=False)
        assert px4.change_mode("GUIDED") is False

    def test_successful_mode_change(self):
        px4 = FakePX4()
        result = MagicMock(mode_sent=True)
        px4.set_mode_client.call_async.return_value = _future_with_result(result)
        with patch("mission_controller.px4_setters.rclpy.spin_once"), \
             patch("mission_controller.px4_setters.time.sleep"):
            assert px4.change_mode("OFFBOARD") is True

    def test_failed_mode_change(self):
        px4 = FakePX4()
        result = MagicMock(mode_sent=False)
        px4.set_mode_client.call_async.return_value = _future_with_result(result)
        with patch("mission_controller.px4_setters.rclpy.spin_once"), \
             patch("mission_controller.px4_setters.time.sleep"):
            assert px4.change_mode("GUIDED") is False

    def test_mode_change_exception(self):
        px4 = FakePX4()
        px4.set_mode_client.call_async.side_effect = RuntimeError("x")
        assert px4.change_mode("GUIDED") is False


# ---------------------------------------------------------------------------
# takeoff / land
# ---------------------------------------------------------------------------
class TestTakeoff:
    def test_not_connected_returns_false(self):
        px4 = FakePX4(connected=False)
        assert px4.takeoff(10) is False

    def test_arm_failure_aborts(self):
        px4 = FakePX4(armed=False)
        # arming fails
        px4.arming_client.call_async.return_value = _future_with_result(
            MagicMock(success=False)
        )
        with patch("mission_controller.px4_setters.rclpy.spin_once"), \
             patch("mission_controller.px4_setters.time.sleep"):
            assert px4.takeoff(10) is False

    def test_mode_change_failure_aborts(self):
        px4 = FakePX4(armed=True)
        px4.set_mode_client.call_async.return_value = _future_with_result(
            MagicMock(mode_sent=False)
        )
        with patch("mission_controller.px4_setters.rclpy.spin_once"), \
             patch("mission_controller.px4_setters.time.sleep"):
            assert px4.takeoff(10) is False

    def test_reaches_target_altitude(self):
        px4 = FakePX4(armed=True)
        px4.set_mode_client.call_async.return_value = _future_with_result(
            MagicMock(mode_sent=True)
        )
        px4.takeoff_client.call_async.return_value = _future_pending()
        # altitude already at target
        px4.current_position.pose.position.z = 10.0
        with patch("mission_controller.px4_setters.rclpy.spin_once"), \
             patch("mission_controller.px4_setters.time.sleep"):
            assert px4.takeoff(10) is True


class TestLand:
    def test_not_connected_returns_false(self):
        px4 = FakePX4(connected=False)
        assert px4.land() is False

    def test_lands_when_near_ground(self):
        px4 = FakePX4()
        px4.land_client.call_async.return_value = _future_pending()
        px4.current_position.pose.position.z = 0.05
        with patch("mission_controller.px4_setters.rclpy.spin_once"), \
             patch("mission_controller.px4_setters.time.sleep"):
            assert px4.land() is True

    def test_landing_exception_returns_false(self):
        px4 = FakePX4()
        px4.land_client.call_async.side_effect = RuntimeError("nope")
        assert px4.land() is False


# ---------------------------------------------------------------------------
# Setpoint publishers
# ---------------------------------------------------------------------------
class TestSendPositionSetpoint:
    def test_publishes_and_returns_true(self):
        px4 = FakePX4()
        assert px4.send_position_setpoint(1.0, 2.0, 3.0) is True
        px4.setpoint_pub.publish.assert_called_once()

    def test_publish_exception_returns_false(self):
        px4 = FakePX4()
        px4.setpoint_pub.publish.side_effect = RuntimeError("broken")
        assert px4.send_position_setpoint(1, 2, 3) is False


class TestSendVelocitySetpoint:
    def test_publishes_and_returns_true(self):
        px4 = FakePX4()
        assert px4.send_velocity_setpoint(0.5, 0.0, 0.0, 0.1) is True
        px4.velocity_setpoint_pub.publish.assert_called_once()

    def test_default_yaw_rate_zero(self):
        px4 = FakePX4()
        assert px4.send_velocity_setpoint(1.0, 0.0, 0.0) is True
        px4.velocity_setpoint_pub.publish.assert_called_once()

    def test_publish_exception_returns_false(self):
        px4 = FakePX4()
        px4.velocity_setpoint_pub.publish.side_effect = RuntimeError("x")
        assert px4.send_velocity_setpoint(0, 0, 0) is False


# ---------------------------------------------------------------------------
# High-level helpers
# ---------------------------------------------------------------------------
class TestHoldCurrentPosition:
    def test_returns_false_if_no_position(self):
        px4 = FakePX4()
        px4.current_position = None
        assert px4.hold_current_position() is False

    def test_publishes_current_pose(self):
        px4 = FakePX4()
        px4.current_position.pose.position.x = 4.0
        px4.current_position.pose.position.y = 5.0
        px4.current_position.pose.position.z = 6.0
        assert px4.hold_current_position() is True
        px4.setpoint_pub.publish.assert_called_once()


class TestStartOffboard:
    def test_not_connected_returns_false(self):
        px4 = FakePX4(connected=False)
        assert px4.start_offboard() is False

    def test_warms_up_and_switches_mode(self):
        px4 = FakePX4()
        px4.set_mode_client.call_async.return_value = _future_with_result(
            MagicMock(mode_sent=True)
        )
        with patch("mission_controller.px4_setters.rclpy.spin_once"), \
             patch("mission_controller.px4_setters.time.sleep"):
            assert px4.start_offboard(warmup_count=3) is True
        # Each warmup publishes a velocity setpoint
        assert px4.velocity_setpoint_pub.publish.call_count == 3


class TestSetRcChannel:
    def test_returns_false_not_implemented(self):
        px4 = FakePX4()
        assert px4.set_rc_channel(8, 2000) is False


# ---------------------------------------------------------------------------
# Movement helpers
# ---------------------------------------------------------------------------
class TestFlyForwardBackward:
    def test_fly_forward_publishes_velocity(self):
        px4 = FakePX4()
        with patch("mission_controller.px4_setters.rclpy.spin_once"), \
             patch("mission_controller.px4_setters.time.sleep"), \
             patch("mission_controller.px4_setters.time.time",
                   side_effect=[0.0, 0.0, 0.1, 1.0]):
            assert px4.fly_forward(1.5, 0.5) is True
        assert px4.velocity_setpoint_pub.publish.call_count >= 1

    def test_fly_backward_calls_fly_forward_with_negated_speed(self):
        px4 = FakePX4()
        with patch.object(px4, "fly_forward", return_value=True) as mock_fwd:
            assert px4.fly_backward(2.0, 1.0) is True
        mock_fwd.assert_called_once_with(-2.0, 1.0)


class TestMoveToOffset:
    def test_returns_false_without_position(self):
        px4 = FakePX4()
        px4.current_position = None
        assert px4.move_to_offset(1, 1, 1) is False

    def test_delegates_to_move_to_position(self):
        px4 = FakePX4()
        px4.current_position.pose.position.x = 5.0
        px4.current_position.pose.position.y = 10.0
        px4.current_position.pose.position.z = 2.0
        with patch.object(
            px4, "move_to_position", return_value=True
        ) as mock_move:
            assert px4.move_to_offset(1, -2, 0.5, timeout=15) is True
        mock_move.assert_called_once_with(6.0, 8.0, 2.5, 15)


class TestMoveToPosition:
    def test_returns_false_without_position(self):
        px4 = FakePX4()
        px4.current_position = None
        assert px4.move_to_position(0, 0, 0) is False

    def test_reaches_target_within_tolerance(self):
        px4 = FakePX4()
        # Drone already at the target within tolerance
        px4.current_position.pose.position.x = 0.0
        px4.current_position.pose.position.y = 0.0
        px4.current_position.pose.position.z = 0.0
        with patch("mission_controller.px4_setters.rclpy.spin_once"), \
             patch("mission_controller.px4_setters.time.sleep"):
            assert px4.move_to_position(0.0, 0.0, 0.0) is True


class TestHover:
    def test_publishes_zero_velocity_during_hover(self):
        px4 = FakePX4()
        with patch("mission_controller.px4_setters.rclpy.spin_once"), \
             patch("mission_controller.px4_setters.time.sleep"), \
             patch("mission_controller.px4_setters.time.time",
                   side_effect=[0.0, 0.0, 0.1, 5.0]):
            assert px4.hover(0.5) is True
        assert px4.velocity_setpoint_pub.publish.call_count >= 1
