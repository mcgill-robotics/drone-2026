#!/usr/bin/env python3
"""Mission 2 PX4 movement helpers for building perimeter inspection."""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

# Import GPS coordinate conversion utilities from px4_setters
from mission_controller.px4_setters import gps_to_local_offset, local_offset_to_gps

ARRIVAL_TOLERANCE_M = 0.5
ALTITUDE_TOLERANCE_M = 0.75
SETPOINT_RATE_HZ = 10
MOTION_LIMIT_PARAMS = (
    ("MPC_XY_VEL_MAX", 1.5),  # Conservative for wall following
    ("MPC_XY_CRUISE", 0.8),
    ("MPC_VEL_MANUAL", 1.5),
    ("MPC_LAND_SPEED", 0.4),
    ("MPC_LAND_CRWL", 0.2),
)

_px4 = None
_launch_alt = None
_hover_alt = None


def fcu_url(*, sitl: bool = False, port: str | None = None) -> str:
    """Resolve FCU connection URL for SITL or hardware."""
    if sitl:
        return "udp://:14540@localhost:14580"
    return f"serial:///{port}:921600" if port else "serial:///dev/ttyTHS1:921600"


def setup_environment(*, sitl: bool = False, port: str | None = None, boot: bool = True, wait_seconds: float = 10.0) -> bool:
    """Boot MAVROS/PX4 once. Phase scripts connect separately."""
    if not boot:
        return True

    from mission_controller.px4_interface import boot_px4

    url = fcu_url(sitl=sitl, port=port)
    print(f"[MISSION2 SETUP] Booting PX4/MAVROS with {url}")
    if boot_px4(fcu_url=url) is None:
        return False
    if wait_seconds > 0:
        print(f"[MISSION2 SETUP] Waiting {wait_seconds:g}s for MAVROS initialization")
        time.sleep(wait_seconds)
    return True


def connect() -> Any:
    """Get or create global PX4 connection."""
    global _px4

    if _px4 is not None:
        return _px4

    from mission_controller.px4_interface import init_px4

    print("[MISSION2 MOVE] Initializing PX4Interface")
    _px4 = init_px4()
    if not _px4.connected:
        raise RuntimeError("Failed to connect to MAVROS")
    return _px4


def configure_motion_limits(px4: Any) -> bool:
    """Set conservative PX4 motion limits for precise wall following."""
    print("[MISSION2 MOVE] Setting conservative motion limits for wall following")
    for name, value in MOTION_LIMIT_PARAMS:
        if not px4.set_param(name, value):
            print(f"[MISSION2 MOVE] Failed to set {name}")
            continue
    return True


def begin_phase(*, takeoff_altitude: float = 5.0, arm_timeout: float = 60.0) -> bool:
    """
    Prepare OFFBOARD movement and take off for building perimeter inspection.
    
    Args:
        takeoff_altitude: Altitude in meters relative to ground
        arm_timeout: Maximum time to wait for manual arm
    
    Returns:
        True if successfully in OFFBOARD mode at takeoff altitude, False otherwise
    """
    global _launch_alt, _hover_alt

    px4 = connect()
    if not configure_motion_limits(px4):
        return False

    print("[MISSION2 MOVE] Switching to OFFBOARD")
    if not px4.start_offboard():
        return False

    print("[MISSION2 MOVE] Starting background setpoint stream")
    if not px4.start_offboard_stream_background():
        return False

    print("[MISSION2 MOVE] Waiting for manual arm")
    if not px4.wait_for_arm_with_heartbeat(timeout=arm_timeout, heartbeat_rate=SETPOINT_RATE_HZ):
        return False

    launch_pos = px4.get_location()
    if not launch_pos:
        print("[MISSION2 MOVE] Cannot get launch position")
        return False
    _launch_alt = launch_pos["z"]

    print(f"[MISSION2 MOVE] Taking off to {takeoff_altitude:g}m")
    if not px4.takeoff(altitude=takeoff_altitude, timeout=30):
        return False

    hover_pos = px4.get_location()
    if not hover_pos:
        print("[MISSION2 MOVE] Cannot get hover position")
        return False
    _hover_alt = hover_pos["z"]
    return True


def navigate_to_coordinate(lat: float, lon: float, alt_agl: float | None = None, timeout: float = 60.0) -> bool:
    """
    Fly to a GPS coordinate using position setpoints.
    
    Args:
        lat, lon: Target GPS coordinates
        alt_agl: Altitude above ground (relative). If None, uses current hover altitude
        timeout: Maximum time to reach target in seconds
    
    Returns:
        True if target reached, False on timeout or error
    """
    px4 = connect()
    
    import rclpy
    
    current_pos = px4.get_location()
    current_gps = px4.get_gps_location()
    if not current_pos or not current_gps:
        print("[MISSION2 MOVE] Cannot get current position")
        return False

    # Convert GPS target to local coordinates
    east, north = gps_to_local_offset(
        current_gps["latitude"],
        current_gps["longitude"],
        float(lat),
        float(lon),
    )
    target_x = current_pos["x"] + east
    target_y = current_pos["y"] + north
    target_alt = _hover_alt if alt_agl is None else (_launch_alt or current_pos["z"]) + float(alt_agl)
    if target_alt is None:
        target_alt = current_pos["z"]

    print(
        f"[MISSION2 MOVE] Navigating to GPS ({lat:.7f}, {lon:.7f}); "
        f"local offset ({east:.2f}, {north:.2f})m; target ({target_x:.2f}, {target_y:.2f}, {target_alt:.2f})"
    )

    dt = 1.0 / SETPOINT_RATE_HZ
    start_time = time.time()
    last_log = 0.0
    
    while (time.time() - start_time) < timeout:
        current_pos = px4.get_location()
        if not current_pos:
            time.sleep(dt)
            continue

        rclpy.spin_once(px4, timeout_sec=0.0)

        horizontal_distance = math.hypot(current_pos["x"] - target_x, current_pos["y"] - target_y)
        altitude_error = abs(current_pos["z"] - target_alt)
        
        # Send position setpoint
        px4.send_position_setpoint(target_x, target_y, target_alt, yaw=px4._mission_yaw, yaw_from_direction=False)

        now = time.time()
        if now - last_log >= 1.0:
            print(f"[MISSION2 MOVE] Distance: XY={horizontal_distance:.2f}m, alt error={altitude_error:.2f}m")
            last_log = now

        if horizontal_distance <= ARRIVAL_TOLERANCE_M and altitude_error <= ALTITUDE_TOLERANCE_M:
            print(f"[MISSION2 MOVE] Reached target coordinate")
            return True
        
        time.sleep(dt)

    print("[MISSION2 MOVE] Timeout reaching target coordinate")
    return False


def send_velocity_command(vx: float, vy: float, vz: float = 0.0, yaw_rate: float = 0.0) -> bool:
    """
    Send velocity setpoint command (for wall-following strafing).
    
    Args:
        vx: Forward velocity (m/s) in body frame
        vy: Right velocity (m/s) in body frame (positive = strafe right)
        vz: Vertical velocity (m/s) (positive = up)
        yaw_rate: Yaw rate (rad/s)
    
    Returns:
        True if command sent successfully
    """
    px4 = connect()
    return px4.send_velocity_setpoint(float(vx), float(vy), float(vz), float(yaw_rate))


def hold_position() -> bool:
    """Hold current position (zero velocity setpoint)."""
    px4 = connect()
    return px4.hold_current_position()


def end_phase(*, land: bool = True) -> bool:
    """End mission phase: land and stop offboard stream."""
    px4 = connect()
    ok = True
    if land:
        print("[MISSION2 MOVE] Landing")
        ok = px4.land(timeout=60)
    px4.stop_offboard_stream_background()
    return ok


def cleanup(*, stop_px4_process: bool = False) -> None:
    """Cleanup: disconnect from MAVROS and optionally stop PX4 process."""
    global _px4

    if _px4 is not None:
        try:
            _px4.stop_offboard_stream_background()
        except Exception:
            pass
        try:
            _px4.disconnect()
        except Exception:
            pass
        _px4 = None

    if stop_px4_process:
        try:
            from mission_controller.px4_interface import stop_px4
            stop_px4()
        except Exception:
            pass

    try:
        import rclpy
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass
