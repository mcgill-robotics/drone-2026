#!/usr/bin/env python3
"""Mission 1 PX4 movement helpers."""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path
from typing import Any, Callable


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

EARTH_RADIUS_M = 6_378_137.0
ARRIVAL_TOLERANCE_M = 2.0
ALTITUDE_TOLERANCE_M = 0.75
SETPOINT_RATE_HZ = 10
MOTION_LIMIT_PARAMS = (
    ("MPC_XY_VEL_MAX", 2.0),
    ("MPC_XY_CRUISE", 1.0),
    ("MPC_VEL_MANUAL", 2.0),
    ("MPC_LAND_SPEED", 0.4),
    ("MPC_LAND_CRWL", 0.2),
)

_px4 = None
_launch_alt = None
_hover_alt = None


def fcu_url(*, sitl: bool = False, port: str | None = None) -> str:
    if sitl:
        return "udp://127.0.0.1:14540"
    return f"serial:///{port}:921600" if port else "serial:///dev/ttyTHS1:921600"


def gps_to_local_offset(origin_lat: float, origin_lon: float, target_lat: float, target_lon: float) -> tuple[float, float]:
    origin_lat_rad = math.radians(origin_lat)
    north = math.radians(target_lat - origin_lat) * EARTH_RADIUS_M
    east = math.radians(target_lon - origin_lon) * EARTH_RADIUS_M * math.cos(origin_lat_rad)
    return east, north


def setup_environment(*, sitl: bool = False, port: str | None = None, boot: bool = True, wait_seconds: float = 10.0):
    """Boot MAVROS/PX4 once. Phase scripts connect separately."""
    if not boot:
        return True

    from mission_controller.px4_interface import boot_px4

    url = fcu_url(sitl=sitl, port=port)
    print(f"[MISSION1 SETUP] Booting PX4/MAVROS with {url}")
    if boot_px4(fcu_url=url) is None:
        return False
    if wait_seconds > 0:
        print(f"[MISSION1 SETUP] Waiting {wait_seconds:g}s for MAVROS initialization")
        time.sleep(wait_seconds)
    return True


def connect():
    global _px4

    if _px4 is not None:
        return _px4

    from mission_controller.px4_interface import init_px4

    print("[MISSION1 MOVE] Initializing PX4Interface")
    _px4 = init_px4()
    if not _px4.connected:
        raise RuntimeError("Failed to connect to MAVROS")
    return _px4


def current_gps_coordinate(timeout: float = 10.0) -> dict[str, float] | None:
    """Return the current GPS lat/lon after spinning briefly for telemetry."""
    px4 = connect()

    import rclpy

    start_time = time.time()
    while (time.time() - start_time) < timeout:
        gps = px4.get_gps_location()
        if gps is not None:
            return {"lat": float(gps["latitude"]), "lon": float(gps["longitude"])}
        rclpy.spin_once(px4, timeout_sec=0.1)
        time.sleep(0.1)

    return None


def configure_motion_limits(px4: Any) -> bool:
    print("[MISSION1 MOVE] Setting conservative PX4 motion limits")
    param_names = [name for name, _value in MOTION_LIMIT_PARAMS]
    if not px4.wait_for_params(param_names, timeout=30):
        print("[MISSION1 MOVE] MAVROS params not ready")
        return False

    for name, value in MOTION_LIMIT_PARAMS:
        if not px4.set_param(name, value):
            print(f"[MISSION1 MOVE] Failed to set {name}")
            return False
    return True


def begin_phase(*, takeoff_altitude: float = 5.0, arm_timeout: float = 60.0) -> bool:
    """Prepare OFFBOARD movement and take off for one mission phase."""
    global _launch_alt, _hover_alt

    px4 = connect()
    if not configure_motion_limits(px4):
        return False

    print("[MISSION1 MOVE] Switching to OFFBOARD")
    if not px4.start_offboard():
        return False

    print("[MISSION1 MOVE] Starting background setpoint stream")
    if not px4.start_offboard_stream_background():
        return False

    print("[MISSION1 MOVE] Waiting for manual arm")
    if not px4.wait_for_arm_with_heartbeat(timeout=arm_timeout, heartbeat_rate=SETPOINT_RATE_HZ):
        return False

    launch_pos = px4.get_location()
    if not launch_pos:
        print("[MISSION1 MOVE] Cannot get launch position")
        return False
    _launch_alt = launch_pos["z"]

    print(f"[MISSION1 MOVE] Taking off to {takeoff_altitude:g}m")
    if not px4.takeoff(altitude=takeoff_altitude, timeout=30):
        return False

    hover_pos = px4.get_location()
    if not hover_pos:
        print("[MISSION1 MOVE] Cannot get hover position")
        return False
    _hover_alt = hover_pos["z"]
    return True


def navigate_to_coordinate(
    lat: float,
    lon: float,
    alt_agl: float | None = None,
    timeout: float = 60.0,
    interrupt_check: Callable[[], str | None] | None = None,
) -> bool | str:
    """Fly to a GPS lat/lon while holding current hover altitude by default."""
    px4 = connect()
    current_pos = px4.get_location()
    current_gps = px4.get_gps_location()
    if not current_pos or not current_gps:
        print("[MISSION1 MOVE] Cannot get current local/GPS position")
        return False

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
        f"[MISSION1 MOVE] GPS target lat={float(lat):.7f}, lon={float(lon):.7f}; "
        f"offset E/N=({east:.2f}, {north:.2f})m; local=({target_x:.2f}, {target_y:.2f}, {target_alt:.2f})"
    )

    import rclpy

    dt = 1.0 / SETPOINT_RATE_HZ
    start_time = time.time()
    last_log = 0.0
    while (time.time() - start_time) < timeout:
        if interrupt_check is not None:
            action = interrupt_check()
            if action in {"p", "q"}:
                px4.hold_current_position()
                return "paused" if action == "p" else "stopped"

        current_pos = px4.get_location()
        if not current_pos:
            time.sleep(dt)
            continue

        horizontal_distance = math.hypot(current_pos["x"] - target_x, current_pos["y"] - target_y)
        altitude_error = abs(current_pos["z"] - target_alt)
        px4.send_position_setpoint(target_x, target_y, target_alt, yaw_from_direction=True)
        rclpy.spin_once(px4, timeout_sec=0.0)

        now = time.time()
        if now - last_log >= 1.0:
            print(f"[MISSION1 MOVE] Distance XY: {horizontal_distance:.2f}m, alt error: {altitude_error:.2f}m")
            last_log = now

        if horizontal_distance <= ARRIVAL_TOLERANCE_M and altitude_error <= ALTITUDE_TOLERANCE_M:
            return True
        time.sleep(dt)

    print("[MISSION1 MOVE] Timeout reaching GPS target")
    return False


def end_phase(*, land: bool = True) -> bool:
    px4 = connect()
    ok = True
    if land:
        print("[MISSION1 MOVE] Landing")
        ok = px4.land(timeout=60)
    px4.stop_offboard_stream_background()
    return ok


def cleanup(*, stop_px4_process: bool = False) -> None:
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
