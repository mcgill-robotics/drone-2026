#!/usr/bin/env python3
"""
GPS Lap Test in PX4 OFFBOARD mode.

Safe behavior:
- Take off to TAKEOFF_ALTITUDE_M
- Record the actual hover altitude
- Follow GPS waypoints while holding that altitude
- Use velocity control for smoother movement
- RTL after all waypoints or on failure
"""

import sys
import os
import math
import time
import argparse

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import rclpy
from mission_controller.px4_interface import init_px4, boot_px4, stop_px4


# ============================================================
# EDIT WAYPOINTS HERE
# Format: (latitude, longitude)
# Do NOT put altitude here for now. Altitude is held automatically.
# ============================================================
LAP_WAYPOINTS = [
    (45.504800, -73.577200),
    (45.504850, -73.577250),
    (45.504900, -73.577200),
    (45.504850, -73.577150),
]


EARTH_RADIUS_M = 6_378_137.0

TAKEOFF_ALTITUDE_M = 5.0
ARRIVAL_TOLERANCE_M = 2.0
ALTITUDE_TOLERANCE_M = 0.8
WAYPOINT_TIMEOUT_S = 90
POST_RTL_HOLD_S = 5

CONTROL_RATE_HZ = 10

MAX_XY_SPEED_MPS = 1.0
MIN_XY_SPEED_MPS = 0.15
MAX_Z_SPEED_MPS = 0.4
SLOWDOWN_RADIUS_M = 6.0


def clamp(value, low, high):
    return max(low, min(high, value))


def gps_to_local_offset(origin_lat, origin_lon, target_lat, target_lon):
    """
    Convert GPS difference to local ENU offset.
    Returns: east, north in meters.
    """
    origin_lat_rad = math.radians(origin_lat)
    d_lat = math.radians(target_lat - origin_lat)
    d_lon = math.radians(target_lon - origin_lon)

    north = d_lat * EARTH_RADIUS_M
    east = d_lon * EARTH_RADIUS_M * math.cos(origin_lat_rad)

    return east, north


class LapTest:
    def __init__(self, px4, waypoints, takeoff_altitude_m=TAKEOFF_ALTITUDE_M):
        self.px4 = px4
        self.waypoints = waypoints
        self.takeoff_altitude_m = float(takeoff_altitude_m)
        self.mission_altitude_m = None

    def log(self, msg):
        print(msg)

    def resolve_gps_waypoint_to_local(self, target_lat, target_lon):
        """
        Convert target GPS waypoint into local ENU target.
        Uses current GPS/current local position as anchor.
        """
        current_gps = self.px4.get_gps_location()
        current_pos = self.px4.get_location()

        if not current_gps or not current_pos:
            self.log("[TEST] Missing GPS or local position")
            return None

        east, north = gps_to_local_offset(
            current_gps["latitude"],
            current_gps["longitude"],
            target_lat,
            target_lon,
        )

        target_x = current_pos["x"] + east
        target_y = current_pos["y"] + north
        target_z = self.mission_altitude_m

        self.log(f"[TEST] GPS target: lat={target_lat:.7f}, lon={target_lon:.7f}")
        self.log(f"[TEST] Offset: east={east:.2f}m, north={north:.2f}m")
        self.log(
            f"[TEST] Local target: x={target_x:.2f}, "
            f"y={target_y:.2f}, z={target_z:.2f}"
        )

        return target_x, target_y, target_z

    def go_to_local_smooth(self, target_x, target_y, target_z, waypoint_index):
        """
        Move to one local target using velocity setpoints.
        Far from target: faster.
        Near target: slower.
        At target: stop.
        """
        dt = 1.0 / CONTROL_RATE_HZ
        start = time.time()
        last_log = 0.0

        while time.time() - start < WAYPOINT_TIMEOUT_S:
            pos = self.px4.get_location()
            if not pos:
                rclpy.spin_once(self.px4, timeout_sec=0.02)
                time.sleep(dt)
                continue

            dx = target_x - pos["x"]
            dy = target_y - pos["y"]
            dz = target_z - pos["z"]

            xy_dist = math.sqrt(dx * dx + dy * dy)
            alt_error = abs(dz)

            if xy_dist <= ARRIVAL_TOLERANCE_M and alt_error <= ALTITUDE_TOLERANCE_M:
                self.px4.send_velocity_setpoint(0.0, 0.0, 0.0, 0.0)
                self.log(f"[TEST] ✓ Reached waypoint {waypoint_index}")
                return True

            # Smooth XY speed: slows down near target.
            xy_speed = MAX_XY_SPEED_MPS * (xy_dist / SLOWDOWN_RADIUS_M)
            xy_speed = clamp(xy_speed, MIN_XY_SPEED_MPS, MAX_XY_SPEED_MPS)

            if xy_dist > 0.05:
                vx = xy_speed * dx / xy_dist
                vy = xy_speed * dy / xy_dist
            else:
                vx = 0.0
                vy = 0.0

            # Hold altitude gently.
            vz = clamp(0.5 * dz, -MAX_Z_SPEED_MPS, MAX_Z_SPEED_MPS)

            self.px4.send_velocity_setpoint(vx, vy, vz, 0.0)
            rclpy.spin_once(self.px4, timeout_sec=0.02)

            now = time.time()
            if now - last_log >= 1.0:
                self.log(
                    f"[TEST] WP{waypoint_index}: xy={xy_dist:.2f}m, "
                    f"alt_err={alt_error:.2f}m, "
                    f"vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}"
                )
                last_log = now

            time.sleep(dt)

        self.px4.send_velocity_setpoint(0.0, 0.0, 0.0, 0.0)
        self.log(f"[TEST] ✗ Timeout at waypoint {waypoint_index}")
        return False

    def go_to_gps_waypoint(self, waypoint_index, lat, lon):
        self.log("\n" + "-" * 60)
        self.log(f"[TEST] Going to waypoint {waypoint_index}")
        self.log("-" * 60)

        target = self.resolve_gps_waypoint_to_local(lat, lon)
        if target is None:
            return False

        return self.go_to_local_smooth(*target, waypoint_index=waypoint_index)

    def rtl(self):
        self.log("[TEST] Switching to RTL...")
        ok = self.px4.change_mode("RTL")

        end_time = time.time() + POST_RTL_HOLD_S
        while time.time() < end_time:
            rclpy.spin_once(self.px4, timeout_sec=0.1)

        return ok

    def run(self):
        self.log("\n" + "=" * 70)
        self.log("GPS LAP TEST")
        self.log("=" * 70)

        try:
            if not self.px4.connected:
                self.log("[TEST] Not connected to MAVROS")
                return False

            self.log("[TEST] Starting OFFBOARD...")
            if not self.px4.start_offboard():
                self.log("[TEST] Failed to enter OFFBOARD")
                return False

            self.log("[TEST] Starting heartbeat stream...")
            if not self.px4.start_offboard_stream_background(rate_hz=10):
                self.log("[TEST] Failed to start heartbeat stream")
                return False

            self.log("[TEST] Waiting for manual arm...")
            if not self.px4.wait_for_arm_with_heartbeat(timeout=60, heartbeat_rate=10):
                self.log("[TEST] Arm timeout")
                return False

            self.log(f"[TEST] Taking off to {self.takeoff_altitude_m:.1f}m...")
            if not self.px4.takeoff(altitude=self.takeoff_altitude_m, timeout=60):
                self.log("[TEST] Takeoff failed")
                return False

            time.sleep(2)

            hover_pos = self.px4.get_location()
            if not hover_pos:
                self.log("[TEST] Cannot read post-takeoff position")
                return False

            self.mission_altitude_m = hover_pos["z"]
            self.log(f"[TEST] Holding mission altitude: {self.mission_altitude_m:.2f}m")

            for i, (lat, lon) in enumerate(self.waypoints, start=1):
                if not self.go_to_gps_waypoint(i, lat, lon):
                    self.log("[TEST] Waypoint failed. RTL for safety.")
                    self.rtl()
                    return False

            self.log("\n[TEST] All waypoints complete.")
            return self.rtl()

        except Exception as e:
            self.log(f"[TEST] Exception: {e}")
            import traceback
            traceback.print_exc()

            try:
                self.rtl()
            except Exception:
                pass

            return False

        finally:
            self.log("[TEST] Cleaning up...")
            try:
                self.px4.stop_offboard_stream_background()
            except Exception:
                pass
            try:
                self.px4.disconnect()
            except Exception:
                pass


def main():
    parser = argparse.ArgumentParser(description="GPS lap test using smooth velocity control")
    parser.add_argument("--sitl", action="store_true", help="Use SITL")
    parser.add_argument("--hardware", action="store_true", help="Use hardware on /dev/ttyUSB0")
    parser.add_argument("--port", type=str, default=None, help="Custom serial port")
    parser.add_argument("--alt", type=float, default=TAKEOFF_ALTITUDE_M, help="Takeoff altitude in meters")
    args = parser.parse_args()

    sitl = args.sitl
    port = args.port

    if args.hardware:
        port = "/dev/ttyUSB0"
        sitl = False

    rclpy.init()

    try:
        if sitl:
            fcu_url = "udp://127.0.0.1:14540"
            print("[MAIN] Using SITL")
        else:
            fcu_url = f"serial:///{port}:921600" if port else "serial:///dev/ttyTHS1:921600"
            print(f"[MAIN] Using hardware: {fcu_url}")

        print("[MAIN] Booting PX4/MAVROS...")
        boot_px4(fcu_url=fcu_url)

        print("[MAIN] Waiting for MAVROS initialization...")
        time.sleep(10)

        px4 = init_px4()
        if not px4.connected:
            print("[MAIN] Failed to connect to MAVROS")
            return False

        tester = LapTest(
            px4=px4,
            waypoints=LAP_WAYPOINTS,
            takeoff_altitude_m=args.alt,
        )

        return tester.run()

    except KeyboardInterrupt:
        print("\n[MAIN] Interrupted by user")
        return False

    finally:
        print("[MAIN] Shutting down...")
        try:
            stop_px4()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)