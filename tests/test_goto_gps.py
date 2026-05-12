#!/usr/bin/env python3
"""
GPS Goto Test

Drives the drone to a target GPS coordinate using ABSOLUTE position
setpoints. The setpoint published every loop iteration always describes
the absolute target (not a delta from the current pose), so as the drone
travels the remaining distance shrinks naturally — when the drone is x
meters into a 5 m move, the next message still points at the same absolute
target, which is equivalent to commanding the remaining (5 - x) meters.

Usage:
    python3 test_goto_gps.py --lat 45.5048 --lon -73.5772 --alt 10
    python3 test_goto_gps.py --sitl --lat 47.3977 --lon 8.5456 --alt 10
    python3 test_goto_gps.py --api --lat ... --lon ... --alt 10
"""

import sys
import os
import math
import time
import argparse

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import rclpy
from mission_controller.px4_interface import init_px4, boot_px4, stop_px4


EARTH_RADIUS_M = 6_378_137.0
ARRIVAL_TOLERANCE_M = 1.0
SETPOINT_RATE_HZ = 10
GOTO_TIMEOUT_S = 120


def gps_to_local_offset(origin_lat, origin_lon, target_lat, target_lon):
    """
    Convert a target GPS coordinate to a local ENU offset (meters) from
    an origin GPS coordinate using an equirectangular approximation.
    Returns (east, north) in meters.
    """
    o_lat = math.radians(origin_lat)
    d_lat = math.radians(target_lat - origin_lat)
    d_lon = math.radians(target_lon - origin_lon)
    north = d_lat * EARTH_RADIUS_M
    east = d_lon * EARTH_RADIUS_M * math.cos(o_lat)
    return east, north


class GotoGpsTest:
    def __init__(self, px4, target_lat, target_lon, target_alt, manual_arm=True):
        self.px4 = px4
        self.target_lat = target_lat
        self.target_lon = target_lon
        self.target_alt = float(target_alt)
        self.manual_arm = manual_arm

    def _resolve_target_local(self):
        """Anchor the GPS target to the drone's current local frame."""
        gps = self.px4.get_gps_location()
        loc = self.px4.get_location()
        if gps is None or loc is None:
            print("[TEST] Missing GPS or local pose; cannot resolve target")
            return None

        east, north = gps_to_local_offset(
            gps["latitude"], gps["longitude"],
            self.target_lat, self.target_lon,
        )
        # MAVROS local frame is ENU (x=east, y=north, z=up).
        target_x = loc["x"] + east
        target_y = loc["y"] + north
        target_z = self.target_alt
        print(f"[TEST] Current GPS: ({gps['latitude']:.7f}, {gps['longitude']:.7f})")
        print(f"[TEST] Target  GPS: ({self.target_lat:.7f}, {self.target_lon:.7f})")
        print(f"[TEST] Offset (E,N): ({east:.2f} m, {north:.2f} m)")
        print(f"[TEST] Absolute local target: x={target_x:.2f}, y={target_y:.2f}, z={target_z:.2f}")
        return target_x, target_y, target_z

    def _distance_to(self, tx, ty, tz):
        loc = self.px4.get_location()
        if loc is None:
            return float("inf")
        return math.sqrt(
            (loc["x"] - tx) ** 2 + (loc["y"] - ty) ** 2 + (loc["z"] - tz) ** 2
        )

    def run(self):
        print("\n" + "=" * 70)
        print("GPS GOTO TEST (absolute setpoint streaming)")
        print("=" * 70 + "\n")

        try:
            if not self.px4.connected:
                print("[TEST] Not connected to MAVROS")
                return False

            # Take off first so the drone has altitude before the GPS move.
            print(f"[TEST] Taking off to {self.target_alt:.1f} m...")
            if not self.px4.takeoff(self.target_alt, timeout=60):
                print("[TEST] Takeoff failed")
                return False

            # Optional: arm via API if requested (takeoff() will arm anyway).
            if not self.manual_arm and not self.px4.is_armed():
                if not self.px4.arm_vehicle():
                    print("[TEST] Arm failed")
                    return False

            # Allow telemetry (GPS + local pose) to settle.
            print("[TEST] Waiting for telemetry to stabilize...")
            settle_start = time.time()
            while time.time() - settle_start < 5:
                rclpy.spin_once(self.px4, timeout_sec=0.1)
                if self.px4.get_gps_location() and self.px4.get_location():
                    break

            target = self._resolve_target_local()
            if target is None:
                return False
            tx, ty, tz = target

            # GUIDED accepts setpoint_position/local on ArduPilot. Mode is
            # already GUIDED from takeoff(), but make the intent explicit.
            if not self.px4.change_mode("GUIDED"):
                print("[TEST] GUIDED mode rejected")
                return False

            print(f"[TEST] Streaming absolute setpoints at {SETPOINT_RATE_HZ} Hz...")
            dt = 1.0 / SETPOINT_RATE_HZ
            start = time.time()
            last_log = 0.0

            while True:
                # The published setpoint is always the SAME absolute target.
                # As the drone closes in, the implicit remaining-distance
                # shrinks even though the message itself never changes.
                self.px4.send_position_setpoint(tx, ty, tz)
                rclpy.spin_once(self.px4, timeout_sec=0.0)

                dist = self._distance_to(tx, ty, tz)

                now = time.time()
                if now - last_log >= 1.0:
                    print(f"[TEST] Distance to target: {dist:.2f} m")
                    last_log = now

                if dist <= ARRIVAL_TOLERANCE_M:
                    print(f"[TEST] ✓ Arrived (within {ARRIVAL_TOLERANCE_M:.1f} m)")
                    break

                if now - start > GOTO_TIMEOUT_S:
                    print(f"[TEST] ✗ Timeout after {GOTO_TIMEOUT_S}s, dist={dist:.2f} m")
                    return False

                time.sleep(dt)

            # Hold the target briefly to confirm stability.
            hold_until = time.time() + 3.0
            while time.time() < hold_until:
                self.px4.send_position_setpoint(tx, ty, tz)
                rclpy.spin_once(self.px4, timeout_sec=0.0)
                time.sleep(dt)

            print("[TEST] Landing...")
            self.px4.land(timeout=60)

            print("\n" + "=" * 70)
            print("✓ GPS GOTO TEST COMPLETE")
            print("=" * 70 + "\n")
            return True

        except Exception as e:
            print(f"[TEST] Exception: {e}")
            import traceback
            traceback.print_exc()
            return False
        finally:
            try:
                self.px4.disconnect()
            except Exception:
                pass


def main():
    parser = argparse.ArgumentParser(description="Move the drone to a GPS coordinate")
    parser.add_argument("--lat", type=float, required=True, help="Target latitude (deg)")
    parser.add_argument("--lon", type=float, required=True, help="Target longitude (deg)")
    parser.add_argument("--alt", type=float, default=10.0, help="Target altitude AGL (m)")
    parser.add_argument("--sitl", action="store_true", help="Use SITL (udp://127.0.0.1:14540)")
    parser.add_argument("--hardware", action="store_true", help="Use /dev/ttyUSB0")
    parser.add_argument("--port", type=str, default=None, help="Custom serial port")
    parser.add_argument("--api", action="store_true", help="Arm via API instead of waiting for manual RC arm")

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
            print("[MAIN] Using SITL (UDP)")
        else:
            fcu_url = f"serial:///{port}:921600" if port else "serial:///dev/ttyTHS1:921600"
            print(f"[MAIN] Using hardware ({fcu_url})")

        boot_px4(fcu_url=fcu_url)
        print("[MAIN] Waiting 10s for MAVROS initialization...")
        time.sleep(10)

        px4 = init_px4()
        if not px4.connected:
            print("[MAIN] Failed to connect to MAVROS")
            return False

        tester = GotoGpsTest(
            px4,
            target_lat=args.lat,
            target_lon=args.lon,
            target_alt=args.alt,
            manual_arm=not args.api,
        )
        success = tester.run()
        print(f"\n[MAIN] {'✓ Test passed' if success else '✗ Test failed'}")
        return success

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
