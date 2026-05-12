#!/usr/bin/env python3
"""
GPS / Local Coordinate Movement Test WITH TAKEOFF

Goal:
    Test whether the drone can move to specific nearby coordinates from a script.

Important design note:
    The project/task name says "GPS coordinates", but for very short movements
    such as 0.5 m, 1.0 m, or 1.5 m, raw GPS latitude/longitude is usually too
    noisy to verify the movement accurately.

    Therefore this script commands MAVROS LOCAL POSITION coordinates:
        /mavros/local_position/pose

    This still uses PX4's estimator output, which may be based on GPS + IMU +
    barometer + other sensors. It is the better interface for short-distance
    movement tests.

What this script does:
    1. Boot PX4/MAVROS
    2. Connect to MAVROS
    3. Start OFFBOARD heartbeat stream
    4. Switch to OFFBOARD mode
    5. Arm the drone
    6. Take off to a small test altitude
    7. Save the takeoff hover position as the origin
    8. Move forward, return origin
    9. Move backward, return origin
    10. Move left, return origin
    11. Move right, return origin
    12. Land

Absolute command behavior:
    The script does NOT keep adding commands like:
        x = x + 1.0, x = x + 1.0, x = x + 1.0, ...

    Instead, each movement calculates one absolute local target and repeatedly
    sends the SAME target until the drone reaches it.

    Example:
        Current x = 2.0
        Move forward 1.0 m
        Target x = 3.0

        During the movement loop, the script keeps sending:
            go to x = 3.0

        If the drone has already moved 0.4 m, the remaining distance is 0.6 m,
        but the command is still the absolute target x = 3.0. This avoids
        accumulated movement commands and makes the test easier to reason about.

Safety limits:
    - Default movement distance is 1.0 m.
    - Maximum movement distance is limited to 1.5 m.
    - Default takeoff altitude is 1.0 m.
    - Maximum takeoff altitude is limited to 1.5 m.
    - After every direction test, the drone returns to the saved origin.

Usage examples:
    python3 tests/test_gps_coordinates.py --hardware --api
    python3 tests/test_gps_coordinates.py --hardware --distance 1.0 --takeoff-altitude 1.0
    python3 tests/test_gps_coordinates.py --sitl --api --distance 1.0
    python3 tests/test_gps_coordinates.py --port /dev/ttyUSB0 --api --distance 0.5

Notes:
    - Use --api if you want the script to arm through MAVROS.
    - Without --api, the script waits for manual arm through RC / QGroundControl.
    - This file assumes the hover/takeoff behavior has already been tested by
      the separate hover test, but this script still performs its own takeoff
      because this coordinate movement test should be runnable by itself.
"""

import argparse
import math
import os
import sys
import time

# Allow importing mission_controller when this file is placed inside tests/
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import rclpy

from mission_controller.px4_interface import boot_px4, init_px4, stop_px4


class GPSCoordinateMovementTest:
    """
    Simple movement tester using local position setpoints.

    The class name keeps "GPS" because that is the task wording, but the actual
    short-distance command interface is local x/y/z. This is intentional.
    """

    def __init__(
        self,
        px4,
        distance=1.0,
        takeoff_altitude=1.0,
        tolerance=0.20,
        max_distance=1.5,
        max_takeoff_altitude=1.5,
        verbose=True,
    ):
        self.px4 = px4
        self.distance = min(float(distance), float(max_distance))
        self.takeoff_altitude = min(float(takeoff_altitude), float(max_takeoff_altitude))
        self.tolerance = float(tolerance)
        self.max_distance = float(max_distance)
        self.max_takeoff_altitude = float(max_takeoff_altitude)
        self.verbose = verbose

    def log(self, message):
        """Print a test log message."""
        if self.verbose:
            print(message)

    # ------------------------------------------------------------------
    # Position helpers
    # ------------------------------------------------------------------

    def get_position(self):
        """
        Return current local position as a dictionary: {x, y, z}.

        This uses px4.get_location(), which reads MAVROS local position.
        """
        return self.px4.get_location()

    def wait_for_position(self, timeout=15.0):
        """Wait until MAVROS local position data becomes available."""
        self.log("[TEST] Waiting for local position data...")
        start = time.time()

        while time.time() - start < timeout:
            rclpy.spin_once(self.px4, timeout_sec=0.1)
            position = self.get_position()

            if position is not None:
                self.log(
                    "[TEST] ✓ Local position received: "
                    f"x={position['x']:.2f}, y={position['y']:.2f}, z={position['z']:.2f}"
                )
                return True

            time.sleep(0.1)

        self.log("[TEST] ✗ No local position data received")
        return False

    def distance_to_target(self, target):
        """Compute 3D distance from current local position to target."""
        current = self.get_position()
        if current is None:
            return None

        dx = target["x"] - current["x"]
        dy = target["y"] - current["y"]
        dz = target["z"] - current["z"]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    # ------------------------------------------------------------------
    # Takeoff / hover helpers
    # ------------------------------------------------------------------

    def arm_if_needed(self, use_api_arm=False, manual_arm_timeout=60):
        """
        Arm the drone.

        If use_api_arm is True, the script calls MAVROS arm service.
        If use_api_arm is False, the script waits for the pilot/operator to arm
        manually through RC or QGroundControl.
        """
        if self.px4.is_armed():
            self.log("[TEST] ✓ Drone is already armed")
            return True

        if use_api_arm:
            self.log("[TEST] Arming vehicle through MAVROS API...")
            return self.px4.arm_vehicle(timeout=20)

        self.log("[TEST] Waiting for manual arm through RC / QGroundControl...")
        return self.px4.wait_for_arm_with_heartbeat(
            timeout=manual_arm_timeout,
            heartbeat_rate=10,
        )

    def takeoff_and_hold(self):
        """
        Take off to the configured altitude and hold briefly.

        This script assumes the hover/takeoff behavior has already been tested
        separately, but the coordinate test still needs its own takeoff so that
        it can run as a full movement test.
        """
        self.log("\n" + "=" * 70)
        self.log(f"[TEST] Taking off to {self.takeoff_altitude:.2f} m")
        self.log("=" * 70)

        # Prefer the existing project interface. This keeps the test consistent
        # with the rest of the codebase and the hover test design.
        if not self.px4.takeoff(self.takeoff_altitude, timeout=45):
            self.log("[TEST] ✗ Takeoff failed")
            return False

        self.log("[TEST] ✓ Takeoff command completed")

        # Hold current position after takeoff so that the drone stabilizes
        # before the x/y movement tests begin.
        return self.hold_position(duration=3.0)

    def hold_position(self, duration=2.0, rate_hz=10.0):
        """Hold the current local position for a short stabilization period."""
        current = self.get_position()
        if current is None:
            self.log("[TEST] ✗ Cannot hold position because local position is unavailable")
            return False

        self.log(
            "[TEST] Holding position: "
            f"x={current['x']:.2f}, y={current['y']:.2f}, z={current['z']:.2f}"
        )

        dt = 1.0 / rate_hz
        start = time.time()

        while time.time() - start < duration:
            rclpy.spin_once(self.px4, timeout_sec=0.0)
            self.px4.send_position_setpoint(current["x"], current["y"], current["z"])
            time.sleep(dt)

        return True

    # ------------------------------------------------------------------
    # Absolute target command helper
    # ------------------------------------------------------------------

    def move_to_absolute_target(self, target, timeout=18.0, rate_hz=10.0):
        """
        Move to one ABSOLUTE local position target.

        Important:
            The target is fixed during this function.

        Example:
            target = {"x": 3.0, "y": 1.0, "z": 1.0}

        The script keeps sending exactly that target until the drone reaches it.
        It does not add extra distance on every loop iteration.
        """
        self.log(
            "[TEST] Absolute target: "
            f"x={target['x']:.2f}, y={target['y']:.2f}, z={target['z']:.2f}"
        )

        dt = 1.0 / rate_hz
        start = time.time()
        last_print = 0.0

        while time.time() - start < timeout:
            rclpy.spin_once(self.px4, timeout_sec=0.0)

            remaining = self.distance_to_target(target)
            if remaining is None:
                self.log("[TEST] Waiting for position update...")
                time.sleep(dt)
                continue

            # Stop condition: close enough to the target.
            if remaining <= self.tolerance:
                self.log(f"[TEST] ✓ Target reached, remaining distance={remaining:.2f} m")
                return True

            # Send the same absolute target again. This is NOT a queue of old
            # commands; it is just refreshing the newest desired position.
            self.px4.send_position_setpoint(target["x"], target["y"], target["z"])

            # Print progress once per second so the operator can see what is happening.
            now = time.time()
            if now - last_print >= 1.0:
                current = self.get_position()
                self.log(
                    "[TEST] Moving... "
                    f"current=({current['x']:.2f}, {current['y']:.2f}, {current['z']:.2f}), "
                    f"remaining={remaining:.2f} m"
                )
                last_print = now

            time.sleep(dt)

        self.log("[TEST] ✗ Timeout before reaching target")
        return False

    # ------------------------------------------------------------------
    # Relative movement helper
    # ------------------------------------------------------------------

    def move_relative(self, name, dx=0.0, dy=0.0, dz=0.0):
        """
        Move by a small relative offset, but send an absolute local target.

        This is the key logic for the test.

        Example:
            current x = 2.0
            dx = 1.0
            target x = 3.0

        The script sends target x = 3.0 repeatedly until the drone reaches it.
        """
        current = self.get_position()
        if current is None:
            self.log(f"[TEST] ✗ Cannot move {name}: current position unavailable")
            return False

        requested_distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        if requested_distance > self.max_distance:
            self.log(
                f"[TEST] ✗ Refusing {name}: requested distance "
                f"{requested_distance:.2f} m exceeds max {self.max_distance:.2f} m"
            )
            return False

        target = {
            "x": current["x"] + dx,
            "y": current["y"] + dy,
            "z": current["z"] + dz,
        }

        self.log("\n" + "=" * 70)
        self.log(f"[TEST] Moving {name}")
        self.log(
            "[TEST] Start:  "
            f"x={current['x']:.2f}, y={current['y']:.2f}, z={current['z']:.2f}"
        )
        self.log(
            "[TEST] Offset: "
            f"dx={dx:.2f}, dy={dy:.2f}, dz={dz:.2f}"
        )

        return self.move_to_absolute_target(target)

    # ------------------------------------------------------------------
    # Direction-specific movement functions
    # ------------------------------------------------------------------

    def move_forward(self):
        """Move forward in +x direction of the local frame."""
        return self.move_relative("FORWARD (+x)", dx=self.distance)

    def move_backward(self):
        """Move backward in -x direction of the local frame."""
        return self.move_relative("BACKWARD (-x)", dx=-self.distance)

    def move_left(self):
        """Move left in +y direction of the local frame."""
        return self.move_relative("LEFT (+y)", dy=self.distance)

    def move_right(self):
        """Move right in -y direction of the local frame."""
        return self.move_relative("RIGHT (-y)", dy=-self.distance)

    def return_to_origin(self, origin):
        """Return to the saved origin after each direction test."""
        self.log("\n[TEST] Returning to original start position")
        return self.move_to_absolute_target(origin)

    # ------------------------------------------------------------------
    # Main test sequence
    # ------------------------------------------------------------------

    def run_test_sequence(self):
        """Run takeoff, four direction tests, return origin, and land."""
        self.log("\n" + "=" * 70)
        self.log("[TEST] GPS / LOCAL COORDINATE MOVEMENT TEST WITH TAKEOFF")
        self.log("=" * 70)
        self.log(f"[TEST] Takeoff altitude: {self.takeoff_altitude:.2f} m")
        self.log(f"[TEST] Movement distance: {self.distance:.2f} m")
        self.log(f"[TEST] Position tolerance: {self.tolerance:.2f} m")

        if not self.wait_for_position():
            return False

        if not self.takeoff_and_hold():
            return False

        # Save the local position after takeoff. This is the origin for all
        # movement tests. Returning here after each test prevents the drone from
        # drifting too far away from the start area.
        origin = self.get_position()
        if origin is None:
            self.log("[TEST] ✗ Cannot save origin because position is unavailable")
            return False

        self.log(
            "[TEST] Origin saved after takeoff: "
            f"x={origin['x']:.2f}, y={origin['y']:.2f}, z={origin['z']:.2f}"
        )

        # Each movement is followed by a return to origin.
        tests = [
            self.move_forward,
            lambda: self.return_to_origin(origin),
            self.move_backward,
            lambda: self.return_to_origin(origin),
            self.move_left,
            lambda: self.return_to_origin(origin),
            self.move_right,
            lambda: self.return_to_origin(origin),
        ]

        for test_func in tests:
            if not test_func():
                self.log("[TEST] ✗ Movement sequence failed")
                return False

            # Stabilize after every movement before the next command.
            if not self.hold_position(duration=1.5):
                return False

        self.log("\n" + "=" * 70)
        self.log("[TEST] ✓ ALL COORDINATE MOVEMENT TESTS PASSED")
        self.log("=" * 70)
        return True


def build_fcu_url(args):
    """Build the MAVROS FCU URL from command-line arguments."""
    if args.sitl:
        print("[MAIN] Using SITL connection")
        return "udp://127.0.0.1:14540"

    if args.hardware:
        print("[MAIN] Using hardware on /dev/ttyUSB0")
        return "serial:///dev/ttyUSB0:921600"

    if args.port:
        print(f"[MAIN] Using custom hardware port: {args.port}")
        return f"serial:///{args.port}:921600"

    print("[MAIN] Using hardware default port: /dev/ttyTHS1")
    return "serial:///dev/ttyTHS1:921600"


def main():
    parser = argparse.ArgumentParser(
        description="Test drone movement to nearby local coordinates with takeoff",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument("--sitl", action="store_true", help="Use SITL simulation")
    parser.add_argument("--hardware", action="store_true", help="Use real hardware on /dev/ttyUSB0")
    parser.add_argument("--port", type=str, default=None, help="Custom hardware serial port")
    parser.add_argument("--api", action="store_true", help="Arm using MAVROS API instead of manual RC arm")
    parser.add_argument("--distance", type=float, default=1.0, help="Movement distance in meters, max 1.5")
    parser.add_argument("--takeoff-altitude", type=float, default=1.0, help="Takeoff altitude in meters, max 1.5")
    parser.add_argument("--tolerance", type=float, default=0.20, help="Target tolerance in meters")

    args = parser.parse_args()

    # Enforce small test limits. These are intentionally conservative because
    # this is a first coordinate movement validation script.
    if args.distance > 1.5:
        print("[MAIN] Requested distance is above 1.5 m. Limiting to 1.5 m for safety.")
        args.distance = 1.5

    if args.takeoff_altitude > 1.5:
        print("[MAIN] Requested takeoff altitude is above 1.5 m. Limiting to 1.5 m for safety.")
        args.takeoff_altitude = 1.5

    fcu_url = build_fcu_url(args)
    px4 = None
    success = False

    rclpy.init()

    try:
        print("[MAIN] Booting MAVROS / PX4 bridge...")
        boot_px4(fcu_url=fcu_url)

        # Give MAVROS a little time to create topics/services.
        print("[MAIN] Waiting 10s for MAVROS initialization...")
        time.sleep(10)

        print("[MAIN] Initializing PX4 interface...")
        px4 = init_px4(namespace="mavros")

        if not px4.connected:
            print("[MAIN] ✗ Failed to connect to MAVROS")
            return False

        print("[MAIN] ✓ Connected to MAVROS")

        # Start a heartbeat/setpoint stream before OFFBOARD. PX4 expects
        # continuous setpoint messages before and during OFFBOARD mode.
        print("[MAIN] Starting OFFBOARD heartbeat stream...")
        if not px4.start_offboard_stream_background(rate_hz=10):
            print("[MAIN] ✗ Failed to start OFFBOARD heartbeat stream")
            return False

        time.sleep(1.5)

        print("[MAIN] Switching to OFFBOARD mode...")
        if not px4.start_offboard():
            print("[MAIN] ✗ Failed to enter OFFBOARD mode")
            return False

        tester = GPSCoordinateMovementTest(
            px4=px4,
            distance=args.distance,
            takeoff_altitude=args.takeoff_altitude,
            tolerance=args.tolerance,
        )

        if not tester.arm_if_needed(use_api_arm=args.api):
            print("[MAIN] ✗ Arming failed or timed out")
            return False

        success = tester.run_test_sequence()

        if success:
            print("\n[MAIN] ✓ Coordinate movement test passed")
        else:
            print("\n[MAIN] ✗ Coordinate movement test failed")

        return success

    except KeyboardInterrupt:
        print("\n[MAIN] Test interrupted by user")
        return False

    except Exception as e:
        print(f"\n[MAIN] Test failed with exception: {e}")
        import traceback
        traceback.print_exc()
        return False

    finally:
        print("[MAIN] Cleaning up...")

        # If the test reached the flight phase, always try to land before
        # stopping the heartbeat stream. This keeps the cleanup safer.
        if px4 is not None:
            try:
                if px4.connected and px4.is_armed():
                    print("[MAIN] Landing...")
                    px4.land(timeout=60)
            except Exception as e:
                print(f"[MAIN] Landing cleanup warning: {e}")

            try:
                px4.stop_offboard_stream_background()
            except Exception:
                pass

            try:
                px4.disconnect()
            except Exception:
                pass

        try:
            stop_px4()
        except Exception:
            pass

        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    ok = main()
    sys.exit(0 if ok else 1)

