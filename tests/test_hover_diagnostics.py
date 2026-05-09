#!/usr/bin/env python3
"""
Safe PX4 / MAVROS hover diagnostics script.

Purpose
-------
This script is a SAFE first step before writing any hover / takeoff logic.
It only reads telemetry and prints a compact status summary.

By default, it does NOT:
- arm the drone
- take off
- send velocity commands
- send position commands
- switch to OFFBOARD mode

Optional behavior
-----------------
If you pass --offboard, the script will start a zero-velocity OFFBOARD
heartbeat stream and switch to OFFBOARD mode. This still does NOT arm or fly.
Only use --offboard when the vehicle is in a safe test state and QGC shows
Ready to Fly.

Typical usage
-------------
Passive diagnostics only, safest:
    python3 tests/test_hover_diagnostics.py

Passive diagnostics with custom hardware port:
    python3 tests/test_hover_diagnostics.py --port /dev/ttyTHS1

OFFBOARD heartbeat diagnostics, no arming / no takeoff:
    python3 tests/test_hover_diagnostics.py --offboard

Run for 30 seconds and print every 1 second:
    python3 tests/test_hover_diagnostics.py --duration 30 --print-period 1.0
"""

import argparse
import csv
import os
import sys
import time
from datetime import datetime
from typing import Any, Dict, Optional

# Allow running this file directly from the tests/ directory.
# Example: python3 tests/test_hover_diagnostics.py
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import rclpy
from mission_controller.px4_interface import boot_px4, init_px4, stop_px4


# -----------------------------------------------------------------------------
# Small formatting helpers
# -----------------------------------------------------------------------------

def fmt(value: Any, digits: int = 2, default: str = "N/A") -> str:
    """
    Format numbers cleanly for terminal output.

    MAVROS sometimes returns None, NaN, or unavailable fields. This helper keeps
    the printout readable instead of crashing on missing data.
    """
    if value is None:
        return default
    try:
        return f"{float(value):.{digits}f}"
    except (TypeError, ValueError):
        return str(value)


def get_nested(data: Optional[Dict[str, Any]], *keys: str, default: Any = None) -> Any:
    """
    Safely read nested dictionary values.

    Example:
        get_nested(snapshot, "pose", "position", "z")
    """
    current = data
    for key in keys:
        if not isinstance(current, dict) or key not in current:
            return default
        current = current[key]
    return current


def landed_state_to_text(landed: Any) -> str:
    """
    Convert the simplified landed boolean from PX4Getters into readable text.

    In this codebase, px4.is_landed() returns True when MAVROS ExtendedState
    landed_state == 1. For this diagnostic script, that is enough.
    """
    if landed is True:
        return "LANDED"
    if landed is False:
        return "NOT_LANDED_OR_UNKNOWN"
    return "UNKNOWN"


# -----------------------------------------------------------------------------
# Main diagnostics class
# -----------------------------------------------------------------------------

class HoverDiagnostics:
    """
    Reads and prints telemetry that matters before hover testing.

    This class intentionally does not contain any autonomous flight logic.
    It only observes the vehicle state through the existing PX4Interface getter
    APIs already implemented in mission_controller/px4_getters.py.
    """

    def __init__(self, px4, csv_path: Optional[str] = None):
        self.px4 = px4
        self.csv_path = csv_path
        self.csv_file = None
        self.csv_writer = None

        if csv_path:
            self._open_csv(csv_path)

    def _open_csv(self, csv_path: str) -> None:
        """Create a CSV log file for later debugging/comparison."""
        self.csv_file = open(csv_path, "w", newline="")
        self.csv_writer = csv.DictWriter(
            self.csv_file,
            fieldnames=[
                "timestamp",
                "connected",
                "armed",
                "mode",
                "landed",
                "local_x",
                "local_y",
                "local_z",
                "vel_x",
                "vel_y",
                "vel_z",
                "alt_relative",
                "alt_amsl",
                "gps_fix_type",
                "gps_satellites",
                "gps_eph",
                "gps_epv",
                "battery_voltage",
                "battery_current",
                "battery_percentage",
            ],
        )
        self.csv_writer.writeheader()

    def close(self) -> None:
        """Close the CSV file cleanly if logging was enabled."""
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None

    def collect_row(self) -> Dict[str, Any]:
        """
        Collect one telemetry snapshot and flatten it into a simple dictionary.

        We use get_full_telemetry_snapshot() because your codebase already
        centralizes the getter APIs there. This avoids duplicating raw ROS topic
        subscriptions in the test script.
        """
        snapshot = self.px4.get_full_telemetry_snapshot()

        return {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "connected": get_nested(snapshot, "status", "connected"),
            "armed": get_nested(snapshot, "status", "armed"),
            "mode": get_nested(snapshot, "status", "mode"),
            "landed": landed_state_to_text(get_nested(snapshot, "status", "landed")),
            "local_x": get_nested(snapshot, "pose", "position", "x"),
            "local_y": get_nested(snapshot, "pose", "position", "y"),
            "local_z": get_nested(snapshot, "pose", "position", "z"),
            "vel_x": get_nested(snapshot, "velocity_local", "linear", "x"),
            "vel_y": get_nested(snapshot, "velocity_local", "linear", "y"),
            "vel_z": get_nested(snapshot, "velocity_local", "linear", "z"),
            "alt_relative": get_nested(snapshot, "altitude", "relative"),
            "alt_amsl": get_nested(snapshot, "altitude", "amsl"),
            "gps_fix_type": get_nested(snapshot, "gps_raw_1", "fix_type"),
            "gps_satellites": get_nested(snapshot, "gps_raw_1", "satellites_visible"),
            "gps_eph": get_nested(snapshot, "gps_raw_1", "eph"),
            "gps_epv": get_nested(snapshot, "gps_raw_1", "epv"),
            "battery_voltage": get_nested(snapshot, "battery", "voltage"),
            "battery_current": get_nested(snapshot, "battery", "current"),
            "battery_percentage": get_nested(snapshot, "battery", "percentage"),
        }

    def print_row(self, row: Dict[str, Any]) -> None:
        """
        Print the most important values in a compact, readable format.

        The most important hover-prep values are:
        - mode / armed / landed
        - local z stability
        - local velocity near zero while stationary
        - GPS fix + satellite count
        - altitude relative vs local z comparison
        """
        print("\n" + "=" * 72)
        print(f"[DIAG] {row['timestamp']}")
        print(
            f"State: connected={row['connected']} | "
            f"armed={row['armed']} | "
            f"mode={row['mode']} | "
            f"landed={row['landed']}"
        )
        print(
            "Local pose: "
            f"x={fmt(row['local_x'])}, "
            f"y={fmt(row['local_y'])}, "
            f"z={fmt(row['local_z'])}"
        )
        print(
            "Local velocity: "
            f"vx={fmt(row['vel_x'])}, "
            f"vy={fmt(row['vel_y'])}, "
            f"vz={fmt(row['vel_z'])} m/s"
        )
        print(
            "Altitude: "
            f"relative={fmt(row['alt_relative'])}, "
            f"AMSL={fmt(row['alt_amsl'])}"
        )
        print(
            "GPS: "
            f"fix_type={row['gps_fix_type'] if row['gps_fix_type'] is not None else 'N/A'}, "
            f"satellites={row['gps_satellites'] if row['gps_satellites'] is not None else 'N/A'}, "
            f"eph={fmt(row['gps_eph'])}, "
            f"epv={fmt(row['gps_epv'])}"
        )
        print(
            "Battery: "
            f"voltage={fmt(row['battery_voltage'])} V, "
            f"current={fmt(row['battery_current'])} A, "
            f"percentage={fmt(row['battery_percentage'])}"
        )
        print("=" * 72)

    def log_row(self, row: Dict[str, Any]) -> None:
        """Write one row to CSV if CSV logging is enabled."""
        if self.csv_writer:
            self.csv_writer.writerow(row)
            self.csv_file.flush()

    def run(self, duration: Optional[float], print_period: float) -> None:
        """
        Main diagnostics loop.

        rclpy.spin_once() is required so ROS 2 subscription callbacks keep
        updating the cached telemetry inside PX4Getters.
        """
        start_time = time.time()
        next_print = 0.0

        print("[DIAG] Starting hover diagnostics loop.")
        print("[DIAG] This script does not arm, take off, or move the drone by default.")
        print("[DIAG] Press Ctrl+C to stop.\n")

        while True:
            now = time.time()

            if duration is not None and (now - start_time) >= duration:
                print("[DIAG] Duration reached. Exiting diagnostics loop.")
                return

            # Let ROS process incoming MAVROS messages.
            rclpy.spin_once(self.px4, timeout_sec=0.1)

            # Print/log at the requested period instead of spamming the console.
            if now >= next_print:
                row = self.collect_row()
                self.print_row(row)
                self.log_row(row)
                next_print = now + print_period


# -----------------------------------------------------------------------------
# Program entry point
# -----------------------------------------------------------------------------

def build_fcu_url(args: argparse.Namespace) -> str:
    """Choose MAVROS FCU URL from command-line arguments."""
    if args.sitl:
        return "udp://127.0.0.1:14540"
    if args.port:
        return f"serial://{args.port}:921600"
    return "serial:///dev/ttyTHS1:921600"


def safe_rclpy_shutdown() -> None:
    """
    Shutdown ROS 2 without crashing if shutdown was already called.

    Your previous test_arm.py log showed:
        rcl_shutdown already called
    This helper prevents that cleanup issue from becoming noisy.
    """
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception as exc:
        print(f"[DIAG][WARN] rclpy shutdown warning: {exc}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Safe hover diagnostics for PX4/MAVROS. No arming or takeoff by default."
    )
    parser.add_argument("--sitl", action="store_true", help="Use PX4 SITL UDP connection.")
    parser.add_argument("--port", type=str, default=None, help="Hardware serial port, e.g. /dev/ttyTHS1 or /dev/ttyUSB0.")
    parser.add_argument("--no-boot", action="store_true", help="Do not launch MAVROS; connect to an already running MAVROS instance.")
    parser.add_argument("--duration", type=float, default=None, help="How long to run in seconds. Default: run until Ctrl+C.")
    parser.add_argument("--print-period", type=float, default=1.0, help="Seconds between printed summaries. Default: 1.0.")
    parser.add_argument("--csv", type=str, default=None, help="Optional path to save diagnostics as CSV.")
    parser.add_argument(
        "--offboard",
        action="store_true",
        help="Optional: start zero-velocity OFFBOARD heartbeat and switch to OFFBOARD. No arming/takeoff.",
    )
    parser.add_argument("--namespace", type=str, default="mavros", help="MAVROS namespace. Default: mavros.")

    args = parser.parse_args()
    fcu_url = build_fcu_url(args)

    px4 = None
    diagnostics = None
    offboard_stream_started = False

    try:
        if not args.no_boot:
            print(f"[MAIN] Booting MAVROS/PX4 bridge with FCU URL: {fcu_url}")
            boot_px4(fcu_url=fcu_url)
        else:
            print("[MAIN] --no-boot selected. Assuming MAVROS is already running.")

        print("[MAIN] Initializing PX4Interface...")
        if not rclpy.ok():
            rclpy.init()
        px4 = init_px4(namespace=args.namespace)

        if not px4 or not px4.connected:
            print("[MAIN][ERROR] Failed to connect to MAVROS / PX4.")
            return 1

        print("[MAIN] Connected to MAVROS / PX4.")

        # Optional OFFBOARD heartbeat test.
        # This is useful before hover because PX4 requires continuous setpoints
        # in OFFBOARD mode. It still does not arm or move the drone.
        if args.offboard:
            print("[MAIN] Starting background zero-velocity OFFBOARD heartbeat...")
            offboard_stream_started = px4.start_offboard_stream_background(rate_hz=10)

            print("[MAIN] Switching to OFFBOARD mode...")
            if not px4.start_offboard():
                print("[MAIN][ERROR] Failed to enter OFFBOARD mode.")
                return 1
            print("[MAIN] OFFBOARD active. Vehicle is still NOT armed by this script.")

        diagnostics = HoverDiagnostics(px4, csv_path=args.csv)
        diagnostics.run(duration=args.duration, print_period=args.print_period)
        return 0

    except KeyboardInterrupt:
        print("\n[MAIN] Interrupted by user.")
        return 0

    except Exception as exc:
        print(f"[MAIN][ERROR] Diagnostics failed: {exc}")
        import traceback
        traceback.print_exc()
        return 1

    finally:
        print("[MAIN] Cleaning up...")

        if diagnostics:
            diagnostics.close()

        if px4 and offboard_stream_started:
            try:
                px4.stop_offboard_stream_background()
            except Exception as exc:
                print(f"[MAIN][WARN] Failed to stop OFFBOARD stream cleanly: {exc}")

        if px4:
            try:
                px4.disconnect()
            except Exception as exc:
                print(f"[MAIN][WARN] PX4 disconnect warning: {exc}")

        if not args.no_boot:
            try:
                stop_px4()
            except Exception as exc:
                print(f"[MAIN][WARN] stop_px4 warning: {exc}")

        safe_rclpy_shutdown()


if __name__ == "__main__":
    sys.exit(main())
