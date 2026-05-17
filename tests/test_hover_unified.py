#!/usr/bin/env python3
"""
Unified OFFBOARD Hover Test

Combines the old tests/test_hover.py and tests/test_hover_sitl.py into one runner.

Main idea:
- SITL: connect to an already-running PX4 SITL + MAVROS by default; auto-arm by default.
- Hardware: boot MAVROS with a serial fcu_url by default; manual-arm by default.
- Same hover logic for both modes.

Example commands:
    # Gazebo/SITL: PX4 + Gazebo already running; this script launches MAVROS
    python3 tests/test_hover_unified.py --mode sitl

    # Gazebo/SITL: connect to an already-running MAVROS instead
    python3 tests/test_hover_unified.py --mode sitl --no-boot-mavros

    # Real hardware: QGC open manually; this script launches MAVROS and waits for manual arm
    python3 tests/test_hover_unified.py --mode hardware

    # Real hardware using USB serial port
    python3 tests/test_hover_unified.py --mode hardware --port /dev/ttyUSB0

    # Real hardware, API arm. Use carefully.
    python3 tests/test_hover_unified.py --mode hardware --arm api

    # Only connect and print current state, no takeoff
    python3 tests/test_hover_unified.py --mode hardware --dry-run
"""

import argparse
import os
import sys
import time
import traceback
from dataclasses import dataclass
from typing import Optional

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import rclpy
from mission_controller.px4_interface import setup_px4, shutdown_px4


@dataclass
class HoverConfig:
    mode: str
    boot_mavros: bool
    fcu_url: str
    arm_method: str
    altitude_m: float
    hover_s: float
    takeoff_timeout_s: float
    land_timeout_s: float
    connect_retries: int
    retry_sleep_s: float
    dry_run: bool
    verbose: bool


class UnifiedHoverTest:
    def __init__(self, px4, config: HoverConfig):
        self.px4 = px4
        self.config = config

    def log(self, msg: str):
        if self.config.verbose:
            print(msg)

    def print_state(self):
        self.log("\n[STATE] Current MAVROS/PX4 state:")
        self.log(f"[STATE] connected = {self.px4.connected}")
        self.log(f"[STATE] armed     = {self.px4.armed}")
        self.log(f"[STATE] mode      = {self.px4.mode}")
        try:
            pos = self.px4.get_local_position()
            self.log(f"[STATE] local position = x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}")
        except Exception as e:
            self.log(f"[STATE] local position unavailable: {e}")

    def arm(self) -> bool:
        if self.config.arm_method == "manual":
            self.log("\n[TEST] Waiting for manual arm via RC transmitter or QGC...")
            return self.px4.wait_for_arm_with_heartbeat(timeout=60, heartbeat_rate=10)

        if self.config.arm_method == "api":
            self.log("\n[TEST] Arming vehicle through MAVROS API...")
            return self.px4.arm_vehicle(timeout=20)

        raise ValueError(f"Unknown arm method: {self.config.arm_method}")

    def run(self) -> bool:
        self.log("\n" + "=" * 72)
        self.log(f"UNIFIED OFFBOARD HOVER TEST | mode={self.config.mode} | arm={self.config.arm_method}")
        self.log("=" * 72)

        try:
            self.log("\n[TEST] Checking MAVROS connection...")
            if not self.px4.connected:
                self.log("[TEST] Not connected to MAVROS")
                return False
            self.log("[TEST] ✓ Connected to MAVROS")
            self.print_state()

            if self.config.dry_run:
                self.log("\n[TEST] Dry run enabled: connection check only, no OFFBOARD/takeoff/land.")
                return True

            self.log("\n[TEST] Switching to OFFBOARD mode...")
            if not self.px4.start_offboard():
                self.log("[TEST] Failed to switch to OFFBOARD mode")
                return False
            self.log("[TEST] ✓ OFFBOARD mode active")

            self.log("\n[TEST] Starting background setpoint heartbeat stream...")
            if not self.px4.start_offboard_stream_background():
                self.log("[TEST] Failed to start background stream")
                return False
            self.log("[TEST] ✓ Background stream active")

            if not self.arm():
                self.log("[TEST] Arming failed or timed out")
                return False
            self.log("[TEST] ✓ Vehicle armed")

            self.log(f"\n[TEST] Taking off by relative altitude: +{self.config.altitude_m:.1f} m")
            if not self.px4.takeoff(
                altitude=self.config.altitude_m,
                timeout=self.config.takeoff_timeout_s,
            ):
                self.log("[TEST] Takeoff failed")
                return False
            self.log("[TEST] ✓ Takeoff complete")
            self.print_state()

            self.log(f"\n[TEST] Hovering for {self.config.hover_s:.1f} seconds...")
            time.sleep(self.config.hover_s)
            self.log("[TEST] ✓ Hover complete")

            self.log("\n[TEST] Landing...")
            if not self.px4.land(timeout=self.config.land_timeout_s):
                self.log("[TEST] Landing failed")
                return False
            self.log("[TEST] ✓ Landing complete")

            self.log("\n[TEST] Stopping background stream...")
            self.px4.stop_offboard_stream_background()

            self.log("\n" + "=" * 72)
            self.log("✓ UNIFIED OFFBOARD HOVER TEST COMPLETE")
            self.log("=" * 72 + "\n")
            return True

        except KeyboardInterrupt:
            self.log("\n[TEST] Interrupted by user")
            return False
        except Exception as e:
            self.log(f"\n[TEST] Exception: {e}")
            traceback.print_exc()
            return False
        finally:
            self.cleanup()

    def cleanup(self):
        self.log("[TEST] Cleaning up test resources...")
        try:
            self.px4.stop_offboard_stream_background()
        except Exception:
            pass
        try:
            self.px4.disconnect()
        except Exception:
            pass


def build_fcu_url(mode: str, port: Optional[str], baud: int, sitl_url: str) -> str:
    if mode == "sitl":
        return sitl_url

    selected_port = port or "/dev/ttyTHS1"
    return f"serial://{selected_port}:{baud}"


def choose_defaults(args) -> HoverConfig:
    fcu_url = build_fcu_url(
        mode=args.mode,
        port=args.port,
        baud=args.baud,
        sitl_url=args.sitl_url,
    )

    # Safer defaults:
    # - SITL usually has no RC, so API arm is convenient.
    # - Hardware should wait for manual arm unless explicitly overridden.
    if args.arm == "auto":
        arm_method = "api" if args.mode == "sitl" else "manual"
    else:
        arm_method = args.arm

    # Project default: reduce manual terminals by launching MAVROS from this script
    # for BOTH SITL and hardware. PX4/Gazebo and QGroundControl remain manual.
    if args.boot_mavros is None:
        boot_mavros = True
    else:
        boot_mavros = args.boot_mavros

    return HoverConfig(
        mode=args.mode,
        boot_mavros=boot_mavros,
        fcu_url=fcu_url,
        arm_method=arm_method,
        altitude_m=args.altitude,
        hover_s=args.hover_time,
        takeoff_timeout_s=args.takeoff_timeout,
        land_timeout_s=args.land_timeout,
        connect_retries=args.connect_retries,
        retry_sleep_s=args.retry_sleep,
        dry_run=args.dry_run,
        verbose=not args.quiet,
    )


def connect_px4(config: HoverConfig):
    px4 = None
    for attempt in range(1, config.connect_retries + 1):
        print(f"[MAIN] Connecting to MAVROS/PX4: attempt {attempt}/{config.connect_retries}")
        px4 = init_px4()
        if px4.connected:
            print("[MAIN] ✓ Connected to MAVROS")
            return px4
        if attempt < config.connect_retries:
            time.sleep(config.retry_sleep_s)

    return px4


def parse_args():
    parser = argparse.ArgumentParser(description="Unified SITL/hardware OFFBOARD hover test")

    parser.add_argument("--mode", choices=["sitl", "hardware"], required=True)
    parser.add_argument("--port", type=str, default=None, help="Hardware serial port, e.g. /dev/ttyUSB0 or /dev/ttyTHS1")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--sitl-url", type=str, default="udp://:14540@localhost:14580")

    boot_group = parser.add_mutually_exclusive_group()
    boot_group.add_argument("--boot-mavros", dest="boot_mavros", action="store_true", help="Launch MAVROS from this script; this is the default")
    boot_group.add_argument("--no-boot-mavros", dest="boot_mavros", action="store_false", help="Assume MAVROS is already running in another terminal")
    parser.set_defaults(boot_mavros=None)

    parser.add_argument("--arm", choices=["auto", "manual", "api"], default="auto")
    parser.add_argument("--altitude", type=float, default=5.0, help="Relative takeoff altitude in meters")
    parser.add_argument("--hover-time", type=float, default=10.0, help="Hover duration in seconds")
    parser.add_argument("--takeoff-timeout", type=float, default=30.0)
    parser.add_argument("--land-timeout", type=float, default=30.0)
    parser.add_argument("--connect-retries", type=int, default=5)
    parser.add_argument("--retry-sleep", type=float, default=3.0)
    parser.add_argument("--dry-run", action="store_true", help="Only connect and print state; do not arm/takeoff")
    parser.add_argument("--quiet", action="store_true")

    return parser.parse_args()


def main() -> bool:
    args = parse_args()
    config = choose_defaults(args)

    print("\n[MAIN] Unified hover test configuration")
    print(f"[MAIN] mode        = {config.mode}")
    print(f"[MAIN] fcu_url     = {config.fcu_url}")
    print(f"[MAIN] boot_mavros = {config.boot_mavros}")
    print(f"[MAIN] arm_method  = {config.arm_method}")
    print(f"[MAIN] altitude    = {config.altitude_m} m relative")
    print(f"[MAIN] hover_time  = {config.hover_s} s")

    if config.boot_mavros:
        print("[MAIN] MAVROS will be launched by this script.")
        if config.mode == "sitl":
            print("[MAIN] Make sure PX4 SITL + Gazebo are already running before this command.")
        else:
            print("[MAIN] Make sure Pixhawk/PX4 hardware is powered and connected.")
    else:
        print("[MAIN] MAVROS will NOT be launched; another terminal must already run it.")

    rclpy.init()
    mavros_started_by_this_script = False

    try:
        if config.boot_mavros:
            print("\n[MAIN] Booting MAVROS...")
            process = boot_px4(fcu_url=config.fcu_url)
            if process is None:
                print("[MAIN] Failed to boot MAVROS")
                return False
            mavros_started_by_this_script = True
            print("[MAIN] Waiting for MAVROS initialization...")
            time.sleep(5)
        else:
            print("\n[MAIN] Not booting MAVROS; assuming it is already running.")

        px4 = connect_px4(config)
        if px4 is None or not px4.connected:
            print("[MAIN] Failed to connect to MAVROS/PX4")
            return False

        return UnifiedHoverTest(px4, config).run()

    except Exception as e:
        print(f"\n[MAIN] Fatal exception: {e}")
        traceback.print_exc()
        return False
    finally:
        print("[MAIN] Shutting down...")
        if mavros_started_by_this_script:
            try:
                stop_px4()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
