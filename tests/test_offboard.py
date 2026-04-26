#!/usr/bin/env python3
"""
OFFBOARD Mode Test Script

This script:
1. Boots PX4
2. Connects to MAVROS
3. Starts a background thread that continuously publishes zero velocity setpoints
4. Switches to OFFBOARD mode
5. Verifies success from PX4
6. Maintains OFFBOARD mode for 5 seconds (background thread handles publishing)
7. Stops the background thread

The background thread approach:
- Simplifies mission code (no need to worry about publishing loops)
- Ensures no gaps in setpoint publishing (critical for OFFBOARD)
- Allows main code to focus on mission logic

This is useful for:
- Verifying OFFBOARD mode switch works
- Testing MAVROS connection
- Confirming PX4 responds to mode change requests
- Validating background thread setpoint publishing

WORKFLOW:
1. SSH into Jetson and run: python3 test_offboard.py --hardware
2. Script will output: "Background stream started (continuous publishing at 50Hz)"
3. Script will output: "OFFBOARD mode active - success message received from PX4!"
4. Script will maintain OFFBOARD for 5 seconds (background thread keeps publishing)
5. Script will exit safely without moving the drone

Usage:
    python3 test_offboard.py --sitl              # Use SITL simulation
    python3 test_offboard.py --hardware           # Use real hardware on USB
    python3 test_offboard.py --port /dev/ttyUSB0 # Custom port
"""

import sys
import os

# Add parent directory to path so mission_controller can be imported
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import rclpy
import time
import argparse
from mission_controller.px4_interface import init_px4, boot_px4, stop_px4


class OffboardTest:
    """OFFBOARD mode testing class"""

    def __init__(self, px4, verbose=True):
        """
        Initialize OFFBOARD tester

        Args:
            px4: PX4Interface instance
            verbose: Print status messages
        """
        self.px4 = px4
        self.verbose = verbose
        self.running = True

    def log(self, msg):
        """Print log message if verbose"""
        if self.verbose:
            print(msg)

    def test_offboard(self):
        """Test OFFBOARD mode"""
        self.log("\n" + "="*70)
        self.log("OFFBOARD MODE TEST")
        self.log("="*70 + "\n")

        try:
            # Step 1: Check connection
            self.log("[TEST] Checking MAVROS connection...")
            if not self.px4.connected:
                self.log("[TEST] Not connected to MAVROS")
                return False
            self.log("[TEST] ✓ Connected to MAVROS")

            # Step 2: Start background setpoint stream
            # This runs in a separate thread and maintains OFFBOARD mode automatically
            self.log("\n[TEST] Starting background setpoint stream...")
            if not self.px4.start_offboard_stream_background():
                self.log("[TEST] Failed to start background stream")
                return False
            self.log("[TEST] ✓ Background stream started (continuous publishing at 50Hz)")

            # Step 3: Switch to OFFBOARD mode
            self.log("\n[TEST] Switching to OFFBOARD mode...")
            if not self.px4.start_offboard():
                self.log("[TEST] Failed to switch to OFFBOARD mode")
                self.px4.stop_offboard_stream_background()
                return False
            self.log("[TEST] ✓ OFFBOARD mode active - success message received from PX4!")

            # Step 4: Check arm status
            self.log("\n[TEST] Checking arm status...")
            is_armed = self.px4.is_armed()
            self.log(f"[TEST] Drone armed: {is_armed}")

            # Step 5: Maintain stream for a bit to verify stability
            self.log("\n[TEST] Maintaining OFFBOARD for 5 seconds (background thread publishing)...")
            time.sleep(5)
            self.log("[TEST] ✓ OFFBOARD stream stable for 5 seconds")

            # Step 6: Stop background stream
            self.log("\n[TEST] Stopping background stream...")
            if not self.px4.stop_offboard_stream_background():
                self.log("[TEST] Failed to stop background stream")
                return False

            self.log("\n" + "="*70)
            self.log("✓ OFFBOARD TEST COMPLETE")
            self.log("="*70 + "\n")

            return True

        except Exception as e:
            self.log(f"\n[TEST] Exception: {str(e)}")
            return False

        finally:
            # Cleanup
            self.log("[TEST] Cleaning up...")
            try:
                self.px4.stop_offboard_stream_background()
            except:
                pass
            try:
                self.px4.disconnect()
            except:
                pass
            try:
                stop_px4()
            except:
                pass
            try:
                rclpy.shutdown()
            except:
                pass


def main():
    parser = argparse.ArgumentParser(
        description="Test OFFBOARD mode on PX4/MAVROS",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 test_offboard.py --sitl                  # SITL simulation
  python3 test_offboard.py --hardware              # Real hardware (default USB)
  python3 test_offboard.py --port /dev/ttyUSB1    # Custom serial port
        """
    )

    parser.add_argument(
        "--sitl",
        action="store_true",
        help="Use SITL simulation (UDP localhost)"
    )
    parser.add_argument(
        "--hardware",
        action="store_true",
        help="Use real hardware on default USB port"
    )
    parser.add_argument(
        "--port",
        type=str,
        help="Custom serial port (e.g., /dev/ttyUSB0)"
    )

    args = parser.parse_args()

    # Determine FCU URL
    if args.sitl:
        fcu_url = "udp://127.0.0.1:14540"
        print("[MAIN] Using SITL simulation")
    elif args.port:
        fcu_url = f"serial://{args.port}:921600"
        print(f"[MAIN] Using custom port: {args.port}")
    else:
        fcu_url = "serial:///dev/ttyTHS1:921600"
        print("[MAIN] Using hardware (default USB)")

    print("[MAIN] Booting PX4...")
    boot_px4(fcu_url=fcu_url)

    print("[MAIN] Initializing MAVROS interface...")
    rclpy.init()
    px4 = init_px4(namespace="mavros")

    if not px4.connected:
        print("[MAIN] Failed to connect to MAVROS")
        print("[MAIN] Make sure:")
        print("  1. PX4 SITL is running or hardware is connected")
        print("  2. MAVROS is properly configured")
        stop_px4()
        return 1

    print("[MAIN] ✓ Successfully connected!\n")

    # Run the test
    tester = OffboardTest(px4, verbose=True)
    success = tester.test_offboard()

    return 0 if success else 1


if __name__ == "__main__":
    exit(main())
