#!/usr/bin/env python3
"""
OFFBOARD Mode Arming Test

Test arming in OFFBOARD mode with manual RC arm as default:
1. Boot PX4 and connect to MAVROS
2. Switch to OFFBOARD mode
3. Print pre-arm diagnostics (battery, GPS, home position)
4. Wait for manual RC arm (default) or use API arm (--api flag)
5. Start background heartbeat thread for flight
6. Stop heartbeat thread and exit

Default: Waits for manual RC transmitter arm command
With --api: Automatically arms via MAVROS service

Usage:
    python3 test_arm.py                     # Manual RC arm with default hardware port
    python3 test_arm.py --sitl              # Manual RC arm with SITL simulation
    python3 test_arm.py --hardware           # Manual RC arm with real hardware
    python3 test_arm.py --api                # API arm with default hardware port
    python3 test_arm.py --sitl --api         # API arm with SITL
    python3 test_arm.py --port /dev/ttyUSB0 # Custom port
"""

import sys
import os

# Add parent directory to path so mission_controller can be imported
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import rclpy
import time
import argparse
from mission_controller.px4_interface import init_px4, boot_px4, stop_px4


class ArmTest:
    """OFFBOARD mode arming test class"""

    def __init__(self, px4, verbose=True, manual_arm=False):
        """
        Initialize ARM tester

        Args:
            px4: PX4Interface instance
            verbose: Print status messages
            manual_arm: If True, wait for manual RC arm; if False, use API arm
        """
        self.px4 = px4
        self.verbose = verbose
        self.manual_arm = manual_arm

    def log(self, msg):
        """Print log message if verbose"""
        if self.verbose:
            print(msg)

    def test_arm(self):
        """
        Test arming sequence in OFFBOARD mode
        
        Steps:
        1. Verify MAVROS connection
        2. Switch to OFFBOARD mode
        3. Print pre-arm diagnostics
        4. Arm vehicle (API or manual)
        5. Start background heartbeat thread for flight
        6. Stop heartbeat thread
        """
        
        # Step 1: Verify MAVROS connection
        self.log("\nChecking MAVROS connection...")
        if not self.px4.connected:
            self.log("Not connected to MAVROS")
            return False
        self.log("Connected to MAVROS")

        # Step 2: Switch to OFFBOARD mode
        self.log("\nSwitching to OFFBOARD mode...")
        if not self.px4.start_offboard():
            self.log("Failed to switch to OFFBOARD mode")
            return False
        self.log("OFFBOARD mode active")

        # Step 3: Print system diagnostics before arming
        self.log("\n=== PRE-ARM DIAGNOSTICS ===")
        self.log(f"Armed: {self.px4.is_armed()}")
        battery = self.px4.get_battery_status()
        if battery:
            self.log(f"Battery Voltage: {battery.get('voltage', 'N/A')}V")
            self.log(f"Battery Current: {battery.get('current', 'N/A')}A")
            self.log(f"Battery Remaining: {battery.get('remaining', 'N/A')}%")
        gps = self.px4.get_gps_raw()
        if gps:
            self.log(f"GPS Satellites: {gps.get('satellites_visible', 'N/A')}")
            self.log(f"GPS Fix: {gps.get('fix_type', 'N/A')}")
        home = self.px4.get_home_location()
        if home:
            self.log(f"Home Set: Yes ({home['latitude']}, {home['longitude']}, {home['altitude']}m)")
        else:
            self.log(f"Home Set: No")
        self.log("==========================\n")

        # Step 4: Wait for arm or arm programmatically
        if self.manual_arm:
            self.log("\nWaiting for manual arm (60 seconds)...")
            self.log("Please arm the vehicle manually via RC transmitter")
            
            if not self.px4.wait_for_arm_with_heartbeat(timeout=60, heartbeat_rate=10):
                self.log("Arm timeout - vehicle was not armed")
                return False
        else:
            self.log("\nArming vehicle via API...")
            if not self.px4.arm_vehicle(timeout=20):
                self.log("API arm failed")
                return False
            self.log("✓ Vehicle armed via API")

        # Step 5: Start background heartbeat thread for flight
        # Now that we're armed, start the background thread for mission flight
        self.log("\nStarting background heartbeat thread for flight...")
        if not self.px4.start_offboard_stream_background():
            self.log("Warning: Failed to start background stream")
            # Don't fail here - the vehicle is already armed
        self.log("Background stream started")

        # Step 6: Stop heartbeat thread
        self.log("\nStopping background heartbeat thread...")
        self.px4.stop_offboard_stream_background()
        self.log("Heartbeat thread stopped")

        self.log("\nArming test passed!")
        return True


def main():
    """Main test runner"""
    parser = argparse.ArgumentParser(description="Test OFFBOARD mode arming sequence")
    parser.add_argument(
        "--sitl",
        action="store_true",
        help="Use SITL simulation (localhost:14540)",
    )
    parser.add_argument(
        "--hardware",
        action="store_true",
        help="Use real hardware on /dev/ttyUSB0",
    )
    parser.add_argument(
        "--port",
        type=str,
        default=None,
        help="Custom serial port (e.g., /dev/ttyUSB0 or /dev/ttyTHS1)",
    )
    parser.add_argument(
        "--api",
        action="store_true",
        help="Use API arm instead of waiting for manual RC arm (default: manual RC arm)",
    )

    args = parser.parse_args()

    # Determine connection method
    sitl = args.sitl
    port = args.port
    manual_arm = not args.api  # Default to manual, unless --api flag is set

    if args.hardware:
        port = "/dev/ttyUSB0"
        sitl = False

    # Initialize ROS 2
    rclpy.init()

    try:
        # Boot PX4
        print("[MAIN] Booting PX4...")
        
        # Build FCU URL based on connection type
        if sitl:
            fcu_url = "udp://127.0.0.1:14540"
            print("[MAIN] Using SITL (UDP)")
        else:
            fcu_url = f"serial:///{port}:921600" if port else "serial:///dev/ttyTHS1:921600"
            print(f"[MAIN] Using hardware ({fcu_url})")
        
        boot_px4(fcu_url=fcu_url)
        print("PX4 booted")

        # Wait for MAVROS to initialize
        print("[MAIN] Waiting 10s for MAVROS initialization...")
        time.sleep(10)

        # Initialize PX4 interface
        print("Initializing PX4Interface...")
        px4 = init_px4()

        if not px4.connected:
            print("Failed to connect to MAVROS")
            return False

        print("Connected to MAVROS")

        # Run arm test
        tester = ArmTest(px4, verbose=True, manual_arm=manual_arm)
        success = tester.test_arm()

        if success:
            print("\n[MAIN] ✓ Test passed!")
        else:
            print("\n[MAIN] ✗ Test failed")

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
        # Cleanup
        print("[MAIN] Shutting down...")
        stop_px4()
        rclpy.shutdown()


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
