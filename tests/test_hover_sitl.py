#!/usr/bin/env python3
"""
OFFBOARD Hover Test - SITL Version

This script demonstrates basic OFFBOARD control with takeoff/land in SITL:
1. Boot PX4 SITL and connect to MAVROS
2. Switch to OFFBOARD mode
3. Start background heartbeat stream (maintains setpoints)
4. Automatically arm via API (no manual RC needed in SITL)
5. Takeoff to 5 meters
6. Hover for 10 seconds
7. Land back to ground
8. Stop background stream

Usage:
    python3 test_hover_sitl.py
"""

import sys
import os

# Add parent directory to path so mission_controller can be imported
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import rclpy
import time
import argparse
from mission_controller.px4_interface import init_px4, boot_px4, stop_px4


class HoverTest:
    """OFFBOARD hover test"""

    def __init__(self, px4, verbose=True):
        """
        Initialize hover tester

        Args:
            px4: PX4Interface instance
            verbose: Print status messages
        """
        self.px4 = px4
        self.verbose = verbose

    def log(self, msg):
        """Print log message if verbose"""
        if self.verbose:
            print(msg)

    def test_hover(self):
        """Test OFFBOARD mode with hover control"""
        self.log("\n" + "="*70)
        self.log("OFFBOARD HOVER TEST (SITL)")
        self.log("="*70 + "\n")

        try:
            # Step 1: Check connection
            self.log("[TEST] Checking MAVROS connection...")
            if not self.px4.connected:
                self.log("[TEST] Not connected to MAVROS")
                return False
            self.log("[TEST] ✓ Connected to MAVROS")

            # Step 2: Switch to OFFBOARD mode
            self.log("\n[TEST] Switching to OFFBOARD mode...")
            if not self.px4.start_offboard():
                self.log("[TEST] Failed to switch to OFFBOARD mode")
                return False
            self.log("[TEST] ✓ OFFBOARD mode active")

            # Step 3: Start background setpoint stream (BEFORE waiting for arm)
            # This keeps setpoints flowing continuously while in OFFBOARD mode
            self.log("\n[TEST] Starting background setpoint stream...")
            if not self.px4.start_offboard_stream_background():
                self.log("[TEST] Failed to start background stream")
                return False
            self.log("[TEST] ✓ Background stream started (10Hz heartbeat)")

            # Step 4: Arm vehicle via API (no RC needed in SITL)
            self.log("\n[TEST] Arming vehicle via API...")
            if not self.px4.arm_vehicle(timeout=20):
                self.log("[TEST] API arm failed")
                return False
            self.log("[TEST] ✓ Vehicle armed via API")
            
            # Step 5: Takeoff to 5 meters
            self.log("\n[TEST] Taking off to 5 meters...")
            if not self.px4.takeoff(altitude=5, timeout=30):
                self.log("[TEST] Takeoff failed")
                return False
            self.log("[TEST] ✓ Takeoff complete")

            # Step 6: Hover for 10 seconds
            self.log("\n[TEST] Hovering for 10 seconds...")
            time.sleep(10)
            self.log("[TEST] ✓ Hover complete")

            # Step 7: Land
            self.log("\n[TEST] Landing...")
            if not self.px4.land(timeout=30):
                self.log("[TEST] Landing failed")
                return False
            self.log("[TEST] ✓ Landing complete")

            # Step 8: Stop background stream
            self.log("\n[TEST] Stopping background stream...")
            if not self.px4.stop_offboard_stream_background():
                self.log("[TEST] Failed to stop background stream")
                return False

            self.log("\n" + "="*70)
            self.log("✓ OFFBOARD HOVER TEST (SITL) COMPLETE")
            self.log("="*70 + "\n")

            return True

        except Exception as e:
            self.log(f"\n[TEST] Exception: {str(e)}")
            import traceback
            traceback.print_exc()
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
    """Main test runner"""
    parser = argparse.ArgumentParser(description="Test OFFBOARD hover sequence in SITL")
    args = parser.parse_args()

    # Initialize ROS 2
    rclpy.init()

    try:
        # Boot PX4 SITL
        print("[MAIN] Booting PX4 SITL...")
        fcu_url = "udp://127.0.0.1:14540"
        print("[MAIN] Using SITL (UDP)")
        
        boot_px4(fcu_url=fcu_url)
        print("[MAIN] PX4 booted")

        # Wait for MAVROS to initialize
        print("[MAIN] Waiting 10s for MAVROS initialization...")
        time.sleep(10)

        # Initialize PX4 interface
        print("[MAIN] Initializing PX4Interface...")
        px4 = init_px4()

        if not px4.connected:
            print("[MAIN] Failed to connect to MAVROS")
            return False

        print("[MAIN] ✓ Connected to MAVROS")

        # Run hover test
        tester = HoverTest(px4, verbose=True)
        success = tester.test_hover()

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
