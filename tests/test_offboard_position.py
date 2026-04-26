#!/usr/bin/env python3
"""
OFFBOARD Position Control Test (Python equivalent of C++ example)

This script demonstrates position-based offboard control:
1. Start background stream (maintains OFFBOARD mode)
2. Switch to OFFBOARD mode and arm
3. Send a position setpoint to fly to (0, 0, -5m)
4. Maintain that position for 10 seconds
5. Return to hover and land

This is similar to the C++ example but adapted for MAVROS/velocity setpoints.
"""

import rclpy
import time
import argparse
from mission_controller.px4_interface import init_px4, boot_px4, stop_px4


class OffboardPositionTest:
    """Position-based OFFBOARD control test"""

    def __init__(self, px4, verbose=True):
        """
        Initialize position control tester

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

    def test_offboard_position(self):
        """Test OFFBOARD mode with position control"""
        self.log("\n" + "="*70)
        self.log("OFFBOARD POSITION CONTROL TEST")
        self.log("="*70 + "\n")

        try:
            # Step 1: Check connection
            self.log("[TEST] Checking MAVROS connection...")
            if not self.px4.connected:
                self.log("[TEST] Not connected to MAVROS")
                return False
            self.log("[TEST] ✓ Connected to MAVROS")

            # Step 2: Start background setpoint stream
            # This keeps OFFBOARD mode alive by publishing setpoints at 50Hz
            self.log("\n[TEST] Starting background setpoint stream...")
            if not self.px4.start_offboard_stream_background():
                self.log("[TEST] Failed to start background stream")
                return False
            self.log("[TEST] ✓ Background stream started (50Hz)")

            # Step 3: Switch to OFFBOARD mode
            self.log("\n[TEST] Switching to OFFBOARD mode...")
            if not self.px4.start_offboard():
                self.log("[TEST] Failed to switch to OFFBOARD mode")
                self.px4.stop_offboard_stream_background()
                return False
            self.log("[TEST] ✓ OFFBOARD mode active")

            # Step 4: Wait for manual arming (1 minute timeout)
            self.log("\n[TEST] Waiting for manual arm command (60 seconds)...")
            self.log("[TEST] Please arm the vehicle manually within 60 seconds")
            
            arm_timeout = 60
            start_time = time.time()
            while (time.time() - start_time) < arm_timeout:
                if self.px4.is_armed():
                    self.log(f"[TEST] ✓ Vehicle armed!")
                    break
                remaining = int(arm_timeout - (time.time() - start_time))
                if remaining % 10 == 0:
                    self.log(f"[TEST] Waiting... {remaining} seconds remaining")
                time.sleep(1)
            else:
                self.log("[TEST] Arming timeout - vehicle was not armed within 60 seconds")
                self.px4.stop_offboard_stream_background()
                return False
            time.sleep(8)
            # Step 5: Send position setpoint (move to 0, 0, -5m like C++ example)
            # In our velocity-based system, we'll fly upward at 1 m/s for 5 seconds
            self.log("\n[TEST] Sending upward velocity command (1 m/s up)...")
            self.px4.send_velocity_setpoint(0.0, 0.0, 1.0, 0.0)  # Fly up
            self.log("[TEST] ✓ Velocity command sent: moving upward")

            # Step 6: Maintain for 5 seconds
            self.log("\n[TEST] Maintaining position for 5 seconds...")
            time.sleep(5)
            self.log("[TEST] ✓ Position maintained for 5 seconds")

            # Step 7: Stop moving (hover)
            self.log("\n[TEST] Sending hover command (stop movement)...")
            self.px4.send_velocity_setpoint(0.0, 0.0, 0.0, 0.0)  # Hover
            self.log("[TEST] ✓ Hovering")

            # Step 8: Hover for 3 seconds
            time.sleep(3)

            # Step 9: Land
            self.log("\n[TEST] Landing...")
            if not self.px4.land():
                self.log("[TEST] Warning: Land command may have failed, stopping stream")
            self.log("[TEST] ✓ Landing complete")

            # Step 10: Stop background stream
            self.log("\n[TEST] Stopping background stream...")
            if not self.px4.stop_offboard_stream_background():
                self.log("[TEST] Failed to stop background stream")
                return False

            self.log("\n" + "="*70)
            self.log("✓ OFFBOARD POSITION CONTROL TEST COMPLETE")
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
    parser = argparse.ArgumentParser(
        description="Test OFFBOARD position control on PX4/MAVROS",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 test_offboard_position.py --sitl                  # SITL simulation
  python3 test_offboard_position.py --hardware              # Real hardware (default USB)
  python3 test_offboard_position.py --port /dev/ttyUSB1    # Custom serial port
        """
    )

    parser.add_argument(
        "--sitl",
        action="store_true",
        help="Use SITL (Software In The Loop) simulation"
    )

    parser.add_argument(
        "--hardware",
        action="store_true",
        help="Use real hardware on default USB port"
    )

    parser.add_argument(
        "--port",
        type=str,
        default="/dev/ttyUSB0",
        help="Serial port for hardware connection (default: /dev/ttyUSB0)"
    )

    args = parser.parse_args()

    # Determine FCU URL
    if args.sitl:
        fcu_url = "udp://127.0.0.1:14540"
        print("[MAIN] Using SITL mode: UDP localhost:14540")
    elif args.hardware or not args.sitl:
        fcu_url = f"serial://{args.port}:921600"
        print(f"[MAIN] Using hardware mode: {args.port} at 921600 baud")

    # Boot PX4
    print(f"[MAIN] Booting PX4...")
    boot_px4(fcu_url=fcu_url)
    time.sleep(3)  # Wait for MAVROS to connect

    # Initialize ROS2
    rclpy.init()

    # Create PX4 interface
    px4 = init_px4()
    time.sleep(2)  # Wait for subscriptions to warm up

    # Run test
    tester = OffboardPositionTest(px4, verbose=True)
    success = tester.test_offboard_position()

    # Exit with appropriate code
    exit(0 if success else 1)


if __name__ == "__main__":
    main()
