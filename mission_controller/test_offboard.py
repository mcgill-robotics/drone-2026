#!/usr/bin/env python3
"""
OFFBOARD Mode Test Script

This script:
1. Boots PX4
2. Connects to MAVROS
3. Switches to OFFBOARD mode
4. Warms up setpoint stream (>2Hz)
5. Tests OFFBOARD by publishing commands
6. Waits for manual arm
7. Once armed, continues streaming setpoints (hover in place)

This is useful for:
- Testing if OFFBOARD mode is working
- Verifying setpoint publishing
- Testing drone responsiveness
- Preparing for autonomous missions

WORKFLOW:
1. SSH into Jetson and run: python3 test_offboard.py --hardware
2. Script will output: "OFFBOARD mode active, setpoints streaming"
3. Script will output: "Waiting for drone to be armed..."
4. You can now disconnect the ethernet cable
5. Arm the drone manually
6. Script will detect arm and continue hovering with zero velocity
7. Press Ctrl+C to stop and disarm

Usage:
    python3 test_offboard.py --sitl              # Use SITL simulation
    python3 test_offboard.py --hardware           # Use real hardware on USB
    python3 test_offboard.py --port /dev/ttyUSB0 # Custom port
"""

import rclpy
import time
import argparse
from px4_interface import init_px4, boot_px4, stop_px4


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

            # Step 2: Switch to OFFBOARD mode and warm up setpoints
            self.log("\n[TEST] Switching to OFFBOARD mode...")
            self.log("[TEST] Warming up setpoint stream (need >2Hz for OFFBOARD)")
            
            if not self.px4.start_offboard():
                self.log("[TEST] Failed to switch to OFFBOARD mode")
                return False
            
            self.log("[TEST] OFFBOARD mode active")

            # Step 3: Test publishing different velocity commands
            self.log("\n[TEST] Testing velocity setpoint publishing...")
            
            # Test forward
            self.log("[TEST] Publishing forward velocity (1.0 m/s)...")
            for i in range(20):
                self.px4.send_velocity_setpoint(1.0, 0.0, 0.0, 0.0)
                rclpy.spin_once(self.px4, timeout_sec=0.05)
                time.sleep(0.05)
            self.log("[TEST] Forward velocity published (20 messages)")

            # Test backward
            self.log("[TEST] Publishing backward velocity (-1.0 m/s)...")
            for i in range(20):
                self.px4.send_velocity_setpoint(-1.0, 0.0, 0.0, 0.0)
                rclpy.spin_once(self.px4, timeout_sec=0.05)
                time.sleep(0.05)
            self.log("[TEST] Backward velocity published (20 messages)")

            # Test upward
            self.log("[TEST] Publishing upward velocity (-0.5 m/s in Z)...")
            for i in range(20):
                self.px4.send_velocity_setpoint(0.0, 0.0, -0.5, 0.0)
                rclpy.spin_once(self.px4, timeout_sec=0.05)
                time.sleep(0.05)
            self.log("[TEST] Upward velocity published (20 messages)")

            # Stop
            self.log("[TEST] Publishing zero velocity (hover)...")
            for i in range(20):
                self.px4.send_velocity_setpoint(0.0, 0.0, 0.0, 0.0)
                rclpy.spin_once(self.px4, timeout_sec=0.05)
                time.sleep(0.05)
            self.log("[TEST] Zero velocity published (20 messages)")

            self.log("\n[TEST] Setpoint publishing test complete!")

            # Step 4: Wait for manual arm
            self.log("\n[TEST] Waiting for drone to be armed...")
            self.log("[TEST] You can now disconnect the ethernet cable")
            self.log("[TEST] Arm the drone manually (via RC remote or button)")
            
            arm_timeout = 120  # 2 minutes
            start_wait = time.time()
            
            while not self.px4.is_armed() and (time.time() - start_wait) < arm_timeout:
                rclpy.spin_once(self.px4, timeout_sec=0.1)
                time.sleep(0.5)
                elapsed = int(time.time() - start_wait)
                if elapsed % 5 == 0:
                    print(f"[TEST] Waiting for arm... ({elapsed}s)")
            
            if not self.px4.is_armed():
                self.log("[TEST] Timeout waiting for arm. Exiting.")
                return False
            
            self.log("[TEST] Drone armed!")

            # Step 5: Continue streaming hover setpoints
            self.log("\n[TEST] Drone armed! Streaming hover commands...")
            self.log("[TEST] Press Ctrl+C to stop and disarm")
            
            hover_time = 0
            try:
                while self.running:
                    # Stream hover (zero velocity)
                    self.px4.send_velocity_setpoint(0.0, 0.0, 0.0, 0.0)
                    rclpy.spin_once(self.px4, timeout_sec=0.05)
                    time.sleep(0.05)
                    
                    hover_time += 0.05
                    if int(hover_time) % 5 == 0 and hover_time < int(hover_time) + 0.1:
                        self.log(f"[TEST] Hovering... ({int(hover_time)}s)")
            
            except KeyboardInterrupt:
                self.log("\n[TEST] Interrupted by user")

            # Step 6: Disarm
            self.log("\n[TEST] Disarming...")
            if self.px4.disarm_vehicle():
                self.log("[TEST] ✓ Vehicle disarmed")
            else:
                self.log("[TEST] Failed to disarm (may already be disarmed)")

            self.log("\n" + "="*70)
            self.log("✓ OFFBOARD TEST COMPLETE")
            self.log("="*70 + "\n")

            return True

        except Exception as e:
            self.log(f"\n[TEST] Exception: {str(e)}")
            self.log("[TEST] Attempting to disarm...")
            try:
                self.px4.disarm_vehicle()
            except:
                pass
            return False

        finally:
            # Cleanup
            self.log("[TEST] Shutting down...")
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
