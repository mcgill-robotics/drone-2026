#!/usr/bin/env python3
"""
OFFBOARD Hover Test

This script demonstrates basic hover control in OFFBOARD mode:
1. Boot PX4 and connect to MAVROS
2. Switch to OFFBOARD mode
3. Wait for manual RC arm (can use --api for automatic arm)
4. Fly upward at 0.5 m/s for 3 seconds
5. Hover (stop movement) for 3 seconds
6. Land and stop background stream

Default: Wait for manual RC arm
With --api: Automatically arm via MAVROS

Usage:
    python3 test_hover.py                   # Manual RC arm with default hardware port
    python3 test_hover.py --sitl            # Manual RC arm with SITL
    python3 test_hover.py --hardware        # Manual RC arm with real hardware
    python3 test_hover.py --api             # API arm with default hardware port
    python3 test_hover.py --port /dev/ttyUSB0  # Custom serial port
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

    def __init__(self, px4, verbose=True, manual_arm=True):
        """
        Initialize hover tester

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

    def test_hover(self):
        """Test OFFBOARD mode with hover control"""
        self.log("\n" + "="*70)
        self.log("OFFBOARD HOVER TEST")
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
            self.log("[TEST] ✓ Background stream started (50Hz)")

            # Step 4: Wait for arming (manual or API)
            if self.manual_arm:
                self.log("\n[TEST] Waiting for manual arm (60 seconds)...")
                self.log("[TEST] Please arm the vehicle manually via RC transmitter")
                
                # Simple polling approach with detailed logging
                start_arm = time.time()
                arm_timeout = 60
                poll_interval = 0.5
                
                while (time.time() - start_arm) < arm_timeout:
                    rclpy.spin_once(self.px4, timeout_sec=0.1)
                    
                    is_armed = self.px4.is_armed()
                    current_state = self.px4.current_state
                    
                    if is_armed:
                        self.log("[TEST] ✓ Vehicle armed!")
                        break
                    
                    elapsed = time.time() - start_arm
                    if elapsed % 5 < 0.5:  # Log every ~5 seconds
                        self.log(f"[TEST] Waiting... {int(arm_timeout - elapsed)}s remaining")
                        self.log(f"[TEST]   is_armed()={is_armed}, current_state={current_state}")
                        if current_state:
                            self.log(f"[TEST]   current_state.armed={current_state.armed}, current_state.connected={current_state.connected}")
                    
                    time.sleep(poll_interval)
                else:
                    self.log("[TEST] Arm timeout - vehicle was not armed")
                    self.log(f"[TEST]   Final state: is_armed()={self.px4.is_armed()}")
                    if self.px4.current_state:
                        self.log(f"[TEST]   current_state.armed={self.px4.current_state.armed}")
                    return False
            else:
                #this must never happen, we always want to arm manually
                self.log("\n[TEST] Arming vehicle via API...")
                if not self.px4.arm_vehicle(timeout=20):
                    self.log("[TEST] API arm failed")
                    return False
                self.log("[TEST] ✓ Vehicle armed via API")

            # Step 5: Fly upward at 0.5 m/s for 3 seconds
            self.log("\n[TEST] Sending upward velocity command (0.5 m/s up)...")
            self.px4.send_velocity_setpoint(0.0, 0.0, 0.5, 0.0)  # Fly up
            self.log("[TEST] ✓ Velocity command sent: moving upward")

            # Step 6: Maintain upward flight for 3 seconds
            self.log("\n[TEST] Maintaining upward flight for 3 seconds...")
            time.sleep(3)
            self.log("[TEST] ✓ Upward flight maintained")

            # Step 7: Stop moving (hover)
            self.log("\n[TEST] Sending hover command (stop movement)...")
            self.px4.send_velocity_setpoint(0.0, 0.0, 0.0, 0.0)  # Hover
            self.log("[TEST] ✓ Hovering")

            # Step 8: Hover for 3 seconds
            self.log("\n[TEST] Hovering for 3 seconds...")
            time.sleep(30)
            self.log("[TEST] ✓ Hover complete")

            # Step 9: Land
            self.log("\n[TEST] Landing...")
            
            if not self.px4.land():
                self.log("[TEST] Warning: Land command may have failed")
            self.log("[TEST] ✓ Landing initiated")

            # Step 10: Stop background stream
            self.log("\n[TEST] Stopping background stream...")
            if not self.px4.stop_offboard_stream_background():
                self.log("[TEST] Failed to stop background stream")
                return False

            self.log("\n" + "="*70)
            self.log("✓ OFFBOARD HOVER TEST COMPLETE")
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
    parser = argparse.ArgumentParser(description="Test OFFBOARD hover sequence")
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
    manual_arm = True  # Default to manual, unless --api flag is set

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

        # Run hover test
        tester = HoverTest(px4, verbose=True, manual_arm=manual_arm)
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
