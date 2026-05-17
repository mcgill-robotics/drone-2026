#!/usr/bin/env python3
"""
Diagnostic script to test if commands are being sent to MAVROS

This script does NOT require the drone to be armed.
It tests:
1. Connection to MAVROS
2. Publishing velocity setpoints
3. Publishing position setpoints
4. Mode changes

Run this and monitor topics in another terminal:
  ros2 topic echo /mavros/setpoint_velocity/cmd_vel
  ros2 topic echo /mavros/setpoint_raw/local_position
"""

import sys
import os

# Add parent directory to path so mission_controller can be imported
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import rclpy
import time
import argparse
from mission_controller.px4_interface import init_px4, boot_px4, stop_px4


def test_commands(px4):
    """Test publishing commands to MAVROS"""
    
    print("\n" + "="*70)
    print("MAVROS COMMAND DIAGNOSTIC TEST")
    print("="*70)
    
    # Check connection
    print("\n[TEST] Checking MAVROS connection...")
    if not px4.connected:
        print("[TEST] ❌ Not connected to MAVROS!")
        return False
    print("[TEST] ✓ Connected to MAVROS")
    
    # Test 1: Publish velocity setpoints
    print("\n[TEST] Test 1: Publishing velocity setpoints...")
    print("[TEST] Publishing forward velocity (1.0 m/s)")
    for i in range(10):
        px4.send_velocity_setpoint(1.0, 0.0, 0.0, 0.0)
        rclpy.spin_once(px4, timeout_sec=0.05)
        time.sleep(0.1)
        if i % 5 == 0:
            print(f"[TEST]   Still publishing... ({i+1}/10)")
    print("[TEST] ✓ Forward velocity published")
    
    time.sleep(1)
    
    # Test 2: Publish backward velocity
    print("\n[TEST] Test 2: Publishing backward velocity...")
    print("[TEST] Publishing backward velocity (-1.0 m/s)")
    for i in range(10):
        px4.send_velocity_setpoint(-1.0, 0.0, 0.0, 0.0)
        rclpy.spin_once(px4, timeout_sec=0.05)
        time.sleep(0.1)
        if i % 5 == 0:
            print(f"[TEST]   Still publishing... ({i+1}/10)")
    print("[TEST] ✓ Backward velocity published")
    
    time.sleep(1)
    
    # Test 3: Publish strafe (Y) velocity
    print("\n[TEST] Test 3: Publishing strafe velocity...")
    print("[TEST] Publishing right strafe (1.0 m/s, Y)")
    for i in range(10):
        px4.send_velocity_setpoint(0.0, 1.0, 0.0, 0.0)
        rclpy.spin_once(px4, timeout_sec=0.05)
        time.sleep(0.1)
        if i % 5 == 0:
            print(f"[TEST]   Still publishing... ({i+1}/10)")
    print("[TEST] ✓ Strafe velocity published")
    
    time.sleep(1)
    
    # Test 4: Publish altitude velocity (up/down)
    print("\n[TEST] Test 4: Publishing altitude velocity...")
    print("[TEST] Publishing upward velocity (1.0 m/s, Z)")
    for i in range(10):
        px4.send_velocity_setpoint(0.0, 0.0, -1.0, 0.0)  # Negative Z is up in NED
        rclpy.spin_once(px4, timeout_sec=0.05)
        time.sleep(0.1)
        if i % 5 == 0:
            print(f"[TEST]   Still publishing... ({i+1}/10)")
    print("[TEST] ✓ Altitude velocity published")
    
    time.sleep(1)
    
    # Test 5: Stop commands (zero velocity)
    print("\n[TEST] Test 5: Stopping commands...")
    print("[TEST] Publishing zero velocity")
    for i in range(10):
        px4.send_velocity_setpoint(0.0, 0.0, 0.0, 0.0)
        rclpy.spin_once(px4, timeout_sec=0.05)
        time.sleep(0.1)
    print("[TEST] ✓ Stop command published")
    
    time.sleep(1)
    
    # Test 6: Test mode changes
    print("\n[TEST] Test 6: Testing mode changes...")
    print("[TEST] Attempting to change mode to GUIDED...")
    if px4.change_mode("GUIDED"):
        print("[TEST] ✓ Mode change command sent")
    else:
        print("[TEST] ⚠️  Mode change may have failed (check autopilot)")
    
    print("\n" + "="*70)
    print("✓ DIAGNOSTIC TEST COMPLETE")
    print("="*70)
    print("\nTo verify commands are being received:")
    print("  Open another terminal and run:")
    print("    ros2 topic echo /mavros/setpoint_velocity/cmd_vel")
    print("  You should see velocity messages appearing in real-time\n")
    
    return True


def main():
    parser = argparse.ArgumentParser(description="Test MAVROS command publishing")
    parser.add_argument("--sitl", action="store_true", help="Use SITL simulation")
    parser.add_argument("--hardware", action="store_true", help="Use real hardware")
    parser.add_argument("--port", type=str, help="Custom serial port")
    
    args = parser.parse_args()
    
    # Determine FCU URL
    if args.sitl:
        fcu_url = "udp://:14540@localhost:14580"
        print("[MAIN] Using SITL simulation")
    elif args.port:
        fcu_url = f"serial://{args.port}:921600"
        print(f"[MAIN] Using custom port: {args.port}")
    else:
        fcu_url = "serial:///dev/ttyUSB0:921600"
        print("[MAIN] Using hardware (default USB)")
    
    print("[MAIN] Booting PX4...")
    boot_px4(fcu_url=fcu_url)
    
    print("[MAIN] Initializing MAVROS interface...")
    rclpy.init()
    px4 = init_px4(namespace="mavros")
    
    if not px4.connected:
        print("[MAIN] ❌ Failed to connect to MAVROS")
        stop_px4()
        return 1
    
    print("[MAIN] ✓ Connected!\n")
    
    # Run tests
    try:
        success = test_commands(px4)
        return 0 if success else 1
    except KeyboardInterrupt:
        print("\n[MAIN] Interrupted by user")
        return 1
    except Exception as e:
        print(f"\n[MAIN] ❌ Error: {str(e)}")
        return 1
    finally:
        print("[MAIN] Cleaning up...")
        try:
            px4.disconnect()
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


if __name__ == "__main__":
    exit(main())
