#!/usr/bin/env python3
"""
GPS Movement Test

This script demonstrates GPS-based movement in OFFBOARD mode:
1. Boot PX4 and connect to MAVROS
2. Switch to OFFBOARD mode
3. Start background heartbeat stream
4. Wait for manual RC arm
5. Takeoff 5 meters using GPS coordinates (current location, local altitude frame)
6. Fly to a target GPS location (if provided)
7. Land and stop background stream

Note: Altitudes used in setpoints are LOCAL (NED frame, relative to home), not MSL.
GPS coordinates are absolute (lat/lon), but altitude is relative to home position.
"""

import sys
import os

# Add parent directory to path so mission_controller can be imported
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import rclpy
import time
import argparse
from mission_controller.px4_interface import init_px4, boot_px4, stop_px4


class GPSMovementTest:
    """GPS-based movement test"""

    def __init__(self, px4, target_gps=None, verbose=True):
        """
        Initialize GPS movement tester

        Args:
            px4: PX4Interface instance
            target_gps: Tuple of (lat, lon, alt) or None
            verbose: Print status messages
        """
        self.px4 = px4
        self.target_gps = target_gps
        self.verbose = verbose

    def log(self, msg):
        """Print log message if verbose"""
        if self.verbose:
            print(msg)

    def test_gps_movement(self):
        """Test GPS-based movement in OFFBOARD mode"""
        self.log("\n" + "="*70)
        self.log("GPS MOVEMENT TEST")
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

            # Step 3: Start background setpoint stream
            self.log("\n[TEST] Starting background setpoint stream...")
            if not self.px4.start_offboard_stream_background():
                self.log("[TEST] Failed to start background stream")
                return False

            # Step 4: Wait for arming
            self.log("\n[TEST] Waiting for manual arm (60 seconds)...")
            if not self.px4.wait_for_arm_with_heartbeat(timeout=60, heartbeat_rate=10):
                self.log("[TEST] Arm timeout - vehicle was not armed")
                return False

            # Step 5: Get current position and GPS location
            current_pos = self.px4.get_position()
            current_gps = self.px4.get_gps_location()
            if not current_pos or not current_gps:
                self.log("[TEST] Cannot get position or GPS location")
                return False
            
            current_lat = current_gps["latitude"]
            current_lon = current_gps["longitude"]
            current_z = current_pos["z"]  # Local altitude in NED frame
            
            self.log(f"\n[TEST] Current GPS: lat={current_lat:.6f}, lon={current_lon:.6f}")
            self.log(f"[TEST] Current local altitude: {current_z:.2f}m (NED frame)")
            
            # Takeoff 5m up using GPS coordinates with LOCAL altitude
            target_z_takeoff = current_z + 5
            self.log(f"\n[TEST] Taking off to {target_z_takeoff:.2f}m (local) using GPS coordinates...")
            self.px4.send_position_setpoint_gps(
                current_lat, current_lon, current_z,
                current_lat, current_lon, target_z_takeoff,
                yaw_from_direction=True
            )
            
            # Wait for takeoff to complete
            start = time.time()
            while (time.time() - start) < 30:  # 30s timeout
                rclpy.spin_once(self.px4, timeout_sec=0.1)
                
                current_pos = self.px4.get_position()
                if current_pos:
                    current_z_now = current_pos["z"]
                    if current_z_now >= (target_z_takeoff * 0.95):  # 95% of target
                        self.log(f"[TEST] ✓ Takeoff complete, reached {current_z_now:.2f}m")
                        break
                
                time.sleep(0.5)
            else:
                self.log("[TEST] Takeoff timeout")
                return False

            # Step 6: Fly to target GPS location
            if not self.target_gps:
                self.log("\n[TEST] No target GPS provided. Hovering for 10 seconds...")
                time.sleep(10)
            else:
                target_lat, target_lon, target_z = self.target_gps
                self.log(f"\n[TEST] Flying to target GPS: lat={target_lat:.6f}, lon={target_lon:.6f}, alt={target_z:.2f}m (local)...")
                
                current_pos = self.px4.get_position()
                current_z = current_pos["z"] if current_pos else 5  # Use current local altitude
                
                self.px4.send_position_setpoint_gps(
                    current_lat, current_lon, current_z,
                    target_lat, target_lon, target_z,
                    yaw_from_direction=True
                )
                
                # Wait for drone to reach target (simplified: just wait a bit)
                # In reality, you'd check distance to target
                start = time.time()
                while (time.time() - start) < 60:  # 60s timeout to reach target
                    rclpy.spin_once(self.px4, timeout_sec=0.1)
                    
                    current_gps = self.px4.get_gps_location()
                    if current_gps:
                        curr_lat = current_gps["latitude"]
                        curr_lon = current_gps["longitude"]
                        
                        # Calculate distance to target (simplified, not accounting for altitude much)
                        dlat = (target_lat - curr_lat) * 111320  # meters per degree lat
                        dlon = (target_lon - curr_lon) * 111320 * 0.707  # rough estimate for lon
                        distance = (dlat**2 + dlon**2)**0.5
                        
                        if distance < 2:  # Within 2 meters
                            self.log(f"[TEST] ✓ Reached target GPS (distance: {distance:.2f}m)")
                            break
                    
                    time.sleep(1)
                else:
                    self.log("[TEST] Timeout reaching target (may still be in progress)")

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
            self.log("✓ GPS MOVEMENT TEST COMPLETE")
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
    parser = argparse.ArgumentParser(description="Test GPS-based movement")
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
        "--target",
        type=str,
        default=None,
        help='Target GPS coordinates as "lat,lon,alt_local" where alt is LOCAL altitude in meters (e.g., "37.7749,-122.4194,10")',
    )

    args = parser.parse_args()

    # Parse target GPS if provided
    target_gps = None
    if args.target:
        try:
            parts = args.target.split(",")
            target_gps = (float(parts[0]), float(parts[1]), float(parts[2]))
            print(f"[MAIN] Target GPS: lat={target_gps[0]}, lon={target_gps[1]}, alt={target_gps[2]}")
        except (ValueError, IndexError):
            print("[MAIN] Invalid target format. Use: 'lat,lon,alt'")
            return False

    # Determine connection method
    sitl = args.sitl
    port = args.port

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

        # Run GPS movement test
        tester = GPSMovementTest(px4, target_gps=target_gps, verbose=True)
        success = tester.test_gps_movement()

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
