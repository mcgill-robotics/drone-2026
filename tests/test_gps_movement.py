#!/usr/bin/env python3
"""
GPS Movement Test

This script demonstrates GPS-based movement in OFFBOARD mode:
1. Boot PX4 and connect to MAVROS
2. Switch to OFFBOARD mode
3. Start background heartbeat stream
4. Wait for manual RC arm
5. Takeoff 5 meters using the known-good local OFFBOARD setpoint path
6. Fly to a target GPS location (if provided)
7. Land and stop background stream

Note: GPS coordinates are absolute lat/lon. The optional target altitude is
relative to the local altitude at launch, not MSL.
"""

import sys
import os

# Add parent directory to path so mission_controller can be imported
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import rclpy
import time
import argparse
import math
from mission_controller.px4_interface import init_px4, boot_px4, stop_px4


EARTH_RADIUS_M = 6_378_137.0
ARRIVAL_TOLERANCE_M = 0.5
SETPOINT_RATE_HZ = 10
ALTITUDE_TOLERANCE_M = 0.5
MOTION_LIMIT_PARAMS = (
    ("MPC_XY_VEL_MAX", 2.0),
    ("MPC_XY_CRUISE", 1.0),
    ("MPC_VEL_MANUAL", 2.0),
    ("MPC_LAND_SPEED", 0.4),
    ("MPC_LAND_CRWL", 0.2),
)


def gps_to_local_offset(origin_lat, origin_lon, target_lat, target_lon):
    """
    Convert a GPS target to an ENU local offset from the origin GPS coordinate.
    Returns (east, north) in meters.
    """
    origin_lat_rad = math.radians(origin_lat)
    d_lat = math.radians(target_lat - origin_lat)
    d_lon = math.radians(target_lon - origin_lon)
    north = d_lat * EARTH_RADIUS_M
    east = d_lon * EARTH_RADIUS_M * math.cos(origin_lat_rad)
    return east, north


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

    def configure_motion_limits(self):
        """Configure conservative PX4 motion limits for this test."""
        self.log("\n[TEST] Setting conservative PX4 motion limits...")
        param_names = [name for name, _value in MOTION_LIMIT_PARAMS]
        """
        if not self.px4.wait_for_params(param_names, timeout=30):
            self.log("[TEST] MAVROS params not ready")
            continue
        """
        for name, value in MOTION_LIMIT_PARAMS:
            if not self.px4.set_param(name, value):
                self.log(f"[TEST] Failed to set {name}")
                continue
        self.log("[TEST] ✓ Motion limits configured")
        return True

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

            if not self.configure_motion_limits():
                return False

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
            self.log("[TEST] ✓ Background stream started (10Hz heartbeat)")

            # Step 4: Wait for arming
            self.log("\n[TEST] Waiting for manual arm (60 seconds)...")
            self.log("[TEST] Please arm the vehicle manually via RC transmitter or QGC")
            if not self.px4.wait_for_arm_with_heartbeat(timeout=60, heartbeat_rate=10):
                self.log("[TEST] Arm timeout - vehicle was not armed")
                return False

            launch_pos = self.px4.get_location()
            if not launch_pos:
                self.log("[TEST] Cannot get launch position before takeoff")
                return False
            launch_alt = launch_pos["z"]

            # Step 5: Takeoff using the same path as test_hover.py. Do not use
            # GPS setpoints here: the PX4 local z setpoint is relative, while
            # GPS altitude is absolute MSL.
            self.log("\n[TEST] Taking off to 5 meters...")
            if not self.px4.takeoff(altitude=5, timeout=30):
                self.log("[TEST] Takeoff failed")
                return False
            self.log("[TEST] ✓ Takeoff complete")

            hover_pos = self.px4.get_location()
            if not hover_pos:
                self.log("[TEST] Cannot get hover position after takeoff")
                return False
            hover_alt = hover_pos["z"]

            # Step 6: Fly to target GPS location
            if not self.target_gps:
                self.log("\n[TEST] No target GPS provided. Hovering for 10 seconds...")
                time.sleep(10)
            else:
                target_lat, target_lon, target_z = self.target_gps
                current_pos = self.px4.get_location()
                current_gps = self.px4.get_gps_location()
                if not current_pos or not current_gps:
                    self.log("[TEST] Cannot get position or GPS location")
                    return False

                east, north = gps_to_local_offset(
                    current_gps["latitude"],
                    current_gps["longitude"],
                    target_lat,
                    target_lon,
                )
                target_x = current_pos["x"] + east
                target_y = current_pos["y"] + north
                target_alt = hover_alt if target_z is None else launch_alt + target_z

                self.log(
                    f"\n[TEST] Flying to target GPS: lat={target_lat:.6f}, "
                    f"lon={target_lon:.6f}, alt={target_alt:.2f}m local"
                )
                if target_z is None:
                    self.log(f"[TEST] Holding post-takeoff altitude: {target_alt:.2f}m")
                else:
                    self.log(
                        f"[TEST] Target altitude: {target_z:.2f}m above launch "
                        f"(local z={target_alt:.2f}m)"
                    )
                self.log(f"[TEST] Offset (E,N): ({east:.2f}m, {north:.2f}m)")
                self.log(
                    f"[TEST] Local target: x={target_x:.2f}, "
                    f"y={target_y:.2f}, z={target_alt:.2f}"
                )

                dt = 1.0 / SETPOINT_RATE_HZ
                start = time.time()
                last_log = 0
                while (time.time() - start) < 60:
                    current_pos = self.px4.get_location()
                    if not current_pos:
                        time.sleep(dt)
                        continue

                    horizontal_distance = math.sqrt(
                        (current_pos["x"] - target_x) ** 2
                        + (current_pos["y"] - target_y) ** 2
                    )

                    self.px4.send_position_setpoint(
                        target_x,
                        target_y,
                        target_alt,
                        yaw_from_direction=True,
                    )
                    rclpy.spin_once(self.px4, timeout_sec=0.0)

                    altitude_error = abs(current_pos["z"] - target_alt)
                    now = time.time()
                    if now - last_log >= 1.0:
                        self.log(
                            f"[TEST] Distance XY: {horizontal_distance:.2f}m, "
                            f"alt error: {altitude_error:.2f}m"
                        )
                        last_log = now

                    if (
                        horizontal_distance <= ARRIVAL_TOLERANCE_M
                        and altitude_error <= ALTITUDE_TOLERANCE_M
                    ):
                        self.log(
                            f"[TEST] ✓ Reached target "
                            f"(xy={horizontal_distance:.2f}m, alt_err={altitude_error:.2f}m)"
                        )
                        break

                    time.sleep(dt)
                else:
                    self.log("[TEST] Timeout reaching target")
                    return False

            # Step 7: Land
            self.log("\n[TEST] Landing...")
            if not self.px4.land(timeout=60):
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
        help='Target GPS coordinates as "lat,lon[,alt_agl]". If altitude is omitted, hold post-takeoff altitude.',
    )
    args = parser.parse_args()

    # Parse target GPS if provided
    target_gps = None
    if args.target:
        try:
            parts = args.target.split(",")
            if len(parts) not in (2, 3):
                raise ValueError("expected lat,lon or lat,lon,alt")
            target_alt = float(parts[2]) if len(parts) == 3 else None
            target_gps = (float(parts[0]), float(parts[1]), target_alt)
            alt_text = f"{target_alt}m above launch" if target_alt is not None else "hold current hover altitude"
            print(f"[MAIN] Target GPS: lat={target_gps[0]}, lon={target_gps[1]}, alt={alt_text}")
        except (ValueError, IndexError):
            print("[MAIN] Invalid target format. Use: 'lat,lon' or 'lat,lon,alt_agl'")
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
            fcu_url = "udp://:14540@localhost:14580"
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
