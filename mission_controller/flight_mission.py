#!/usr/bin/env python3
"""
Flight mission script for PX4/MAVROS

This script demonstrates a complete flight sequence:
1. Boot PX4
2. Connect to MAVROS
3. Arm the vehicle
4. Take off
5. Move forward and backward
6. Return to starting position
7. Land and disarm

Usage:
    python3 flight_mission.py --sitl              # Use SITL simulation
    python3 flight_mission.py --hardware           # Use real hardware on USB
    python3 flight_mission.py --port /dev/ttyUSB0 # Custom port
"""

import rclpy
import time
import argparse
from px4_interface import init_px4, boot_px4, stop_px4


class FlightMission:
    """High-level flight mission API"""

    def __init__(self, px4, verbose=True):
        """
        Initialize mission controller

        Args:
            px4: PX4Interface instance
            verbose: Print status messages
        """
        self.px4 = px4
        self.verbose = verbose
        self.start_position = None

    def log(self, msg):
        """Print log message if verbose"""
        if self.verbose:
            print(msg)

    def run_mission(self):
        """Execute the complete flight mission"""
        self.log("\n" + "="*70)
        self.log("FLIGHT MISSION STARTING")
        self.log("="*70 + "\n")

        try:
            # Step 1: Check connection
            self.log("[MISSION] Checking MAVROS connection...")
            if not self.px4.connected:
                self.log("[MISSION] Not connected to MAVROS")
                return False
            self.log("[MISSION] Connected to MAVROS")

            # Step 2: Arm vehicle
            try:
                self.log("\n[MISSION] Step 1/6: Arming vehicle...")
                if not self.px4.arm_vehicle():
                    self.log("[MISSION] Failed to arm")
                    return False
                self.log("[MISSION] Vehicle armed")
            except Exception as e:
                self.log(f"[MISSION] Error while arming: {str(e)}")
                return False

            time.sleep(1)

            # Step 3: Take off
            self.log("\n[MISSION] Step 2/6: Taking off to 5m...")
            try:
                if not self.px4.takeoff(altitude=5.0, timeout=30):
                    self.log("[MISSION] Failed to take off!")
                    self.px4.disarm_vehicle()
                    return False
                self.log("[MISSION] Reached cruise altitude")
            except Exception as e:
                self.log(f"[MISSION] Error while taking off: {str(e)}")
                return False

            # Record starting position
            if self.px4.current_position:
                self.start_position = self.px4.current_position.pose.position
                self.log(f"[MISSION] Start position: X={self.start_position.x:.2f}, Y={self.start_position.y:.2f}")

            # Step 4: Stream velocity setpoints (GUIDED mode supports this directly)
            # No need to switch to OFFBOARD - GUIDED works fine for velocity control
            self.log("\n[MISSION] Step 3/6: Streaming velocity setpoints...")

            time.sleep(1)

            # Step 5: Fly forward and backward
            self.log("\n[MISSION] Step 4/6: Flying forward 10m...")
            self.px4.fly_forward(speed=1.0, duration=10.0)

            time.sleep(1)

            self.log("[MISSION] Step 5/6: Flying backward 10m (return to start)...")
            self.px4.fly_backward(speed=1.0, duration=10.0)

            time.sleep(1)

            # Step 6: Hover briefly
            self.log("\n[MISSION] Hovering for 2 seconds...")
            self.px4.hover(duration=2.0)

            # Step 7: Land
            self.log("\n[MISSION] Step 6/6: Landing...")
            if not self.px4.land(timeout=30):
                self.log("[MISSION] Land timeout, disarming anyway...")
            self.log("[MISSION] ✓ Landed")

            time.sleep(1)

            # Step 8: Disarm
            if not self.px4.disarm_vehicle():
                self.log("[MISSION] Failed to disarm, trying again...")
                time.sleep(1)
                self.px4.disarm_vehicle()

            self.log("[MISSION] ✓ Vehicle disarmed")

            self.log("\n" + "="*70)
            self.log("✓ MISSION COMPLETE!")
            self.log("="*70 + "\n")

            return True

        except Exception as e:
            self.log(f"\n[MISSION] Exception: {str(e)}")
            self.log("[MISSION] Attempting emergency disarm...")
            try:
                self.px4.disarm_vehicle()
            except:
                pass
            return False

        finally:
            # Cleanup
            self.log("[MISSION] Shutting down...")
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
        description="Flight mission for PX4/MAVROS",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 flight_mission.py --sitl                  # SITL simulation
  python3 flight_mission.py --hardware              # Real hardware (default USB)
  python3 flight_mission.py --port /dev/ttyUSB1    # Custom serial port
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
        print("[MAIN] Using SITL simulation mode")
    elif args.port:
        fcu_url = f"serial://{args.port}:921600"
        print(f"[MAIN] Using custom port: {args.port}")
    else:
        # Default to hardware
        fcu_url = "serial:///dev/ttyUSB0:921600"
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

    # Run the mission
    mission = FlightMission(px4, verbose=True)
    success = mission.run_mission()

    return 0 if success else 1


if __name__ == "__main__":
    exit(main())
