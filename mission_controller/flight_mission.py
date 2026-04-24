#!/usr/bin/env python3
"""
Flight mission script for PX4/MAVROS

This script demonstrates a complete autonomous flight sequence:
1. Boot PX4
2. Connect to MAVROS
3. Wait for drone to be armed manually (via RC remote or button)
4. Take off
5. Move forward and backward
6. Return to starting position
7. Land and disarm

WORKFLOW:
1. SSH into Jetson and run: python3 flight_mission.py --hardware
2. Script will output: "Waiting for drone to be armed..."
3. You can now disconnect the ethernet cable
4. Arm the drone manually (via RC remote switch or button)
5. Script will detect arm and automatically execute the mission

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

    def __init__(self, px4, verbose=True, auto_arm=False):
        """
        Initialize mission controller

        Args:
            px4: PX4Interface instance
            verbose: Print status messages
            auto_arm: Automatically arm the drone (default: wait for manual arm)
        """
        self.px4 = px4
        self.verbose = verbose
        self.auto_arm = auto_arm
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

            # Step 2: Wait for drone to be armed (manual arming via RC or other method)
            self.log("\n[MISSION] Waiting for drone to be armed...")
            
            if self.auto_arm:
                # Auto-arm mode
                self.log("[MISSION] Attempting automatic arm...")
                if not self.px4.arm_vehicle():
                    self.log("[MISSION] Failed to arm via API")
                    return False
                self.log("[MISSION] ✓ Vehicle armed (automatic)")
            else:
                # Wait for manual arm
                self.log("[MISSION] You can now disconnect the ethernet cable")
                self.log("[MISSION] Arm the drone manually (via RC remote or button)")
                
                arm_timeout = 120  # Wait up to 2 minutes for arming
                start_wait = time.time()
                while not self.px4.is_armed() and (time.time() - start_wait) < arm_timeout:
                    rclpy.spin_once(self.px4, timeout_sec=0.1)
                    time.sleep(0.5)
                    elapsed = int(time.time() - start_wait)
                    if elapsed % 5 == 0:
                        print(f"[MISSION] Waiting... ({elapsed}s)")
                
                if not self.px4.is_armed():
                    self.log("[MISSION] Timeout waiting for arm. Exiting.")
                    return False
                
                self.log("[MISSION] ✓ Drone armed! Starting mission...")

            time.sleep(1)

            # Step 3: Take off
            self.log("\n[MISSION] Step 1/5: Taking off to 5m...")
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
            self.log("\n[MISSION] Step 2/5: Streaming velocity setpoints...")

            time.sleep(1)

            # Step 5: Fly forward and backward
            self.log("\n[MISSION] Step 3/5: Flying forward 10m...")
            self.px4.fly_forward(speed=1.0, duration=10.0)

            time.sleep(1)

            self.log("[MISSION] Step 4/5: Flying backward 10m (return to start)...")
            self.px4.fly_backward(speed=1.0, duration=10.0)

            time.sleep(1)

            # Step 6: Hover briefly
            self.log("\n[MISSION] Hovering for 2 seconds...")
            self.px4.hover(duration=2.0)

            # Step 7: Land
            self.log("\n[MISSION] Step 5/5: Landing...")
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
  python3 flight_mission.py --hardware              # Real hardware (default USB) - waits for manual arm
  python3 flight_mission.py --hardware --auto-arm  # Real hardware - automatically arms
  python3 flight_mission.py --port /dev/ttyUSB1    # Custom serial port - waits for manual arm
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
    parser.add_argument(
        "--auto-arm",
        action="store_true",
        help="Automatically arm drone (default: wait for manual arm)"
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

    # Run the mission
    mission = FlightMission(px4, verbose=True, auto_arm=args.auto_arm)
    success = mission.run_mission()

    return 0 if success else 1


if __name__ == "__main__":
    exit(main())
