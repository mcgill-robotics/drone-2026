#!/usr/bin/env python3
"""
Mission 2: Building Perimeter Inspection.

This mission:
1. Takes off to 5m altitude
2. Navigates to a specified building/target location (GPS coordinate)
3. Approaches the building using depth camera
4. Follows the perimeter walls using depth sensor (2m distance)
5. Integrates lidar via OA bridge for obstacle avoidance during strafing
6. Completes full perimeter inspection and lands

Usage:
    python3 test_mission2.py --lat 37.7749 --lon -122.4194 [--sitl] [--dry-run]

Arguments:
    --lat: Building center latitude (WGS84)
    --lon: Building center longitude (WGS84)
    --sitl: Use SITL simulation (UDP connection to PX4)
    --dry-run: Don't arm or fly, just show what would happen
"""

import argparse
import math
import sys
import time
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(SCRIPT_DIR))

import movement
from depth_camera import DepthCameraInterface
from wall_follower import WallFollower, WallFollowingConfig


def main():
    parser = argparse.ArgumentParser(description="Mission 2: Building Perimeter Inspection")
    parser.add_argument("--lat", type=float, required=True, help="Building center latitude (WGS84)")
    parser.add_argument("--lon", type=float, required=True, help="Building center longitude (WGS84)")
    parser.add_argument("--sitl", action="store_true", help="Use SITL simulation")
    parser.add_argument("--dry-run", action="store_true", help="Dry run (no flight)")
    parser.add_argument("--takeoff-alt", type=float, default=5.0, help="Takeoff altitude (m)")
    parser.add_argument(
        "--depth-backend",
        choices=["auto", "d455", "sim"],
        default="auto",
        help="Depth camera preset to use",
    )
    parser.add_argument("--depth-topic", type=str, default=None, help="Override the depth image topic")
    parser.add_argument(
        "--depth-scale-m",
        type=float,
        default=None,
        help="Override integer depth units to metres conversion",
    )
    
    args = parser.parse_args()
    
    if args.dry_run:
        print("[MISSION2] DRY RUN MODE - NO ACTUAL FLIGHT")
        print(f"[MISSION2] Would navigate to: lat={args.lat}, lon={args.lon}")
        return 0
    
    try:
        # =========================================================
        # Phase 1: Boot and OFFBOARD setup
        # =========================================================
        print("\n" + "="*60)
        print("[MISSION2] Phase 1: Booting PX4/MAVROS and entering OFFBOARD")
        print("="*60)
        
        if not movement.setup_environment(sitl=args.sitl, boot=True, wait_seconds=10):
            print("[MISSION2] [ERROR] Failed to boot PX4/MAVROS")
            return 1
        
        # =========================================================
        # Phase 2: Takeoff to 5m
        # =========================================================
        print("\n" + "="*60)
        print("[MISSION2] Phase 2: Takeoff to 5m altitude")
        print("="*60)
        
        if not movement.begin_phase(takeoff_altitude=args.takeoff_alt, arm_timeout=60):
            print("[MISSION2] [ERROR] Failed to reach takeoff altitude")
            movement.cleanup(stop_px4_process=False)
            return 1
        
        # =========================================================
        # Phase 3: Navigate to building
        # =========================================================
        print("\n" + "="*60)
        print(f"[MISSION2] Phase 3: Navigating to building center ({args.lat}, {args.lon})")
        print("="*60)
        
        if not movement.navigate_to_coordinate(args.lat, args.lon, alt_agl=0.0, timeout=120.0):
            print("[MISSION2] [ERROR] Failed to reach building coordinate")
            movement.end_phase(land=True)
            movement.cleanup(stop_px4_process=False)
            return 1
        
        # =========================================================
        # Phase 4: Initialize depth camera
        # =========================================================
        print("\n" + "="*60)
        print("[MISSION2] Phase 4: Initializing depth camera and wall follower")
        print("="*60)
        
        depth_backend = args.depth_backend
        if depth_backend == "auto":
            depth_backend = "sim" if args.sitl else "d455"

        depth_camera = DepthCameraInterface(
            profile=depth_backend,
            topic=args.depth_topic,
            depth_scale_m=args.depth_scale_m,
        )
        
        # TODO: Consider integrating OA bridge for obstacle avoidance
        # The wall_follower has a callback for this, but it needs:
        # 1. OA bridge avoider instance
        # 2. Current position/goal queries
        # 3. Coordinate conversion if using GPS-based OA
        
        wall_config = WallFollowingConfig(
            target_wall_distance_m=2.0,
            distance_tolerance_m=0.3,
            strafe_speed_mps=0.5,
            approach_speed_mps=0.3,
        )
        
        # Simple OA callback: when the wall follower detects an obstacle on the
        # strafing side, this callback performs a short GPS-offset detour using
        # `movement.navigate_to_coordinate`. It returns True when the detour
        # completed successfully (wall follower can resume strafing), or False
        # to indicate no detour was possible.
        def _local_offset_to_gps(current_gps, north_m: float, east_m: float):
            # Very small-angle flat-earth approximation
            lat = float(current_gps["latitude"]) + (north_m / 111320.0)
            lon = float(current_gps["longitude"]) + (east_m / (111320.0 * math.cos(math.radians(float(current_gps["latitude"])))))
            return lat, lon

        def oa_avoider_callback(info: dict) -> bool:
            side = info.get("side", "right")
            try:
                px4 = movement.connect()
                current_pos = px4.get_location()
                current_gps = px4.get_gps_location()
                if not current_pos or not current_gps:
                    print("[MISSION2][OA] Cannot query current position for detour")
                    return False

                # Build a short lateral-forward offset away from the obstacle
                lateral_m = 3.0 if side == "right" else -3.0
                forward_m = 2.0

                target_lat, target_lon = _local_offset_to_gps(current_gps, forward_m, lateral_m)
                print(f"[MISSION2][OA] Detour waypoint: lat={target_lat:.7f}, lon={target_lon:.7f}")

                # Navigate to the detour waypoint (short hop)
                ok = movement.navigate_to_coordinate(target_lat, target_lon, alt_agl=0.0, timeout=30.0)
                if not ok:
                    print("[MISSION2][OA] Detour navigation failed")
                    return False
                # After detour, short hover to let sensors stabilise
                time.sleep(0.5)
                return True
            except Exception as exc:
                print(f"[MISSION2][OA] Avoider callback error: {exc}")
                return False

        wall_follower = WallFollower(
            depth_camera=depth_camera,
            movement_send_velocity=movement.send_velocity_command,
            movement_hold_position=movement.hold_position,
            avoider_callback=oa_avoider_callback,
            config=wall_config,
        )
        
        # =========================================================
        # Phase 5: Inspect building perimeter
        # =========================================================
        print("\n" + "="*60)
        print("[MISSION2] Phase 5: Building perimeter inspection")
        print("="*60 + "\n")
        
        success, reason = wall_follower.inspect_perimeter(max_walls=4, timeout_s=600.0)
        
        if not success:
            print(f"[MISSION2] [WARN] Perimeter inspection ended: {reason}")
        else:
            print(f"[MISSION2] [SUCCESS] Perimeter inspection complete: {reason}")
        
        # =========================================================
        # Phase 6: Land and cleanup
        # =========================================================
        print("\n" + "="*60)
        print("[MISSION2] Phase 6: Landing and cleanup")
        print("="*60)
        
        depth_camera.shutdown()
        
        if not movement.end_phase(land=True):
            print("[MISSION2] [WARN] Landing may have failed")
        
        movement.cleanup(stop_px4_process=False)
        
        print("[MISSION2] Mission complete")
        return 0
    
    except KeyboardInterrupt:
        print("\n[MISSION2] Mission interrupted by user")
        depth_camera.shutdown()
        movement.end_phase(land=True)
        movement.cleanup(stop_px4_process=False)
        return 1
    
    except Exception as e:
        print(f"\n[MISSION2] [ERROR] Unexpected exception: {e}")
        import traceback
        traceback.print_exc()
        try:
            movement.end_phase(land=True)
            movement.cleanup(stop_px4_process=False)
        except Exception:
            pass
        return 1


if __name__ == "__main__":
    sys.exit(main())
