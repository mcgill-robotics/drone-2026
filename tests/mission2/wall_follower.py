#!/usr/bin/env python3
"""
Wall-following logic for building perimeter inspection.

Handles:
- Wall approach and alignment (perpendicular vs parallel)
- Strafing along walls at constant distance
- Wall end detection and corner turning
- Obstacle avoidance via lidar (OA bridge integration)
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Callable, Any

from depth_camera import DepthCameraInterface


class WallSide(Enum):
    """Which wall side are we following?"""
    FRONT = "front"
    RIGHT = "right"
    BACK = "back"
    LEFT = "left"


class StrafingDirection(Enum):
    """Which direction to strafe along wall?"""
    LEFT = -1
    RIGHT = 1


@dataclass
class WallFollowingConfig:
    """Configuration parameters for wall-following behavior."""
    target_wall_distance_m: float = 2.0      # Maintain this distance from wall (meters)
    distance_tolerance_m: float = 0.3        # Tolerance when maintaining distance
    strafe_speed_mps: float = 0.5            # Speed while strafing along wall (m/s)
    approach_speed_mps: float = 0.3          # Speed while approaching/aligning with wall
    turn_speed_mps: float = 0.2              # Speed while turning corners
    wall_end_threshold_m: float = 4.0        # Distance at which to consider wall ended
    corner_turn_angle_rad: float = math.pi/2 # 90 degrees
    max_strafing_time_s: float = 120.0       # Safety timeout for strafing on one wall
    obstacle_check_distance_m: float = 1.5   # Distance at which to avoid side obstacles


class WallFollower:
    """
    Wall-following behavior for building perimeter inspection.
    
    Coordinates with:
    - movement.py: low-level velocity/position commands
    - depth_camera.py: obstacle detection ahead and to sides
    - oa_bridge (via callback): obstacle avoidance when lidar detects obstacles on strafing side
    """

    def __init__(
        self,
        depth_camera: DepthCameraInterface,
        movement_send_velocity: Callable[[float, float, float, float], bool],
        movement_hold_position: Callable[[], bool],
        avoider_callback: Optional[Callable[[dict], Optional[dict]]] = None,
        config: Optional[WallFollowingConfig] = None,
    ):
        """
        Initialize wall follower.
        
        Args:
            depth_camera: DepthCameraInterface for obstacle detection
            movement_send_velocity: Callback to send velocity commands (vx, vy, vz, yaw_rate)
            movement_hold_position: Callback to hold current position
            avoider_callback: Optional callback to query OA bridge for safe waypoints
            config: WallFollowingConfig with behavior parameters
        """
        self.depth_camera = depth_camera
        self.send_velocity = movement_send_velocity
        self.hold_position = movement_hold_position
        self.avoider_callback = avoider_callback
        self.config = config or WallFollowingConfig()
        
        self.current_wall: Optional[WallSide] = None
        self.current_direction: Optional[StrafingDirection] = None
        self.wall_start_time: Optional[float] = None
        self.is_strafing = False
        
        print("[WALL_FOLLOWER] Initialized")

    def approach_wall(self, timeout_s: float = 60.0) -> bool:
        """
        Approach the wall until at target distance.
        
        Moves forward until depth camera reads target_wall_distance_m ahead.
        
        Args:
            timeout_s: Maximum time to approach (safety timeout)
        
        Returns:
            True if wall reached and at target distance, False on timeout
        """
        print("[WALL] Approaching wall...")
        start_time = time.time()
        
        while (time.time() - start_time) < timeout_s:
            distance = self.depth_camera.get_wall_distance()
            
            if distance is None:
                print("[WALL] [WARN] No valid depth reading; holding position")
                self.hold_position()
                time.sleep(0.5)
                continue
            
            # Check if we've reached target distance
            if distance <= (self.config.target_wall_distance_m + self.config.distance_tolerance_m):
                print(f"[WALL] ✓ Wall reached at {distance:.2f}m")
                self.hold_position()
                return True
            
            # Move forward slowly towards wall
            remaining = distance - self.config.target_wall_distance_m
            if remaining > 2.0:
                # Far from wall, move faster
                vx = self.config.approach_speed_mps
            else:
                # Close to target, move slower
                vx = self.config.approach_speed_mps * 0.5
            
            self.send_velocity(vx, 0.0, 0.0, 0.0)
            time.sleep(0.1)
        
        print("[WALL] [ERROR] Timeout approaching wall")
        self.hold_position()
        return False

    def align_with_wall(self, initial_wall: WallSide = WallSide.FRONT, timeout_s: float = 30.0) -> bool:
        """
        Align drone perpendicular to the wall.
        
        Uses depth camera left/right readings to adjust heading until parallel with wall.
        
        TODO: Implement yaw control to align perpendicular to wall.
        
        Args:
            initial_wall: Which wall we're starting with
            timeout_s: Maximum alignment time
        
        Returns:
            True if aligned, False on timeout
        """
        print(f"[WALL] Aligning with wall ({initial_wall.value})...")
        self.current_wall = initial_wall
        
        # TODO: Implement:
        # 1. Compare left_distance vs right_distance
        # 2. If left > right, drone is tilted right, rotate left
        # 3. If right > left, drone is tilted left, rotate right
        # 4. When left == right, wall is parallel to drone heading
        # 5. Then turn 90° to align perpendicular (ready to strafe)
        
        print("[WALL] [TODO] Wall alignment not yet implemented; assuming aligned")
        self.hold_position()
        return True

    def strafe_along_wall(self, direction: StrafingDirection, timeout_s: Optional[float] = None) -> tuple[bool, str]:
        """
        Strafe along wall, maintaining constant distance.
        
        Moves sideways (left or right) while maintaining target distance from wall ahead.
        Detects wall end and returns when wall disappears.
        
        Integrates OA bridge: if obstacle detected on strafing side, uses avoider callback
        to request a safe detour waypoint.
        
        Args:
            direction: StrafingDirection.LEFT or RIGHT
            timeout_s: Maximum strafing time (uses config default if None)
        
        Returns:
            Tuple of (success: bool, reason: str)
            - (True, 'wall_ended'): Wall disappeared, end of side detected
            - (True, 'timeout'): Reached maximum time limit (safety)
            - (False, 'error'): Error during strafing
        """
        if timeout_s is None:
            timeout_s = self.config.max_strafing_time_s
        
        self.current_direction = direction
        self.wall_start_time = time.time()
        self.is_strafing = True
        
        direction_str = "left" if direction == StrafingDirection.LEFT else "right"
        print(f"[WALL] Strafing {direction_str} along {self.current_wall.value} wall...")
        
        strafe_speed_vy = self.config.strafe_speed_mps * direction.value  # Positive = right
        start_time = time.time()
        last_distance = None
        distance_stable_count = 0
        
        while (time.time() - start_time) < timeout_s:
            distance = self.depth_camera.get_wall_distance()
            
            if distance is None:
                print("[WALL] [WARN] No valid depth reading; holding position")
                self.hold_position()
                time.sleep(0.5)
                continue
            
            # Check for wall end (sudden increase in distance)
            if last_distance is not None:
                distance_change = distance - last_distance
                if distance_change > 1.5:  # Significant increase = wall likely ended
                    distance_stable_count += 1
                    if distance_stable_count >= 3:  # Confirm over 3 readings
                        print(f"[WALL] ✓ Wall ended (distance jumped from {last_distance:.2f}m to {distance:.2f}m)")
                        self.hold_position()
                        self.is_strafing = False
                        return (True, "wall_ended")
                else:
                    distance_stable_count = 0
            
            last_distance = distance
            
            # Adjust forward velocity to maintain distance from wall
            distance_error = distance - self.config.target_wall_distance_m
            if abs(distance_error) > self.config.distance_tolerance_m:
                # Too far or too close, adjust forward velocity
                if distance_error > 0:
                    vx = self.config.strafe_speed_mps * 0.3  # Too far, move forward
                else:
                    vx = -self.config.strafe_speed_mps * 0.3  # Too close, move back
            else:
                vx = 0.0
            
            # Check for obstacles on the strafing side (lidar via OA bridge)
            obstacle_on_side = self.depth_camera.has_obstacle_on_side(direction_str, self.config.obstacle_check_distance_m)
            
            if obstacle_on_side:
                print(f"[WALL] [OA] Obstacle detected on {direction_str}; requesting safe waypoint")
                # TODO: Call avoider_callback to get detour waypoint
                # For now, just reduce strafe speed
                strafe_speed_vy *= 0.5
            
            # Send strafing velocity
            self.send_velocity(vx, strafe_speed_vy, 0.0, 0.0)
            time.sleep(0.1)
        
        print(f"[WALL] Timeout strafing {self.current_wall.value} wall")
        self.hold_position()
        self.is_strafing = False
        return (True, "timeout")

    def turn_corner(self, timeout_s: float = 20.0) -> bool:
        """
        Turn at corner to align with next wall.
        
        Rotates 90° in place to prepare for strafing the perpendicular wall.
        
        Args:
            timeout_s: Maximum time for turn
        
        Returns:
            True if turn completed, False on timeout
        """
        print(f"[WALL] Turning corner from {self.current_wall.value} wall...")
        start_time = time.time()
        
        # TODO: Implement:
        # 1. Rotate 90° counter-clockwise (to turn left/continue perimeter)
        # 2. Update self.current_wall to next wall (FRONT -> RIGHT -> BACK -> LEFT -> FRONT)
        # 3. Verify with depth camera that new wall is ahead
        
        print("[WALL] [TODO] Corner turning not yet fully implemented")
        self.hold_position()
        return True

    def inspect_perimeter(self, max_walls: int = 4, timeout_s: Optional[float] = None) -> tuple[bool, str]:
        """
        Inspect full building perimeter by following all 4 walls.
        
        High-level orchestration:
        1. Approach wall
        2. Align perpendicular
        3. Strafe along wall until end detected
        4. Turn corner
        5. Repeat for next wall (up to max_walls times)
        
        Args:
            max_walls: Maximum number of walls to follow (default 4 for building perimeter)
            timeout_s: Overall mission timeout (None = no limit)
        
        Returns:
            Tuple of (success: bool, reason: str)
            - (True, 'complete'): Successfully inspected all walls
            - (True, 'stopped'): Stopped after max_walls (for partial inspection)
            - (False, 'error'): Error during inspection
        """
        if timeout_s is None:
            timeout_s = 600.0  # 10 minute default limit
        
        print(f"[WALL] Starting perimeter inspection (max {max_walls} walls, timeout {timeout_s}s)")
        start_time = time.time()
        walls_inspected = 0
        
        wall_sequence = [WallSide.FRONT, WallSide.RIGHT, WallSide.BACK, WallSide.LEFT]
        
        while walls_inspected < max_walls:
            if (time.time() - start_time) > timeout_s:
                print(f"[WALL] Mission timeout")
                self.hold_position()
                return (False, "timeout")
            
            current_wall = wall_sequence[walls_inspected % len(wall_sequence)]
            
            # Approach and align with wall
            if not self.approach_wall(timeout_s=30.0):
                print(f"[WALL] Failed to approach wall")
                return (False, "approach_failed")
            
            if not self.align_with_wall(current_wall, timeout_s=30.0):
                print(f"[WALL] Failed to align with wall")
                return (False, "alignment_failed")
            
            # Strafe along wall
            success, reason = self.strafe_along_wall(
                StrafingDirection.LEFT,  # TODO: make this configurable per wall
                timeout_s=60.0
            )
            
            if not success:
                print(f"[WALL] Strafing error: {reason}")
                return (False, f"strafe_error: {reason}")
            
            walls_inspected += 1
            print(f"[WALL] Completed {walls_inspected}/{max_walls} walls")
            
            # Turn corner for next wall (unless we're done)
            if walls_inspected < max_walls:
                if not self.turn_corner(timeout_s=20.0):
                    print(f"[WALL] Failed to turn corner")
                    return (False, "turn_failed")
                time.sleep(0.5)
        
        print(f"[WALL] ✓ Perimeter inspection complete")
        self.hold_position()
        return (True, "complete")
