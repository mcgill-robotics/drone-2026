#!/usr/bin/env python3
"""
Depth camera interface for building perimeter inspection.

Provides obstacle detection and wall-following assistance via depth sensor.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Optional

try:
    import numpy as np
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
except Exception:
    np = None
    rclpy = None
    qos_profile_sensor_data = 10
    Image = None
    CvBridge = None

DEPTH_CAMERA_TOPIC = "/camera/aligned_depth_to_color/image_raw"
SIM_DEPTH_CAMERA_TOPIC = "/camera/depth/image_raw"


@dataclass(frozen=True)
class DepthCameraProfile:
    """Configuration for a depth-camera source."""

    name: str
    topic: str
    depth_scale_m: float | None
    center_fraction: tuple[float, float] = (0.30, 0.70)
    left_fraction: tuple[float, float] = (0.00, 0.30)
    right_fraction: tuple[float, float] = (0.70, 1.00)
    row_fraction: tuple[float, float] = (0.25, 0.75)


D455_DEPTH_PROFILE = DepthCameraProfile(
    name="d455",
    topic=DEPTH_CAMERA_TOPIC,
    depth_scale_m=0.001,
)

SIM_DEPTH_PROFILE = DepthCameraProfile(
    name="sim",
    topic=SIM_DEPTH_CAMERA_TOPIC,
    depth_scale_m=1.0,
)
# TODO: Process depth frame to extract relevant measurements (center, left, right)
# TODO: Handle depth frame coordinate system and camera calibration


@dataclass
class DepthReading:
    """Depth camera reading at a single moment."""
    timestamp: float
    center_distance_m: float    # Distance straight ahead (meters)
    left_distance_m: float      # Distance to the left
    right_distance_m: float     # Distance to the right
    center_valid: bool          # Whether center reading is valid
    left_valid: bool
    right_valid: bool


class DepthCameraInterface:
    """
    Interface for depth camera on the drone.
    
    Subscribes to depth camera topic and provides methods to query:
    - Center distance (wall ahead?)
    - Left/right distances (obstacles during strafing?)
    - Change in distance (wall end detection)
    
    TODO: Implement actual ROS 2 depth camera subscriber and frame processing.
    """

    def __init__(
        self,
        node=None,
        topic: str | None = None,
        profile: str | DepthCameraProfile = D455_DEPTH_PROFILE,
        depth_scale_m: float | None = None,
    ):
        """Initialize depth camera interface."""
        self.current_reading: Optional[DepthReading] = None
        self._lock = threading.Lock()
        self._node = node
        self._subscriber = None
        self._spin_thread = None
        self._owns_node = node is None
        self._max_distance_m = 10.0  # Maximum sensed distance
        self._min_distance_m = 0.1   # Minimum sensed distance (blind spot)
        self._cv_bridge = None

        if isinstance(profile, str):
            profile_map = {
                "d455": D455_DEPTH_PROFILE,
                "realsense": D455_DEPTH_PROFILE,
                "sim": SIM_DEPTH_PROFILE,
                "gz_x500_depth": SIM_DEPTH_PROFILE,
            }
            profile = profile_map.get(profile.lower(), D455_DEPTH_PROFILE)

        self._profile = profile
        self._depth_scale_m = depth_scale_m if depth_scale_m is not None else profile.depth_scale_m
        self._topic = topic or profile.topic

        if rclpy is None or Image is None:
            print("[DEPTH] ROS 2 depth camera dependencies unavailable; subscriber disabled")
            return

        try:
            if self._node is None:
                if not rclpy.ok():
                    rclpy.init(args=None)
                self._node = rclpy.create_node("depth_camera_interface")

            self._subscriber = self._node.create_subscription(
                Image,
                self._topic,
                self._depth_callback,
                qos_profile_sensor_data,
            )

            if self._owns_node:
                self._spin_thread = threading.Thread(
                    target=rclpy.spin,
                    args=(self._node,),
                    daemon=True,
                )
                self._spin_thread.start()

            print(f"[DEPTH] Subscribed to depth camera topic: {self._topic} ({self._profile.name})")
        except Exception as exc:
            self._subscriber = None
            print(f"[DEPTH] Failed to initialize depth camera subscriber: {exc}")

    def _depth_to_meters(self, msg):
        # Must have numpy available to decode image bytes
        if np is None:
            return None

        # Prefer CvBridge when available for encoding handling
        if CvBridge is not None:
            try:
                if self._cv_bridge is None:
                    self._cv_bridge = CvBridge()
                depth_array = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                depth_array = np.asarray(depth_array)
                if depth_array.ndim == 3 and depth_array.shape[2] == 1:
                    depth_array = depth_array[:, :, 0]

                if np.issubdtype(depth_array.dtype, np.floating):
                    depth_m = depth_array.astype(np.float32, copy=False)
                else:
                    scale_m = self._depth_scale_m if self._depth_scale_m is not None else 1.0
                    depth_m = depth_array.astype(np.float32) * float(scale_m)

                depth_m = np.asarray(depth_m)
                depth_m[depth_array == 0] = np.nan
                return depth_m
            except Exception:
                # Fall through to manual decode
                pass

        # Manual decode fallback (no CvBridge). Support common encodings.
        try:
            encoding = (getattr(msg, "encoding", "") or "").lower()
            height = int(msg.height)
            width = int(msg.width)
            is_big = bool(getattr(msg, "is_bigendian", False))
            data = msg.data

            # Map encoding -> (numpy dtype, scale applies?)
            if "16" in encoding or encoding in {"16uc1", "mono16", "z16"}:
                dtype = np.dtype(">u2") if is_big else np.dtype("<u2")
                arr = np.frombuffer(data, dtype=dtype)
                if arr.size != height * width:
                    arr = arr[: height * width]
                arr = arr.reshape((height, width))
                scale_m = self._depth_scale_m if self._depth_scale_m is not None else 1.0
                depth_m = arr.astype(np.float32) * float(scale_m)

            elif "32f" in encoding or "float32" in encoding or encoding in {"32fc1"}:
                dtype = np.dtype(">f4") if is_big else np.dtype("<f4")
                arr = np.frombuffer(data, dtype=dtype)
                if arr.size != height * width:
                    arr = arr[: height * width]
                arr = arr.reshape((height, width))
                depth_m = arr.astype(np.float32)

            elif "8u" in encoding or encoding in {"mono8", "rgb8", "bgr8"}:
                # Unusual for depth, but handle gracefully
                dtype = np.dtype("u1")
                arr = np.frombuffer(data, dtype=dtype)
                if arr.size != height * width:
                    arr = arr[: height * width]
                arr = arr.reshape((height, width))
                scale_m = self._depth_scale_m if self._depth_scale_m is not None else 1.0
                depth_m = arr.astype(np.float32) * float(scale_m)

            else:
                # Unknown encoding — cannot decode without CvBridge
                return None

            depth_m = np.asarray(depth_m)
            # Treat zero as missing measurement
            try:
                depth_m[np.asarray(depth_m) == 0] = np.nan
            except Exception:
                pass
            return depth_m
        except Exception as exc:
            print(f"[DEPTH] Manual decode failed: {exc}")
            return None

    def _depth_callback(self, msg):
        """
        Process a depth image message from either the real D455 or the sim camera.
        
        Extracts center/left/right distances and updates current_reading.
        """
        if np is None or CvBridge is None:
            return
        
        try:
            depth_m = self._depth_to_meters(msg)
            if depth_m is None:
                return

            depth_array = np.asarray(depth_m)
            height, width = depth_array.shape

            # Extract regions from the lower-middle part of the frame where the wall is most visible.
            center_col_start = int(width * self._profile.center_fraction[0])
            center_col_end = int(width * self._profile.center_fraction[1])
            left_col_start = int(width * self._profile.left_fraction[0])
            left_col_end = int(width * self._profile.left_fraction[1])
            right_col_start = int(width * self._profile.right_fraction[0])
            right_col_end = int(width * self._profile.right_fraction[1])

            row_start = int(height * self._profile.row_fraction[0])
            row_end = int(height * self._profile.row_fraction[1])
            
            center_region = depth_m[row_start:row_end, center_col_start:center_col_end]
            left_region = depth_m[row_start:row_end, left_col_start:left_col_end]
            right_region = depth_m[row_start:row_end, right_col_start:right_col_end]
            
            # Compute median distance for each region (robust to outliers/noise)
            center_dist = np.nanmedian(center_region)
            left_dist = np.nanmedian(left_region)
            right_dist = np.nanmedian(right_region)
            
            # Check if readings are valid (not NaN and within range)
            center_valid = (
                not np.isnan(center_dist) and
                self._min_distance_m <= center_dist <= self._max_distance_m
            )
            left_valid = (
                not np.isnan(left_dist) and
                self._min_distance_m <= left_dist <= self._max_distance_m
            )
            right_valid = (
                not np.isnan(right_dist) and
                self._min_distance_m <= right_dist <= self._max_distance_m
            )
            
            # Replace NaN with max distance for missing data
            if not center_valid:
                center_dist = self._max_distance_m
            if not left_valid:
                left_dist = self._max_distance_m
            if not right_valid:
                right_dist = self._max_distance_m
            
            # Create reading and update thread-safely
            reading = DepthReading(
                timestamp=time.time(),
                center_distance_m=float(center_dist),
                left_distance_m=float(left_dist),
                right_distance_m=float(right_dist),
                center_valid=center_valid,
                left_valid=left_valid,
                right_valid=right_valid,
            )
            
            with self._lock:
                self.current_reading = reading
                
        except Exception as exc:
            print(f"[DEPTH] Error processing depth frame: {exc}")

    def get_reading(self) -> Optional[DepthReading]:
        """Get the latest depth camera reading."""
        with self._lock:
            return self.current_reading

    def is_wall_ahead(self, threshold_m: float = 2.0) -> bool:
        """
        Check if there's a wall directly ahead (within threshold distance).
        
        Args:
            threshold_m: Distance threshold in meters (default 2.0m)
        
        Returns:
            True if center distance is within threshold and valid
        """
        reading = self.get_reading()
        if reading is None or not reading.center_valid:
            return False
        return reading.center_distance_m <= threshold_m

    def get_wall_distance(self) -> Optional[float]:
        """Get distance to wall ahead. Returns None if no valid reading."""
        reading = self.get_reading()
        if reading is None or not reading.center_valid:
            return None
        return reading.center_distance_m

    def has_obstacle_on_side(self, side: str, threshold_m: float = 1.5) -> bool:
        """
        Check if there's an obstacle on the specified side.
        
        Args:
            side: 'left' or 'right'
            threshold_m: Distance threshold for obstacle detection (default 1.5m)
        
        Returns:
            True if obstacle detected on that side
        """
        reading = self.get_reading()
        if reading is None:
            return False
        
        if side.lower() == "left":
            return reading.left_valid and reading.left_distance_m <= threshold_m
        elif side.lower() == "right":
            return reading.right_valid and reading.right_distance_m <= threshold_m
        else:
            raise ValueError(f"side must be 'left' or 'right', got {side!r}")

    def get_side_distance(self, side: str) -> Optional[float]:
        """Get distance to obstacle on specified side. Returns None if invalid."""
        reading = self.get_reading()
        if reading is None:
            return None
        
        if side.lower() == "left":
            return reading.left_distance_m if reading.left_valid else None
        elif side.lower() == "right":
            return reading.right_distance_m if reading.right_valid else None
        else:
            raise ValueError(f"side must be 'left' or 'right', got {side!r}")

    def is_wall_ending(self, history_depth: int = 5) -> bool:
        """
        Detect if the wall is ending based on sudden distance change.
        
        TODO: Implement reading history buffer and change detection.
        
        Args:
            history_depth: Number of previous readings to consider
        
        Returns:
            True if recent readings show wall disappearing
        """
        # TODO: Keep history of readings and detect variance
        # If readings suddenly jump from 2.0m to 10.0m, wall is ending
        return False

    def get_distance_trend(self, side: str) -> Optional[str]:
        """
        Get trend of distance readings on specified side.
        
        TODO: Implement trend analysis from reading history.
        
        Returns:
            'increasing', 'decreasing', 'stable', or None if no data
        """
        # TODO: Compare recent readings to detect trend
        return None

    def shutdown(self):
        """Cleanup depth camera subscriber."""
        if self._subscriber is not None:
            # TODO: Destroy ROS 2 subscriber
            self._subscriber = None
