#!/usr/bin/env python3
"""
Depth camera interface for building perimeter inspection.

Provides obstacle detection and wall-following assistance via depth sensor.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

try:
    import numpy as np
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
except ImportError:
    np = None
    rclpy = None
    qos_profile_sensor_data = 10
    Image = None
    CvBridge = None

DEPTH_CAMERA_TOPIC = "/camera/aligned_depth_to_color/image_raw"
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

    def __init__(self, node=None, topic: str = DEPTH_CAMERA_TOPIC):
        """Initialize depth camera interface."""
        self.current_reading: Optional[DepthReading] = None
        self._lock = threading.Lock()
        self._node = node
        self._subscriber = None
        self._spin_thread = None
        self._owns_node = node is None
        self._max_distance_m = 10.0  # Maximum sensed distance
        self._min_distance_m = 0.1   # Minimum sensed distance (blind spot)

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
                topic,
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

            print(f"[DEPTH] Subscribed to depth camera topic: {topic}")
        except Exception as exc:
            self._subscriber = None
            print(f"[DEPTH] Failed to initialize depth camera subscriber: {exc}")

    def _depth_callback(self, msg):
        """
        Process RealSense D455 depth image message.
        
        Extracts center/left/right distances and updates current_reading.
        RealSense publishes uint16 depth in millimeters.
        """
        if np is None or CvBridge is None:
            return
        
        try:
            bridge = CvBridge()
            
            # Convert ROS Image to numpy (uint16, depth in mm)
            depth_array = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            height, width = depth_array.shape
            
            # Convert mm to meters, filter invalid readings (0 = no data)
            depth_m = depth_array.astype(float) / 1000.0
            depth_m[depth_array == 0] = np.nan
            
            # Extract regions: center 40%, left 30%, right 30%
            center_col_start = int(width * 0.30)
            center_col_end = int(width * 0.70)
            left_col_start = int(width * 0.0)
            left_col_end = int(width * 0.30)
            right_col_start = int(width * 0.70)
            right_col_end = int(width * 1.0)
            
            # Use middle rows (ignore top/bottom edges due to camera angle)
            row_start = int(height * 0.25)
            row_end = int(height * 0.75)
            
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
