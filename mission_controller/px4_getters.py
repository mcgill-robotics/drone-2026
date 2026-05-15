"""
PX4 telemetry / getter interface using MAVROS and ROS 2

Responsibilities:
- Subscribe to MAVROS topics
- Store the latest telemetry data locally
- Provide read-only APIs (getter methods)

This file does NOT directly command the drone.
It focuses on reading data that is already being published by MAVROS.
"""

# NumPy 2.0+ compatibility patch for compiled modules built with NumPy 1.x
import sys
try:
    import numpy as np
    if hasattr(np, '__version__') and np.__version__.startswith('2.'):
        # Suppress _ARRAY_API attribute errors from NumPy 1.x compiled modules
        import warnings
        warnings.filterwarnings('ignore', category=AttributeError, message='.*_ARRAY_API.*')
except Exception:
    pass

import time
import math
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import numpy as np

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import (
    State,
    ExtendedState,
    HomePosition,
    Altitude,
    GPSRAW,
    GPSRTK,
    RTKBaseline,
)
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, CommandHome, ParamSet
from sensor_msgs.msg import BatteryState, NavSatFix, Imu
from types import Point


class PX4Getters(Node):
    """
    Wrapper for receiving telemetry from PX4 via MAVROS.

    Requires MAVROS to be running and connected to the flight controller.
    This class is responsible for:
    - creating subscriptions
    - caching the newest messages
    - exposing getter APIs for the rest of the system
    """

    def __init__(self, node_name="ardupilot_interface", namespace="mavros"):
        """
        Initialize MAVROS getter interface

        Args:
            node_name: ROS 2 node name
            namespace: MAVROS namespace (default "mavros")
        """
        super().__init__(node_name)

        #these are managed locally. namespace is used to create the topic paths
        self.namespace = namespace
        self.connected = False
        self.timeout = 30

        # =========================================================
        # Core telemetry storage
        # These variables hold the latest received message for each
        # subscribed topic.
        # =========================================================

        # the next variables are pulled by PX4 / MAVROS messages
        self.current_state = None
        self.current_position = None
        self.battery_status = None
        self.current_gps = None
        self.current_attitude = None
        self.extended_state = None
        self.home_position = None
        self.current_velocity = None

        # extra telemetry storage for the additional MAVROS topics
        self.current_odometry = None
        self.current_velocity_body = None
        self.current_velocity_local = None
        self.current_altitude = None
        self.current_gps_raw_1 = None
        self.current_gps_raw_2 = None
        self.current_gps_velocity = None
        self.current_gps_raw_fix = None
        self.current_rtk_1 = None
        self.current_rtk_2 = None
        self.current_rtk_baseline = None

        # =========================================================
        # First-message flags
        # These are only used for one-time debug prints when we first
        # start receiving a given telemetry stream.
        # =========================================================
        self._seen_position = False
        self._seen_battery = False
        self._seen_gps = False
        self._seen_attitude = False
        self._seen_extended_state = False
        self._seen_home_position = False
        self._seen_velocity = False
        self._seen_odometry = False
        self._seen_velocity_body = False
        self._seen_velocity_local = False
        self._seen_altitude = False
        self._seen_gps_raw_1 = False
        self._seen_gps_raw_2 = False
        self._seen_gps_velocity = False
        self._seen_gps_raw_fix = False
        self._seen_rtk_1 = False
        self._seen_rtk_2 = False
        self._seen_rtk_baseline = False



        # =========================================================
        # Timestamp trackers for debugging
        # These let us detect "NO DATA" vs "STALE DATA".
        # =========================================================
        self._last_state_time = None
        self._last_position_time = None
        self._last_battery_time = None
        self._last_gps_time = None
        self._last_attitude_time = None
        self._last_extended_state_time = None
        self._last_home_position_time = None
        self._last_velocity_time = None
        self._last_odometry_time = None
        self._last_velocity_body_time = None
        self._last_velocity_local_time = None
        self._last_altitude_time = None
        self._last_gps_raw_1_time = None
        self._last_gps_raw_2_time = None
        self._last_gps_velocity_time = None
        self._last_gps_raw_fix_time = None
        self._last_rtk_1_time = None
        self._last_rtk_2_time = None
        self._last_rtk_baseline_time = None

        # =========================================================
        # ROS2 subscriptions
        # callbacks are constantly called by the ros node when new
        # messages are received on the subscribed topics
        # =========================================================

        self.state_sub = self.create_subscription(
            State,  # mavros msg
            f"/{namespace}/state",
            # this callback is a function that is continuously called from the ros node
            self._state_callback,
            10  # queue size; older messages may be dropped if needed
        )

        self.position_sub = self.create_subscription(
            PoseStamped,
            f"/{namespace}/local_position/pose",
            self._position_callback,
            # default queue size is 5, for sensor data, drops older msg if not needed
            qos_profile_sensor_data
        )

        self.odometry_sub = self.create_subscription(
            Odometry,
            f"/{namespace}/local_position/odom",
            self._odometry_callback,
            qos_profile_sensor_data
        )

        self.battery_sub = self.create_subscription(
            BatteryState,
            f"/{namespace}/battery",
            self._battery_callback,
            qos_profile_sensor_data
        )

        self.global_position_sub = self.create_subscription(
            NavSatFix,
            f"/{namespace}/global_position/global",
            self._global_position_callback,
            qos_profile_sensor_data
        )

        self.global_position_raw_fix_sub = self.create_subscription(
            NavSatFix,
            f"/{namespace}/global_position/raw/fix",
            self._global_position_raw_fix_callback,
            qos_profile_sensor_data
        )

        self.attitude_sub = self.create_subscription(
            Imu,
            f"/{namespace}/imu/data",
            self._attitude_callback,
            qos_profile_sensor_data
        )

        self.extended_state_sub = self.create_subscription(
            ExtendedState,
            f"/{namespace}/extended_state",
            self._extended_state_callback,
            qos_profile_sensor_data
        )

        self.home_position_sub = self.create_subscription(
            HomePosition,
            f"/{namespace}/home_position/home",
            self._home_position_callback,
            qos_profile_sensor_data
        )

        self.velocity_sub = self.create_subscription(
            TwistStamped,
            f"/{namespace}/local_position/velocity_local",
            self._velocity_callback,
            qos_profile_sensor_data
        )

        self.velocity_local_sub = self.create_subscription(
            TwistStamped,
            f"/{namespace}/local_position/velocity_local",
            self._velocity_local_callback,
            qos_profile_sensor_data
        )

        self.velocity_body_sub = self.create_subscription(
            TwistStamped,
            f"/{namespace}/local_position/velocity_body",
            self._velocity_body_callback,
            qos_profile_sensor_data
        )

        self.altitude_sub = self.create_subscription(
            Altitude,
            f"/{namespace}/altitude",
            self._altitude_callback,
            qos_profile_sensor_data
        )

        self.gps_raw_1_sub = self.create_subscription(
            GPSRAW,
            f"/{namespace}/gpsstatus/gps1/raw",
            self._gps_raw_1_callback,
            qos_profile_sensor_data
        )

        self.gps_raw_2_sub = self.create_subscription(
            GPSRAW,
            f"/{namespace}/gpsstatus/gps2/raw",
            self._gps_raw_2_callback,
            qos_profile_sensor_data
        )

        self.gps_velocity_sub = self.create_subscription(
            TwistStamped,
            f"/{namespace}/global_position/raw/gps_vel",
            self._gps_velocity_callback,
            qos_profile_sensor_data
        )

        self.rtk_1_sub = self.create_subscription(
            GPSRTK,
            f"/{namespace}/gpsstatus/gps1/rtk",
            self._rtk_1_callback,
            qos_profile_sensor_data
        )

        self.rtk_2_sub = self.create_subscription(
            GPSRTK,
            f"/{namespace}/gpsstatus/gps2/rtk",
            self._rtk_2_callback,
            qos_profile_sensor_data
        )

        self.rtk_baseline_sub = self.create_subscription(
            RTKBaseline,
            f"/{namespace}/gps_rtk/rtk_baseline",
            self._rtk_baseline_callback,
            qos_profile_sensor_data
        )



        # =========================================================
        # Service clients and publishers
        # These are created here because the unified interface object
        # will also use setter/control mixins.
        # clients are two way communication, messages are one way.
        # =========================================================

        self.param_set_client = self.create_client(ParamSet, f"/{namespace}/param/set")
        self.arming_client = self.create_client(CommandBool, f"/{namespace}/cmd/arming")
        self.set_mode_client = self.create_client(SetMode, f"/{namespace}/set_mode")
        self.takeoff_client = self.create_client(CommandTOL, f"/{namespace}/cmd/takeoff")
        self.land_client = self.create_client(CommandTOL, f"/{namespace}/cmd/land")
        self.home_client = self.create_client(CommandHome, f"/{namespace}/cmd/set_home")

        # Create publishers
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            f"/{namespace}/setpoint_position/local",
            10
        )

        # publisher for sending local velocity setpoints
        self.velocity_setpoint_pub = self.create_publisher(
            TwistStamped,
            f"/{namespace}/setpoint_velocity/cmd_vel",
            10
        )

        print(f"[PX4] Initialized MAVROS interface (namespace: {namespace})")

    # =========================================================
    # Callbacks
    # These are invoked whenever new messages arrive.
    # =========================================================

    def _state_callback(self, msg):
        """Update current vehicle state"""
        self.current_state = msg
        self.connected = msg.connected
        self._last_state_time = time.time()

    def _position_callback(self, msg):
        """Update current vehicle position"""
        self.current_position = msg
        self._last_position_time = time.time()
        if not self._seen_position:
            pos = msg.pose.position
            print(f"[PX4] Receiving local pose: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}")
            self._seen_position = True

    def _odometry_callback(self, msg):
        """Update current odometry"""
        self.current_odometry = msg
        self._last_odometry_time = time.time()
        if not self._seen_odometry:
            pos = msg.pose.pose.position
            print(f"[PX4] Receiving odometry: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}")
            self._seen_odometry = True

    def _battery_callback(self, msg):
        """Update battery status"""
        self.battery_status = msg
        self._last_battery_time = time.time()
        if not self._seen_battery:
            print(f"[PX4] Receiving battery status: {msg.percentage:.2f}")
            self._seen_battery = True

    def _global_position_callback(self, msg):
        """Update GPS position"""
        self.current_gps = msg
        self._last_gps_time = time.time()
        if not self._seen_gps:
            print(f"[PX4] Receiving GPS position: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}, alt={msg.altitude:.2f}")
            self._seen_gps = True

    def _global_position_raw_fix_callback(self, msg):
        """Update raw/fix GPS position"""
        self.current_gps_raw_fix = msg
        self._last_gps_raw_fix_time = time.time()
        if not self._seen_gps_raw_fix:
            print(f"[PX4] Receiving GPS raw/fix: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}, alt={msg.altitude:.2f}")
            self._seen_gps_raw_fix = True

    def _attitude_callback(self, msg):
        """Update attitude/orientation"""
        self.current_attitude = msg
        self._last_attitude_time = time.time()
        if not self._seen_attitude:
            print("[PX4] Receiving IMU data")
            self._seen_attitude = True

    def _extended_state_callback(self, msg):
        """Update extended state"""
        self.extended_state = msg
        self._last_extended_state_time = time.time()
        if not self._seen_extended_state:
            print(f"[PX4] Receiving extended state: landed_state={msg.landed_state}")
            self._seen_extended_state = True

    def _home_position_callback(self, msg):
        """Update home position"""
        self.home_position = msg
        self._last_home_position_time = time.time()
        if not self._seen_home_position:
            print(f"[PX4] Receiving home position: lat={msg.geo.latitude:.6f}, lon={msg.geo.longitude:.6f}, alt={msg.geo.altitude:.2f}")
            self._seen_home_position = True

    def _velocity_callback(self, msg):
        """Update velocity"""
        self.current_velocity = msg
        self._last_velocity_time = time.time()
        if not self._seen_velocity:
            vel = msg.twist.linear
            print(f"[PX4] Receiving local velocity: x={vel.x:.2f}, y={vel.y:.2f}, z={vel.z:.2f}")
            self._seen_velocity = True

    def _velocity_local_callback(self, msg):
        """Update local velocity"""
        self.current_velocity_local = msg
        self._last_velocity_local_time = time.time()
        if not self._seen_velocity_local:
            vel = msg.twist.linear
            print(f"[PX4] Receiving velocity_local: x={vel.x:.2f}, y={vel.y:.2f}, z={vel.z:.2f}")
            self._seen_velocity_local = True

    def _velocity_body_callback(self, msg):
        """Update body velocity"""
        self.current_velocity_body = msg
        self._last_velocity_body_time = time.time()
        if not self._seen_velocity_body:
            vel = msg.twist.linear
            print(f"[PX4] Receiving velocity_body: x={vel.x:.2f}, y={vel.y:.2f}, z={vel.z:.2f}")
            self._seen_velocity_body = True

    def _altitude_callback(self, msg):
        """Update altitude"""
        self.current_altitude = msg
        self._last_altitude_time = time.time()
        if not self._seen_altitude:
            print(f"[PX4] Receiving altitude: relative={msg.relative:.2f}, amsl={msg.amsl:.2f}")
            self._seen_altitude = True

    def _gps_raw_1_callback(self, msg):
        """Update GPS1 raw data"""
        self.current_gps_raw_1 = msg
        self._last_gps_raw_1_time = time.time()
        if not self._seen_gps_raw_1:
            print(f"[PX4] Receiving GPS1 raw: fix_type={msg.fix_type}, satellites={msg.satellites_visible}")
            self._seen_gps_raw_1 = True

    def _gps_raw_2_callback(self, msg):
        """Update GPS2 raw data"""
        self.current_gps_raw_2 = msg
        self._last_gps_raw_2_time = time.time()
        if not self._seen_gps_raw_2:
            print(f"[PX4] Receiving GPS2 raw: fix_type={msg.fix_type}, satellites={msg.satellites_visible}")
            self._seen_gps_raw_2 = True

    def _gps_velocity_callback(self, msg):
        """Update GPS-derived velocity"""
        self.current_gps_velocity = msg
        self._last_gps_velocity_time = time.time()
        if not self._seen_gps_velocity:
            vel = msg.twist.linear
            print(f"[PX4] Receiving GPS velocity: x={vel.x:.2f}, y={vel.y:.2f}, z={vel.z:.2f}")
            self._seen_gps_velocity = True

    def _rtk_1_callback(self, msg):
        """Update GPS1 RTK data"""
        self.current_rtk_1 = msg
        self._last_rtk_1_time = time.time()
        if not self._seen_rtk_1:
            print("[PX4] Receiving GPS1 RTK data")
            self._seen_rtk_1 = True

    def _rtk_2_callback(self, msg):
        """Update GPS2 RTK data"""
        self.current_rtk_2 = msg
        self._last_rtk_2_time = time.time()
        if not self._seen_rtk_2:
            print("[PX4] Receiving GPS2 RTK data")
            self._seen_rtk_2 = True

    def _rtk_baseline_callback(self, msg):
        """Update RTK baseline data"""
        self.current_rtk_baseline = msg
        self._last_rtk_baseline_time = time.time()
        if not self._seen_rtk_baseline:
            print("[PX4] Receiving RTK baseline data")
            self._seen_rtk_baseline = True

    # =========================================================
    # Helper functions used by getter APIs
    # =========================================================

    def _safe_get(self, obj, *attrs):
        """Safely get nested attributes"""
        current = obj
        for attr in attrs:
            if current is None or not hasattr(current, attr):
                return None
            current = getattr(current, attr)
        return current

    def _quat_to_euler(self, x, y, z, w):
        """Convert quaternion to roll, pitch, yaw in radians"""
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def _vector3_to_dict(self, vec):
        """Convert x/y/z object to dict"""
        if vec is None:
            return None
        return {
            "x": getattr(vec, "x", None),
            "y": getattr(vec, "y", None),
            "z": getattr(vec, "z", None)
        }

    def _quaternion_to_dict(self, quat):
        """Convert quaternion object to dict"""
        if quat is None:
            return None
        return {
            "x": getattr(quat, "x", None),
            "y": getattr(quat, "y", None),
            "z": getattr(quat, "z", None),
            "w": getattr(quat, "w", None)
        }

    def _navsatfix_to_dict(self, gps):
        """Convert NavSatFix message to dict"""
        if gps is None:
            return None
        return {
            "latitude": getattr(gps, "latitude", None),
            "longitude": getattr(gps, "longitude", None),
            "altitude": getattr(gps, "altitude", None),
            "position_covariance": getattr(gps, "position_covariance", None),
            "position_covariance_type": getattr(gps, "position_covariance_type", None),
        }

    def _warn_missing_telemetry(self, name):
        """Print a useful debug message when telemetry is missing"""
        print(f"[PX4][WARN] {name} telemetry not available yet. Check MAVROS topic, message type, and whether the FCU is publishing it.")

    # =========================================================
    # Connection helpers
    # =========================================================

    def _wait_for_connection(self, timeout=None):
        """Wait until MAVROS reports the FCU is actually connected (heartbeat received)."""
        if timeout is None:
            timeout = self.timeout

        start = time.time()
        while (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_state is not None and self.current_state.connected:
                print("[PX4] Connected to MAVROS (FCU heartbeat received)")
                self.connected = True
                return True
            time.sleep(0.1)

        print("[PX4] Failed to get FCU heartbeat within timeout")
        self.connected = False
        return False

    def connect(self):
        """
        Connect to MAVROS (wait for connection)
        MAVROS must be running for this to work
        """
        print("[PX4] Connecting to MAVROS...")
        return self._wait_for_connection()

    def disconnect(self):
        """Disconnect from MAVROS"""
        print("[PX4] Disconnected")
        try:
            self.destroy_node()
            print("[PX4] Node destroyed")
        except Exception as e:
            print(f"[PX4] Disconnect failed: {str(e)}")

    # =========================================================
    # Basic getters
    # =========================================================

    def is_armed(self):
        """Check if vehicle is armed"""
        if not self.connected or not self.current_state:
            return False
        return self.current_state.armed

    def get_location(self):
        """Get current vehicle location"""
        if not self.connected or not self.current_position:
            return None

        try:
            pos = self.current_position.pose.position
            return {
                "x": pos.x,
                "y": pos.y,
                "z": pos.z,
            }
        except Exception as e:
            print(f"[PX4] Failed to get location: {str(e)}")
        return None

    def get_altitude(self):
        """Get current altitude from local pose z"""
        loc = self.get_location()
        return loc["z"] if loc else 0

    def get_battery_status(self):
        """Get battery status"""
        if not self.connected or not self.battery_status:
            return None

        try:
            return {
                "voltage": self.battery_status.voltage,
                "current": self.battery_status.current,
                "percentage": self.battery_status.percentage
            }
        except Exception as e:
            print(f"[PX4] Failed to get battery: {str(e)}")
        return None

    def get_gps_location(self):
        """Get GPS latitude, longitude, altitude"""
        if not self.current_gps:
            return None
        return {
            "latitude": self.current_gps.latitude,
            "longitude": self.current_gps.longitude,
            "altitude": self.current_gps.altitude
        }

    def get_velocity(self):
        """Get current velocity (x, y, z in m/s)"""
        if not self.current_velocity:
            return None
        return {
            "x": self.current_velocity.twist.linear.x,
            "y": self.current_velocity.twist.linear.y,
            "z": self.current_velocity.twist.linear.z
        }

    def is_landed(self):
        """Check if vehicle is landed"""
        if not self.extended_state:
            return False
        return self.extended_state.landed_state == 1  # 1 = landed

    def get_home_location(self):
        """Get home position"""
        if not self.home_position:
            return None
        return {
            "latitude": self.home_position.geo.latitude,
            "longitude": self.home_position.geo.longitude,
            "altitude": self.home_position.geo.altitude
        }

    # =========================================================
    # Integrated telemetry getter APIs
    # =========================================================

    def get_odometry(self):
        """Get full odometry information (pose + twist)"""
        if not self.current_odometry:
            self._warn_missing_telemetry("odometry")
            return None

        try:
            pose = self.current_odometry.pose.pose
            twist = self.current_odometry.twist.twist

            orientation_q = self._quaternion_to_dict(pose.orientation)
            orientation_euler = None
            if orientation_q and None not in orientation_q.values():
                roll, pitch, yaw = self._quat_to_euler(
                    orientation_q["x"],
                    orientation_q["y"],
                    orientation_q["z"],
                    orientation_q["w"]
                )
                orientation_euler = {
                    "roll": roll,
                    "pitch": pitch,
                    "yaw": yaw
                }

            return {
                "position": self._vector3_to_dict(pose.position),
                "orientation_quaternion": orientation_q,
                "orientation_euler_rad": orientation_euler,
                "linear_velocity": self._vector3_to_dict(twist.linear),
                "angular_velocity": self._vector3_to_dict(twist.angular)
            }
        except Exception as e:
            print(f"[PX4][ERROR] Failed to get odometry: {str(e)}")
            return None

    def get_pose(self):
        """Get local pose (position + orientation)"""
        if not self.current_position:
            self._warn_missing_telemetry("pose")
            return None

        try:
            pose = self.current_position.pose
            orientation_q = self._quaternion_to_dict(pose.orientation)

            orientation_euler = None
            if orientation_q and None not in orientation_q.values():
                roll, pitch, yaw = self._quat_to_euler(
                    orientation_q["x"],
                    orientation_q["y"],
                    orientation_q["z"],
                    orientation_q["w"]
                )
                orientation_euler = {
                    "roll": roll,
                    "pitch": pitch,
                    "yaw": yaw
                }

            return {
                "position": self._vector3_to_dict(pose.position),
                "orientation_quaternion": orientation_q,
                "orientation_euler_rad": orientation_euler
            }
        except Exception as e:
            print(f"[PX4][ERROR] Failed to get pose: {str(e)}")
            return None

    def get_velocity_body(self):
        """Get body-frame velocity (linear + angular)"""
        if not self.current_velocity_body:
            self._warn_missing_telemetry("velocity_body")
            return None

        try:
            return {
                "linear": self._vector3_to_dict(self.current_velocity_body.twist.linear),
                "angular": self._vector3_to_dict(self.current_velocity_body.twist.angular)
            }
        except Exception as e:
            print(f"[PX4][ERROR] Failed to get body velocity: {str(e)}")
            return None

    def get_velocity_local(self):
        """Get local-frame velocity (linear + angular)"""
        if not self.current_velocity_local:
            self._warn_missing_telemetry("velocity_local")
            return None

        try:
            return {
                "linear": self._vector3_to_dict(self.current_velocity_local.twist.linear),
                "angular": self._vector3_to_dict(self.current_velocity_local.twist.angular)
            }
        except Exception as e:
            print(f"[PX4][ERROR] Failed to get local velocity: {str(e)}")
            return None

    def get_acceleration(self):
        """Get IMU-based acceleration and angular velocity"""
        if not self.current_attitude:
            self._warn_missing_telemetry("imu/acceleration")
            return None

        try:
            orientation_q = self._quaternion_to_dict(self.current_attitude.orientation)

            orientation_euler = None
            if orientation_q and None not in orientation_q.values():
                roll, pitch, yaw = self._quat_to_euler(
                    orientation_q["x"],
                    orientation_q["y"],
                    orientation_q["z"],
                    orientation_q["w"]
                )
                orientation_euler = {
                    "roll": roll,
                    "pitch": pitch,
                    "yaw": yaw
                }

            return {
                "linear_acceleration": self._vector3_to_dict(self.current_attitude.linear_acceleration),
                "angular_velocity": self._vector3_to_dict(self.current_attitude.angular_velocity),
                "orientation_quaternion": orientation_q,
                "orientation_euler_rad": orientation_euler
            }
        except Exception as e:
            print(f"[PX4][ERROR] Failed to get acceleration: {str(e)}")
            return None

    def get_altitude_data(self):
        """Get MAVROS altitude topic data"""
        if not self.current_altitude:
            self._warn_missing_telemetry("altitude")
            return None

        try:
            return {
                "amsl": self.current_altitude.amsl,
                "local": self.current_altitude.local,
                "relative": self.current_altitude.relative,
                "terrain": self.current_altitude.terrain,
                "bottom_clearance": self.current_altitude.bottom_clearance,
                "monotonic": self.current_altitude.monotonic
            }
        except Exception as e:
            print(f"[PX4][ERROR] Failed to get altitude data: {str(e)}")
            return None

    def get_gps_raw_fix(self):
        """Get /global_position/raw/fix GPS data"""
        if not self.current_gps_raw_fix:
            self._warn_missing_telemetry("global_position/raw/fix")
            return None

        try:
            return self._navsatfix_to_dict(self.current_gps_raw_fix)
        except Exception as e:
            print(f"[PX4][ERROR] Failed to get GPS raw/fix data: {str(e)}")
            return None

    def get_gps_raw(self, gps_id=1):
        """Get raw GPS data from /gpsstatus/gpsX/raw"""
        raw = self.current_gps_raw_1 if gps_id == 1 else self.current_gps_raw_2
        if not raw:
            self._warn_missing_telemetry(f"gpsstatus/gps{gps_id}/raw")
            return None

        try:
            return {
                "fix_type": getattr(raw, "fix_type", None),
                "lat": getattr(raw, "lat", None),
                "lon": getattr(raw, "lon", None),
                "alt": getattr(raw, "alt", None),
                "alt_ellipsoid": getattr(raw, "alt_ellipsoid", None),
                "eph": getattr(raw, "eph", None),
                "epv": getattr(raw, "epv", None),
                "vel": getattr(raw, "vel", None),
                "cog": getattr(raw, "cog", None),
                "satellites_visible": getattr(raw, "satellites_visible", None),
                "h_acc": getattr(raw, "h_acc", None),
                "v_acc": getattr(raw, "v_acc", None),
                "vel_acc": getattr(raw, "vel_acc", None),
                "hdg_acc": getattr(raw, "hdg_acc", None),
                "yaw": getattr(raw, "yaw", None),
                "dgps_age": getattr(raw, "dgps_age", None),
                "dgps_numch": getattr(raw, "dgps_numch", None),
            }
        except Exception as e:
            print(f"[PX4][ERROR] Failed to get GPS{gps_id} raw data: {str(e)}")
            return None

    def get_gps_velocity(self):
        """Get GPS-derived velocity from /global_position/raw/gps_vel"""
        if not self.current_gps_velocity:
            self._warn_missing_telemetry("global_position/raw/gps_vel")
            return None

        try:
            return {
                "linear": self._vector3_to_dict(self.current_gps_velocity.twist.linear),
                "angular": self._vector3_to_dict(self.current_gps_velocity.twist.angular)
            }
        except Exception as e:
            print(f"[PX4][ERROR] Failed to get GPS velocity: {str(e)}")
            return None

    def get_rtk_data(self, gps_id=1):
        """Get RTK telemetry data from /gpsstatus/gpsX/rtk"""
        rtk = self.current_rtk_1 if gps_id == 1 else self.current_rtk_2
        if not rtk:
            self._warn_missing_telemetry(f"gpsstatus/gps{gps_id}/rtk")
            return None

        try:
            data = {}
            for field in [
                "rtk_receiver_id",
                "wn",
                "tow",
                "rtk_health",
                "rtk_rate",
                "nsats",
                "baseline_a",
                "baseline_b",
                "baseline_c",
                "accuracy",
                "iar_num_hypotheses"
            ]:
                if hasattr(rtk, field):
                    data[field] = getattr(rtk, field)
            return data
        except Exception as e:
            print(f"[PX4][ERROR] Failed to get GPS{gps_id} RTK data: {str(e)}")
            return None

    def get_rtk_baseline(self):
        """Get RTK baseline data"""
        if not self.current_rtk_baseline:
            self._warn_missing_telemetry("gps_rtk/rtk_baseline")
            return None

        try:
            data = {}
            for field in [
                "header",
                "time_last_baseline_ms",
                "rtk_receiver_id",
                "wn",
                "tow",
                "rtk_health",
                "rtk_rate",
                "nsats",
                "baseline_a_mm",
                "baseline_b_mm",
                "baseline_c_mm",
                "accuracy",
                "iar_num_hypotheses"
            ]:
                if hasattr(self.current_rtk_baseline, field):
                    data[field] = getattr(self.current_rtk_baseline, field)
            return data
        except Exception as e:
            print(f"[PX4][ERROR] Failed to get RTK baseline data: {str(e)}")
            return None

    def get_full_telemetry_snapshot(self):
        """Get a structured snapshot of all available telemetry"""
        return {
            "status": {
                "connected": self.connected,
                "armed": self.is_armed(),
                "mode": self.current_state.mode if self.current_state else None,
                "landed": self.is_landed()
            },
            "battery": self.get_battery_status(),
            "home": self.get_home_location(),
            "gps_location": self.get_gps_location(),
            "gps_raw_fix": self.get_gps_raw_fix(),
            "odometry": self.get_odometry(),
            "pose": self.get_pose(),
            "velocity_body": self.get_velocity_body(),
            "velocity_local": self.get_velocity_local(),
            "acceleration": self.get_acceleration(),
            "altitude": self.get_altitude_data(),
            "gps_raw_1": self.get_gps_raw(1),
            "gps_raw_2": self.get_gps_raw(2),
            "gps_velocity": self.get_gps_velocity(),
            "rtk_1": self.get_rtk_data(1),
            "rtk_2": self.get_rtk_data(2),
            "rtk_baseline": self.get_rtk_baseline(),
        }

    def print_telemetry_summary(self):
        """Pretty-print a compact telemetry summary for debugging"""
        snapshot = self.get_full_telemetry_snapshot()

        print("=" * 60)
        print("[TELEMETRY] Current Status")
        print(f"Connected: {snapshot['status']['connected']}")
        print(f"Armed:     {snapshot['status']['armed']}")
        print(f"Mode:      {snapshot['status']['mode']}")
        print(f"Landed:    {snapshot['status']['landed']}")

        pose = snapshot["pose"]
        if pose and pose["position"]:
            pos = pose["position"]
            print(f"Position (Local): X={pos['x']}, Y={pos['y']}, Z={pos['z']}")

        if pose and pose["orientation_euler_rad"]:
            ori = pose["orientation_euler_rad"]
            print(f"Orientation (RPY rad): R={ori['roll']}, P={ori['pitch']}, Y={ori['yaw']}")

        vel_local = snapshot["velocity_local"]
        if vel_local and vel_local["linear"]:
            lv = vel_local["linear"]
            print(f"Velocity (Local): X={lv['x']}, Y={lv['y']}, Z={lv['z']}")

        vel_body = snapshot["velocity_body"]
        if vel_body and vel_body["linear"]:
            bv = vel_body["linear"]
            print(f"Velocity (Body):  X={bv['x']}, Y={bv['y']}, Z={bv['z']}")

        acc = snapshot["acceleration"]
        if acc and acc["linear_acceleration"]:
            la = acc["linear_acceleration"]
            print(f"Acceleration:     X={la['x']}, Y={la['y']}, Z={la['z']}")

        alt = snapshot["altitude"]
        if alt:
            print(f"Altitude Relative: {alt['relative']}")
            print(f"Altitude AMSL:     {alt['amsl']}")

        gps = snapshot["gps_location"]
        if gps:
            print(f"GPS Position: lat={gps['latitude']}, lon={gps['longitude']}, alt={gps['altitude']}")

        gps_raw = snapshot["gps_raw_1"]
        if gps_raw:
            print(f"GPS1 Raw: fix_type={gps_raw['fix_type']}, satellites={gps_raw['satellites_visible']}, eph={gps_raw['eph']}, epv={gps_raw['epv']}")

        print("=" * 60)

    def print_telemetry_health(self, stale_after_sec=2.0):
        """
        Print which subscriptions are receiving data and which are missing/stale.
        This is useful because wrong topic names often fail silently.
        """
        now = time.time()

        def status_str(last_time):
            if last_time is None:
                return "NO DATA"
            age = now - last_time
            if age > stale_after_sec:
                return f"STALE ({age:.1f}s old)"
            return f"OK ({age:.1f}s old)"

        print("=" * 60)
        print("[PX4] Telemetry Health Check")
        print(f"state:                {status_str(self._last_state_time)}")
        print(f"local_position/pose:  {status_str(self._last_position_time)}")
        print(f"local_position/odom:  {status_str(self._last_odometry_time)}")
        print(f"battery:              {status_str(self._last_battery_time)}")
        print(f"global_position/global:   {status_str(self._last_gps_time)}")
        print(f"global_position/raw/fix:  {status_str(self._last_gps_raw_fix_time)}")
        print(f"imu/data:             {status_str(self._last_attitude_time)}")
        print(f"extended_state:       {status_str(self._last_extended_state_time)}")
        print(f"home_position/home:   {status_str(self._last_home_position_time)}")
        print(f"velocity_local:       {status_str(self._last_velocity_local_time)}")
        print(f"velocity_body:        {status_str(self._last_velocity_body_time)}")
        print(f"altitude:             {status_str(self._last_altitude_time)}")
        print(f"gpsstatus/gps1/raw:   {status_str(self._last_gps_raw_1_time)}")
        print(f"gpsstatus/gps2/raw:   {status_str(self._last_gps_raw_2_time)}")
        print(f"global_position/raw/gps_vel: {status_str(self._last_gps_velocity_time)}")
        print(f"gpsstatus/gps1/rtk:   {status_str(self._last_rtk_1_time)}")
        print(f"gpsstatus/gps2/rtk:   {status_str(self._last_rtk_2_time)}")
        print(f"gps_rtk/rtk_baseline: {status_str(self._last_rtk_baseline_time)}")
        print("=" * 60)


# =========================================================
# Utility Functions
# =========================================================

def get_jetson_ip():
    """
    Retrieve the IP address of the Jetson by running 'hostname -I'.
    
    This is a module-level function that can be called independently
    of the PX4Getters class.
    
    Returns:
        str: The IP address of the Jetson (e.g., "192.168.1.100")
             Returns "IP_NOT_FOUND" if unable to retrieve
    """
    try:
        result = subprocess.run(['hostname', '-I'], 
                              capture_output=True, 
                              text=True, 
                              timeout=5)
        if result.returncode == 0:
            # hostname -I returns multiple IPs separated by spaces
            # Get the first one (usually the primary interface)
            ip_addresses = result.stdout.strip().split()
            if ip_addresses:
                return ip_addresses[0]
        return "IP_NOT_FOUND"
    except Exception as e:
        print(f"[ERROR] Failed to get Jetson IP: {str(e)}")
        return "IP_NOT_FOUND"
