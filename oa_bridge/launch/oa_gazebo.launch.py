"""
oa_gazebo.launch.py — bring up the ROS 2 side of the Gazebo OA stack.

Starts:
  1. MAVROS                    — PX4 SITL  <->  ROS 2 bridge
  2. gazebo_lidar_bridge.py    — Gazebo 2-D lidar (gz.msgs.LaserScan) -> sensor_msgs/LaserScan
  3. oa_ros2_node             — the obstacle-avoidance controller

It does NOT start PX4 SITL + Gazebo itself — that is launched from the
PX4-Autopilot tree (`make px4_sitl gz_x500_lidar_2d`). See README_GAZEBO.md.

Note: Uses custom gazebo_lidar_bridge.py (not ros_gz_bridge) to work around
protobuf version mismatch issues.

Example:
  ros2 launch oa_bridge/launch/oa_gazebo.launch.py \\
      target_x:=20.0 target_y:=0.0 target_z:=3.0 \\
      gz_lidar_topic:=/world/default/model/x500_lidar_2d_0/link/lidar_sensor_link/sensor/lidar_2d_v2/scan
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # ── launch arguments ─────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument("target_x", default_value="20.0",
                              description="Target East in MAVROS ENU map frame (m)"),
        DeclareLaunchArgument("target_y", default_value="0.0",
                              description="Target North in MAVROS ENU map frame (m)"),
        DeclareLaunchArgument("target_z", default_value="3.0",
                              description="Target altitude / Up (m)"),
        DeclareLaunchArgument("takeoff_alt", default_value="3.0",
                              description="Takeoff altitude (m)"),
        DeclareLaunchArgument("fcu_url", default_value="udp://:14540@127.0.0.1:14557",
                              description="MAVROS FCU URL for PX4 SITL"),
        DeclareLaunchArgument("ros_lidar_topic", default_value="/scan",
                              description="ROS 2 LaserScan topic the OA node subscribes to"),
        DeclareLaunchArgument(
            "gz_lidar_topic",
            # Default matches PX4 `gz_x500_lidar_2d` in the `default` world.
            # Run `gz topic -l | grep scan` if your model/world differs.
            default_value="/world/default/model/x500_lidar_2d_0/link/"
                           "lidar_sensor_link/sensor/lidar_2d_v2/scan",
            description="Gazebo (gz-transport) lidar scan topic to bridge"),
    ]

    target_x = LaunchConfiguration("target_x")
    target_y = LaunchConfiguration("target_y")
    target_z = LaunchConfiguration("target_z")
    takeoff_alt = LaunchConfiguration("takeoff_alt")
    fcu_url = LaunchConfiguration("fcu_url")
    ros_lidar_topic = LaunchConfiguration("ros_lidar_topic")
    gz_lidar_topic = LaunchConfiguration("gz_lidar_topic")

    # ── 1. MAVROS ────────────────────────────────────────────────────────
    mavros = Node(
        package="mavros",
        executable="mavros_node",
        name="mavros",
        namespace="mavros",
        output="screen",
        parameters=[{
            "fcu_url": fcu_url,
            "target_system_id": 1,
            "target_component_id": 1,
        }],
    )

    # ── 2. Gazebo lidar -> ROS 2 LaserScan bridge ────────────────────────
    # Custom Python bridge that uses `gz topic -e` to workaround ros_gz_bridge
    # protobuf version mismatch. See gazebo_lidar_bridge.py for details.
    gz_bridge = ExecuteProcess(
        cmd=[
            "python3",
            PathJoinSubstitution([
                os.path.dirname(os.path.abspath(__file__)), "..", "gazebo_lidar_bridge.py"
            ]),
            "--gz-topic", gz_lidar_topic,
            "--ros-topic", ros_lidar_topic,
        ],
        output="screen",
    )

    # ── 3. OA controller node ────────────────────────────────────────────
    oa_node = ExecuteProcess(
        cmd=[
            "python3",
            PathJoinSubstitution([
                os.path.dirname(os.path.abspath(__file__)), "..", "oa_ros2_node.py"
            ]),
            "--target-x", target_x,
            "--target-y", target_y,
            "--target-z", target_z,
            "--takeoff-alt", takeoff_alt,
            "--fcu-url", fcu_url,
            "--lidar-topic", ros_lidar_topic,
        ],
        output="screen",
    )

    return LaunchDescription(args + [mavros, gz_bridge, oa_node])
