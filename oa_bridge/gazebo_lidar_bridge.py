#!/usr/bin/env python3
"""
Custom Gazebo -> ROS 2 LaserScan bridge.
Uses `gz topic` to echo and parse Gazebo LaserScan messages.
Workaround for ros_gz_bridge protobuf version mismatch.
"""

import sys
import argparse
import subprocess
import json
import re
from threading import Thread

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class GazeboLidarBridge(Node):
    def __init__(self, gz_topic, ros_topic="/scan", num_messages=0):
        super().__init__('gazebo_lidar_bridge')
        self.pub = self.create_publisher(LaserScan, ros_topic, 10)
        self.gz_topic = gz_topic
        self.ros_topic = ros_topic
        
        print(f"[Bridge] Subscribing to Gazebo topic: {gz_topic}")
        print(f"[Bridge] Publishing to ROS topic: {ros_topic}")
        
        # Start subprocess to echo Gazebo topic
        self.process = subprocess.Popen(
            ['gz', 'topic', '-e', '--topic', gz_topic, '--num-messages', str(num_messages)],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
            bufsize=1
        )
        
        # Start reader thread
        self.reader_thread = Thread(target=self._read_gazebo_stream, daemon=True)
        self.reader_thread.start()

    def _read_gazebo_stream(self):
        """Read Gazebo topic output and parse LaserScan messages.

        Each gz LaserScan is delimited by a top-level `header:` line. A scan
        has hundreds of `ranges:` lines but the angle_*/range_* metadata
        appears only once, before them — so a message must be parsed whole.
        Splitting it into chunks drops that metadata and produces scans with
        angle_increment=0 / range_max=0, which the OA core then reads as
        "no obstacles". Publish strictly on the message boundary.
        """
        buffer = []

        for line in self.process.stdout:
            line = line.strip()

            # Detect message start (header: at the top level, no indentation)
            if line == 'header:' and buffer:
                # Previous message complete, publish it whole.
                if self._has_ranges(buffer):
                    self._parse_and_publish(buffer)
                buffer = [line]
            else:
                buffer.append(line)
    
    def _has_ranges(self, buffer):
        """Check if buffer has actual range data."""
        return any('ranges:' in line for line in buffer)

    def _parse_and_publish(self, lines):
        """Parse gz topic output and publish as ROS LaserScan."""
        ros_msg = LaserScan()
        ros_msg.header.stamp = self.get_clock().now().to_msg()
        ros_msg.header.frame_id = "lidar_link"
        
        ranges = []
        intensities = []
        
        for line in lines:
            # Extract key-value pairs from "key: value" format
            if ': ' not in line:
                continue
            
            parts = line.split(': ', 1)
            key = parts[0].strip()
            value = parts[1].strip() if len(parts) > 1 else ""
            
            try:
                if key == 'angle_min':
                    ros_msg.angle_min = float(value)
                elif key == 'angle_max':
                    ros_msg.angle_max = float(value)
                elif key == 'angle_increment':
                    ros_msg.angle_increment = float(value)
                elif key == 'time_increment':
                    ros_msg.time_increment = float(value)
                elif key == 'scan_time':
                    ros_msg.scan_time = float(value)
                elif key == 'range_min':
                    ros_msg.range_min = float(value)
                elif key == 'range_max':
                    ros_msg.range_max = float(value)
                elif key == 'ranges':
                    try:
                        ranges.append(float(value))
                    except ValueError:
                        ranges.append(float('inf'))
                elif key == 'intensities':
                    try:
                        intensities.append(float(value))
                    except ValueError:
                        intensities.append(0.0)
            except (ValueError, IndexError):
                pass
        
        if ranges:
            ros_msg.ranges = ranges
            ros_msg.intensities = intensities if intensities else [0.0] * len(ranges)
            self.pub.publish(ros_msg)
            self.get_logger().debug(f"Published LaserScan with {len(ranges)} ranges")


def main():
    parser = argparse.ArgumentParser(description="Gazebo LaserScan -> ROS 2 bridge")
    parser.add_argument('--gz-topic', required=True, help='Gazebo topic (e.g., /world/default/model/.../scan)')
    parser.add_argument('--ros-topic', default='/scan', help='ROS topic to publish on')
    args = parser.parse_args()
    
    rclpy.init()
    bridge = GazeboLidarBridge(args.gz_topic, args.ros_topic, num_messages=0)
    
    print(f"[Bridge] Running... (Ctrl+C to stop)")
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        print("\n[Bridge] Shutting down...")
    finally:
        bridge.process.terminate()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
