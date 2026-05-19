#!/usr/bin/env python3
"""Subscribe to `/scan` and print readable LiDAR ranges.

Usage:
  python3 oa_bridge/print_lidar.py --topic /scan

Press Ctrl+C to stop.
"""
import argparse
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class PrintLidar(Node):
    def __init__(self, topic: str):
        super().__init__('print_lidar')
        self.sub = self.create_subscription(LaserScan, topic, self.cb, 10)
        self.get_logger().info(f"Subscribed to {topic}")

    def cb(self, msg: LaserScan):
        ranges = msg.ranges
        # Count finite ranges
        finite = [r for r in ranges if math.isfinite(r)]
        inf_count = len(ranges) - len(finite)
        if finite:
            rmin = min(finite)
            rmax = max(finite)
            rmean = sum(finite) / len(finite)
        else:
            rmin = rmax = rmean = float('inf')
        # Print summary
        self.get_logger().info(
            f"ranges: total={len(ranges)} finite={len(finite)} inf={inf_count} "
            f"min={rmin:.3f} mean={rmean:.3f} max={rmax:.3f}"
        )
        # Print first 20 ranges (for quick inspection)
        preview = [f"{r:.3f}" if math.isfinite(r) else "inf" for r in ranges[:20]]
        self.get_logger().info(f"preview[0:20]: {preview}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/scan')
    args = parser.parse_args()

    rclpy.init()
    node = PrintLidar(args.topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
