import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import laspy
import numpy as np

class LasVisualizer(Node):
    def __init__(self):
        super().__init__('las_visualizer')
        self.publisher_ = self.create_publisher(PointCloud2, '/cloud_in', 10)
        self.points = np.array([], dtype=np.float32)
        
        # 1. Load the file ONCE during startup
        self.load_las_file()
        
        # 2. Start the timer to publish the ALREADY LOADED points
        self.timer = self.create_timer(5.0, self.publish_cloud)
        self.get_logger().info('LAS Visualizer started and ready.')

    def load_las_file(self):
        path = '/home/ubuntu/ros2_ws/src/my_project/ros2_ws/drone-2026/mission_controller/PUNKTSKY_1km_6181_724.las'
        try:
            las = laspy.read(path)
            # Center the data to prevent RViz camera jitter
            raw_points = np.vstack((las.x - np.mean(las.x), 
                                   las.y - np.mean(las.y), 
                                   las.z - np.mean(las.z))).transpose()
            # Downsample to keep VNC performance smooth
            self.points = raw_points[::20].astype(np.float32)
            self.get_logger().info(f'Loaded {len(self.points)} points from file.')
        except Exception as e:
            self.get_logger().error(f'Failed to load .las file: {e}')

    def publish_cloud(self):
        if self.points.size == 0:
            return

        try:
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'map'
            
            cloud_msg = pc2.create_cloud_xyz32(header, self.points)
            self.publisher_.publish(cloud_msg)
            self.get_logger().info(f'Broadcasting {len(self.points)} points.')
        except Exception as e:
            self.get_logger().error(f'Failed to publish: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = LasVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
