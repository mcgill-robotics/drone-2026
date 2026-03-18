"""Launch the full LiDAR SLAM and occupancy-mapping pipeline.

Requires a flight controller connected via MAVROS for IMU/odometry.
Without MAVROS, RTAB-Map's ICP odometry can provide basic motion estimation
from LiDAR alone (set use_icp_odom:=true).
"""

import os

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import yaml


def generate_launch_description():
    config_dir = os.path.join(os.path.dirname(__file__), 'config')

    use_mavros = launch.substitutions.LaunchConfiguration('use_mavros', default='false')
    use_icp_odom = launch.substitutions.LaunchConfiguration('use_icp_odom', default='true')

    # Static transform from vehicle body frame to LiDAR frame.
    tf_config_path = os.path.join(config_dir, 'static_transforms.yaml')
    with open(tf_config_path, 'r') as f:
        tf_config = yaml.safe_load(f)
    vt = tf_config['velodyne_transform']

    static_tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', str(vt['x']),
            '--y', str(vt['y']),
            '--z', str(vt['z']),
            '--roll', str(vt['roll']),
            '--pitch', str(vt['pitch']),
            '--yaw', str(vt['yaw']),
            '--frame-id', vt['parent_frame'],
            '--child-frame-id', vt['child_frame'],
        ],
        output='both',
    )

    # Velodyne ingest and point cloud conversion.
    vlp16_params = os.path.join(os.path.dirname(__file__), 'vlp16_live_params.yaml')
    import ament_index_python.packages
    convert_share = ament_index_python.packages.get_package_share_directory('velodyne_pointcloud')
    convert_params_file = os.path.join(convert_share, 'config', 'VLP16-velodyne_transform_node-params.yaml')
    with open(convert_params_file, 'r') as f:
        convert_params = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
    convert_params['calibration'] = os.path.join(convert_share, 'params', 'VLP16db.yaml')

    laserscan_share = ament_index_python.packages.get_package_share_directory('velodyne_laserscan')
    laserscan_params_file = os.path.join(laserscan_share, 'config', 'default-velodyne_laserscan_node-params.yaml')

    velodyne_driver = launch_ros.actions.Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        output='both',
        parameters=[vlp16_params],
    )
    velodyne_transform = launch_ros.actions.Node(
        package='velodyne_pointcloud',
        executable='velodyne_transform_node',
        output='both',
        parameters=[convert_params],
    )
    velodyne_laserscan = launch_ros.actions.Node(
        package='velodyne_laserscan',
        executable='velodyne_laserscan_node',
        output='both',
        parameters=[laserscan_params_file],
    )

    # RTAB-Map ICP odometry for LiDAR-only motion estimation.
    rtabmap_config = os.path.join(config_dir, 'rtabmap.yaml')
    icp_odom = launch_ros.actions.Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        output='both',
        parameters=[rtabmap_config],
        remappings=[('scan_cloud', '/velodyne_points')],
        condition=launch.conditions.IfCondition(use_icp_odom),
    )

    # RTAB-Map SLAM back-end.
    rtabmap_slam = launch_ros.actions.Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='both',
        parameters=[rtabmap_config],
        remappings=[('scan_cloud', '/velodyne_points')],
        arguments=['--delete_db_on_start'],
    )

    # OctoMap occupancy mapper.
    octomap_config = os.path.join(config_dir, 'octomap.yaml')
    octomap = launch_ros.actions.Node(
        package='octomap_server',
        executable='octomap_server_node',
        output='both',
        parameters=[octomap_config],
        remappings=[('cloud_in', '/velodyne_points')],
    )

    # EKF for MAVROS-provided odometry and IMU data.
    ekf_config = os.path.join(config_dir, 'ekf.yaml')
    ekf = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='both',
        parameters=[ekf_config],
        condition=launch.conditions.IfCondition(use_mavros),
    )

    # MAVROS bridge to the flight controller.
    mavros_config = os.path.join(config_dir, 'mavros.yaml')
    mavros = launch_ros.actions.Node(
        package='mavros',
        executable='mavros_node',
        output='both',
        parameters=[mavros_config],
        condition=launch.conditions.IfCondition(use_mavros),
    )

    # Visualization bridge.
    foxglove = launch_ros.actions.Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        output='both',
        parameters=[{'port': 8765}],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('use_mavros', default_value='false',
            description='Enable MAVROS + EKF for flight controller integration'),
        launch.actions.DeclareLaunchArgument('use_icp_odom', default_value='true',
            description='Use ICP odometry from lidar (disable if using MAVROS odom)'),
        static_tf,
        velodyne_driver,
        velodyne_transform,
        velodyne_laserscan,
        icp_odom,
        rtabmap_slam,
        octomap,
        ekf,
        mavros,
        foxglove,
    ])
