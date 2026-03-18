"""Launch live Velodyne ingest and the Foxglove bridge."""

import os
import yaml

import ament_index_python.packages
import launch
import launch.actions
import launch_ros.actions


def generate_launch_description():
    # Prefer the repository-local live driver parameters when present.
    custom_params = os.path.join(
        os.path.dirname(__file__), 'vlp16_live_params.yaml'
    )
    driver_share = ament_index_python.packages.get_package_share_directory('velodyne_driver')
    default_params = os.path.join(driver_share, 'config', 'VLP16-velodyne_driver_node-params.yaml')
    driver_params = custom_params if os.path.exists(custom_params) else default_params

    # Start from the package defaults, then pin the calibration file explicitly.
    convert_share = ament_index_python.packages.get_package_share_directory('velodyne_pointcloud')
    convert_params_file = os.path.join(convert_share, 'config', 'VLP16-velodyne_transform_node-params.yaml')
    with open(convert_params_file, 'r') as f:
        convert_params = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
    convert_params['calibration'] = os.path.join(convert_share, 'params', 'VLP16db.yaml')

    # Use the packaged LaserScan defaults.
    laserscan_share = ament_index_python.packages.get_package_share_directory('velodyne_laserscan')
    laserscan_params_file = os.path.join(laserscan_share, 'config', 'default-velodyne_laserscan_node-params.yaml')

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='velodyne_driver',
            executable='velodyne_driver_node',
            output='both',
            parameters=[driver_params],
        ),
        launch_ros.actions.Node(
            package='velodyne_pointcloud',
            executable='velodyne_transform_node',
            output='both',
            parameters=[convert_params],
        ),
        launch_ros.actions.Node(
            package='velodyne_laserscan',
            executable='velodyne_laserscan_node',
            output='both',
            parameters=[laserscan_params_file],
        ),
        launch_ros.actions.Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            output='both',
            parameters=[{'port': 8765}],
        ),
    ])
