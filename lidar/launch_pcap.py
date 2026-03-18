"""Launch Velodyne PCAP playback and the Foxglove bridge."""

import os
import yaml

import ament_index_python.packages
import launch
import launch.actions
import launch_ros.actions


def generate_launch_description():
    pcap_params = os.path.join(os.path.dirname(__file__), 'vlp16_pcap_params.yaml')
    driver_share = ament_index_python.packages.get_package_share_directory('velodyne_driver')
    default_params = os.path.join(driver_share, 'config', 'VLP16-velodyne_driver_node-params.yaml')
    driver_params = pcap_params if os.path.exists(pcap_params) else default_params

    convert_share = ament_index_python.packages.get_package_share_directory('velodyne_pointcloud')
    convert_params_file = os.path.join(convert_share, 'config', 'VLP16-velodyne_transform_node-params.yaml')
    with open(convert_params_file, 'r') as f:
        convert_params = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
    convert_params['calibration'] = os.path.join(convert_share, 'params', 'VLP16db.yaml')

    laserscan_share = ament_index_python.packages.get_package_share_directory('velodyne_laserscan')
    laserscan_params_file = os.path.join(laserscan_share, 'config', 'default-velodyne_laserscan_node-params.yaml')

    driver = launch_ros.actions.Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        output='both',
        parameters=[driver_params],
    )

    return launch.LaunchDescription([
        driver,
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
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=driver,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
