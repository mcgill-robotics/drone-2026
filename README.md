# SUAS LiDAR Repository

#### DOCS are generated for now, going to re-write this weekend but code is pretty straightforward.


This repository contains ROS 2 Humble configuration and runtime files for:

- Velodyne VLP-16 packet ingest
- point cloud conversion
- LaserScan projection
- RTAB-Map configuration for LiDAR SLAM
- OctoMap configuration for occupancy mapping
- optional MAVROS and `robot_localization` integration

The repository is specific about the sensor and transport currently encoded in the files:

- sensor model: Velodyne VLP-16
- sensor data port: UDP `2368`
- default sensor IP reference: `192.168.1.201`
- typical host address on the same subnet: `192.168.1.100/24`
- Foxglove bridge port: TCP `8765`
- macOS relay port: TCP `12368`

The repository root contains the container runtime definition and the operator-facing entry script. The [`lidar/`](/Users/malachi/code/drone/lidar) directory contains launch files and ROS parameter files. Project-specific geometry, controller transport, and mapper tuning are isolated in YAML files under [`lidar/config/`](/Users/malachi/code/drone/lidar/config).

## Platform Notes

- Linux uses host networking and the existing container display hookup, so `rviz2` can run from inside the container.
- macOS uses the `lidar-mac` profile, which is set up for Foxglove. If you want `rviz2` on macOS, you need separate macOS-specific hooks for it.
- macOS live LiDAR ingest also requires the Docker tunnel path: run the in-container `udp_relay.py` listener and forward host UDP `2368` into Docker over TCP `12368`.
- `start.sh` handles that macOS tunnel automatically by launching the container relay and a host-side `socat` process.

## Files

- [`Dockerfile`](/Users/malachi/code/drone/Dockerfile): ROS 2 image definition and package installation
- [`docker-compose.yml`](/Users/malachi/code/drone/docker-compose.yml): Linux and macOS container profiles
- [`entrypoint.sh`](/Users/malachi/code/drone/entrypoint.sh): ROS environment bootstrap for the container
- [`start.sh`](/Users/malachi/code/drone/start.sh): host-side launcher for live LiDAR ingest
- [`lidar/launch_vlp16.py`](/Users/malachi/code/drone/lidar/launch_vlp16.py): live sensor launch description
- [`lidar/launch_pcap.py`](/Users/malachi/code/drone/lidar/launch_pcap.py): PCAP playback launch description
- [`lidar/launch_slam.py`](/Users/malachi/code/drone/lidar/launch_slam.py): LiDAR, TF, SLAM, OctoMap, and optional MAVROS/EKF launch description
- [`lidar/vlp16_live_params.yaml`](/Users/malachi/code/drone/lidar/vlp16_live_params.yaml): live driver parameters
- [`lidar/vlp16_pcap_params.yaml`](/Users/malachi/code/drone/lidar/vlp16_pcap_params.yaml): playback driver parameters
- [`lidar/config/static_transforms.yaml`](/Users/malachi/code/drone/lidar/config/static_transforms.yaml): rigid transform from vehicle body frame to LiDAR frame
- [`lidar/config/rtabmap.yaml`](/Users/malachi/code/drone/lidar/config/rtabmap.yaml): RTAB-Map and ICP odometry parameters
- [`lidar/config/octomap.yaml`](/Users/malachi/code/drone/lidar/config/octomap.yaml): OctoMap parameters
- [`lidar/config/ekf.yaml`](/Users/malachi/code/drone/lidar/config/ekf.yaml): EKF fusion parameters for MAVROS topics
- [`lidar/config/mavros.yaml`](/Users/malachi/code/drone/lidar/config/mavros.yaml): MAVROS transport and frame parameters
- [`USAGE.md`](/Users/malachi/code/drone/USAGE.md): technical reference for runtime behavior, topics, frames, IP assumptions, and parameter semantics

## Mounted Paths

- host `./lidar` -> container `/home/rosuser/ros_ws/src/lidar`
- host `./data` -> container `/home/rosuser/data`
- host `./bags` -> container `/home/rosuser/bags`
