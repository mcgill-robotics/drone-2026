# Technical Reference

This document defines the runtime behavior encoded in this repository: launch files, topics, transforms, parameters, Docker behavior, network assumptions, and the role of each configuration file.

## 1. Runtime Model

The repository contains three ROS entry points:

- `./start.sh`
- `ros2 launch /home/rosuser/ros_ws/src/lidar/launch_pcap.py`
- `ros2 launch /home/rosuser/ros_ws/src/lidar/launch_slam.py`

The container image is built from [`Dockerfile`](/Users/malachi/code/drone/Dockerfile), and runtime behavior is selected through Docker Compose profiles in [`docker-compose.yml`](/Users/malachi/code/drone/docker-compose.yml).

## 2. Network and IP Assumptions

The current repository contents encode the following sensor-side assumptions:

- LiDAR model: Velodyne VLP-16
- data transport: UDP
- data port: `2368`
- reference sensor IP: `192.168.1.201`
- example host address on the LiDAR subnet: `192.168.1.100/24`

The live driver parameter file leaves `device_ip` unset:

- `device_ip: ""`

That setting accepts packets from any source address. It is required for the macOS relay path because packets are re-emitted from `127.0.0.1` inside the container instead of arriving directly from `192.168.1.201`.

The macOS relay path adds one TCP hop:

- host receives UDP `2368`
- host forwards to TCP `127.0.0.1:12368`
- container relay reads TCP `12368`
- container relay re-emits UDP to `127.0.0.1:2368`

The visualization bridge listens on:

- Foxglove WebSocket: `ws://localhost:8765`

## 3. Docker Profiles

### Linux profile

- service name: `lidar-linux`
- networking mode: `host`
- display path: X11 socket mounted into the container

### macOS profile

- service name: `lidar-mac`
- container architecture: `linux/amd64`
- networking mode: bridged Docker networking with explicit TCP port publishing

### Mounted paths

- host `./lidar` -> container `/home/rosuser/ros_ws/src/lidar`
- host `./data` -> container `/home/rosuser/data`
- host `./bags` -> container `/home/rosuser/bags`

These mounts are part of the operational contract. Launch files and parameter files in this repository assume those in-container paths exist.

## 4. Launch Files

### `lidar/launch_vlp16.py`

Purpose: live Velodyne ingest and visualization bridge.

Nodes started:

- `velodyne_driver/velodyne_driver_node`
- `velodyne_pointcloud/velodyne_transform_node`
- `velodyne_laserscan/velodyne_laserscan_node`
- `foxglove_bridge/foxglove_bridge`

Parameter sources:

- live driver parameters come from [`lidar/vlp16_live_params.yaml`](/Users/malachi/code/drone/lidar/vlp16_live_params.yaml)
- point cloud conversion parameters are loaded from the installed `velodyne_pointcloud` package, then the calibration path is overridden to `VLP16db.yaml`
- LaserScan parameters come from the installed `velodyne_laserscan` package defaults

### `lidar/launch_pcap.py`

Purpose: replay a recorded PCAP as if it were live sensor traffic.

Differences relative to the live launch:

- the driver parameter file is [`lidar/vlp16_pcap_params.yaml`](/Users/malachi/code/drone/lidar/vlp16_pcap_params.yaml)
- the launch shuts the system down when the driver process exits

Operational note:

- the PCAP path is hard-coded in the YAML file to `/home/rosuser/data/...pcap`, so playback only works if the matching file exists under the mounted host `data/` directory

### `lidar/launch_slam.py`

Purpose: run the full LiDAR, TF, SLAM, and occupancy-mapping process defined by the repository.

Nodes started unconditionally:

- static transform publisher for `base_link -> velodyne`
- `velodyne_driver_node`
- `velodyne_transform_node`
- `velodyne_laserscan_node`
- `rtabmap_slam/rtabmap`
- `octomap_server/octomap_server_node`
- `foxglove_bridge`

Nodes started conditionally:

- `rtabmap_odom/icp_odometry` when `use_icp_odom:=true`
- `robot_localization/ekf_node` when `use_mavros:=true`
- `mavros/mavros_node` when `use_mavros:=true`

Launch arguments:

- `use_mavros` default `false`
- `use_icp_odom` default `true`

Expected frame graph:

- `map -> odom -> base_link -> velodyne`

Responsibilities by process:

- static transform: defines sensor mounting geometry
- driver and transform nodes: convert UDP packets to `PointCloud2`
- ICP odometry: provides LiDAR-only odometry when flight-controller odometry is not being used
- RTAB-Map: builds the SLAM graph and map representation
- OctoMap: produces a 3D occupancy grid from the incoming point cloud plus TF
- EKF and MAVROS: integrate external state estimation from the flight controller when enabled

## 5. Topic and Frame Contract

### Core topics

| Topic | Type | Producer | Consumer |
|---|---|---|---|
| `/velodyne_packets` | `velodyne_msgs/VelodyneScan` | `velodyne_driver_node` | `velodyne_transform_node` |
| `/velodyne_points` | `sensor_msgs/PointCloud2` | `velodyne_transform_node` | Foxglove, RTAB-Map, OctoMap |
| `/scan` | `sensor_msgs/LaserScan` | `velodyne_laserscan_node` | optional 2D tooling |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | driver nodes | operators |

### MAVROS and localization topics

These are only relevant when `use_mavros:=true`.

| Topic | Type | Consumer |
|---|---|---|
| `/mavros/imu/data` | `sensor_msgs/Imu` | EKF |
| `/mavros/local_position/odom` | `nav_msgs/Odometry` | EKF |

### Frame assumptions

- `base_link` is the vehicle body frame used by localization and SLAM
- `velodyne` is the LiDAR frame
- `odom` is the locally consistent odometry frame
- `map` is the global SLAM frame

The repository assumes the vehicle can supply or derive the transform chain needed by RTAB-Map and OctoMap. If TF is incomplete or inconsistent, the resulting point cloud registration and occupancy map will be incorrect.

## 6. Configuration Files

### `lidar/vlp16_live_params.yaml`

Role: live sensor driver configuration.

Parameters currently set:

- `device_ip: ""`
  Accept packets from any source. This accommodates direct packets from `192.168.1.201` on Linux and relayed packets from `127.0.0.1` on macOS.
- `gps_time: false`
  Use host receipt time instead of GPS-derived timestamps.
- `time_offset: 0.0`
  No timestamp correction is applied.
- `enabled: true`
  Driver starts immediately.
- `read_once: false`
  Continue reading indefinitely.
- `read_fast: false`
  Use normal packet timing rather than maximum-rate playback.
- `repeat_delay: 0.0`
  No delay is inserted between playback loops. This is effectively unused in live mode.
- `frame_id: velodyne`
  The frame attached to outgoing packets and derived point cloud data.
- `model: VLP16`
  Selects the correct Velodyne model.
- `rpm: 600.0`
  Sensor spin rate. Must match the hardware setting.
- `port: 2368`
  UDP data port expected from the sensor or relay.
- `timestamp_first_packet: false`
  Packet timestamps are not anchored to the first packet in a scan.

### `lidar/vlp16_pcap_params.yaml`

Role: playback configuration for recorded packet captures.

Parameters of note:

- `pcap`
  Absolute in-container path to the capture file.
- `read_once: false`
  The current configuration loops rather than exiting after one pass.
- `read_fast: false`
  Playback runs in recorded time rather than as fast as possible.
- `frame_id`, `model`, `rpm`, and `port`
  Must remain coherent with the capture source and downstream TF assumptions.

### `lidar/config/static_transforms.yaml`

Role: define the rigid transform from `base_link` to `velodyne`.

Parameters:

- `x`, `y`, `z`: translational offset in meters
- `roll`, `pitch`, `yaw`: rotational offset in radians
- `parent_frame`: expected to remain `base_link`
- `child_frame`: expected to remain `velodyne`

This file is one of the highest-impact configuration surfaces in the repository. If the mount geometry is wrong, every downstream consumer sees a geometrically incorrect point cloud.

### `lidar/config/rtabmap.yaml`

Role: configure RTAB-Map SLAM and ICP odometry.

`rtabmap.ros__parameters`:

- `frame_id`, `odom_frame_id`, `map_frame_id`
  Define the SLAM TF interfaces.
- `subscribe_scan_cloud: true`
  Uses point cloud input rather than RGB-D or 2D scan input.
- `approx_sync: true`
  Allows approximate synchronization of subscribed inputs.
- `Icp/VoxelSize: "0.1"`
  Downsamples points into 10 cm voxels before ICP. Lower values preserve detail at higher compute cost.
- `Icp/MaxCorrespondenceDistance: "1.0"`
  Maximum point-pair matching distance in meters.
- `Icp/PointToPlane: "true"`
  Uses point-to-plane alignment, which generally performs better on structured 3D geometry.
- `Icp/Iterations: "30"`
  Maximum ICP iterations per update.
- `Rtabmap/DetectionRate: "1.0"`
  Caps processing to roughly one keyframe update per second.
- `Reg/Strategy: "1"`
  Selects ICP-based registration.
- `Grid/CellSize: "0.1"`
  Occupancy grid cell size in meters.
- `Grid/RangeMax: "50.0"`
  Maximum LiDAR range included in grid building.
- `Grid/3D: "true"`
  Enables 3D grid generation.

`icp_odometry.ros__parameters`:

- `publish_tf: true`
  Publishes the odometry transform directly.
- `wait_for_transform: 0.2`
  Maximum TF wait time in seconds before processing.
- `Odom/Strategy: "0"`
  Uses ICP odometry.
- `OdomF2M/ScanMaxSize: "15000"`
  Caps the point count used in frame-to-map matching.
- `OdomF2M/ScanSubtractRadius: "0.1"`
  Removes near-duplicate points within 10 cm.

### `lidar/config/octomap.yaml`

Role: configure the occupancy mapper.

Parameters of note:

- `resolution: 0.1`
  Voxel resolution in meters.
- `frame_id: map`
  Output map frame.
- `base_frame_id: base_link`
  Vehicle body frame used by the mapper.
- `sensor_model/max_range: 50.0`
  Drops returns beyond 50 m.
- `sensor_model/hit: 0.7`
  Occupancy probability update for hits.
- `sensor_model/miss: 0.4`
  Occupancy probability update for misses.
- `sensor_model/min`, `sensor_model/max`
  Clamp bounds for occupancy probability.
- `filter_ground: true`
  Enables ground-plane filtering before map integration.
- `ground_filter/distance`, `ground_filter/plane_distance`
  Ground separation thresholds in meters.
- `pointcloud_min_z`, `pointcloud_max_z`
  Vertical filtering window in meters.

These values determine the trade-off between map density, noise rejection, and computational cost.

### `lidar/config/ekf.yaml`

Role: configure `robot_localization` for fusing MAVROS IMU and odometry.

Key parameters:

- `frequency: 30.0`
  EKF update frequency in Hz.
- `two_d_mode: false`
  Full 3D estimation.
- `publish_tf: true`
  The filter publishes the `odom -> base_link` transform.
- `world_frame: odom`
  The EKF outputs a locally consistent frame, not a globally corrected map frame.
- `imu0`, `odom0`
  Input topics expected from MAVROS.
- `imu0_config`, `odom0_config`
  Per-state selection masks defining which parts of each message are fused.

This file is only valid if the MAVROS topics exist and are aligned with the vehicle frame conventions. A mismatch can produce physically incorrect state estimates.

### `lidar/config/mavros.yaml`

Role: configure the flight-controller bridge.

Parameters:

- `fcu_url`
  Connection string for the flight controller. The current default is a serial link on `/dev/ttyACM0` at `921600`.
- `gcs_url`
  Optional second MAVLink endpoint.
- `target_system_id`, `target_component_id`
  MAVLink addressing.
- `fcu_protocol`
  MAVLink protocol version.
- `imu/frame_id`
  Frame attached to published IMU data.
- `local_position/frame_id`
  Frame attached to local position outputs.
- `local_position/tf/send: false`
  TF publication is disabled because TF is expected to come from `robot_localization`.

## 7. Docker and Host Integration

### `Dockerfile`

Installs:

- ROS 2 Humble desktop base image
- Velodyne driver packages
- PCL and TF support packages
- Foxglove bridge
- RTAB-Map
- OctoMap packages
- `robot_localization`
- MAVROS and its GeographicLib datasets

The image does not build a local ROS workspace. The repository is mounted directly into the container and run from source.

### `docker-compose.yml`

Defines two profiles with the same mounted workspace but different networking behavior.

Implementation detail:

- both services share the same `container_name: suas-lidar`

Only one profile should be active at a time, because both resolve to the same container name.

### `start.sh`

Execution sequence:

1. detect host OS
2. start the correct Compose profile
3. on macOS, ensure `socat` is present
4. on macOS, start `udp_relay.py` inside the container
5. on macOS, start a host `socat` process to tunnel UDP `2368` to container TCP `12368`
6. run `launch_vlp16.py` inside the container
7. hold until interrupted, then tear down the Compose services

### `lidar/udp_relay.py`

This file exists only to work around Docker Desktop networking on macOS.

Behavior:

- accepts a TCP stream on port `12368`
- chunks the stream into fixed `1206` byte Velodyne packets
- re-emits each packet as UDP to `127.0.0.1:2368`

The fixed packet size is specific to the Velodyne VLP-16 packet format. If the repository is later adapted for another LiDAR model, this file is one of the places that must be revalidated.

## 8. File Inventory

| Path | Role |
|---|---|
| `Dockerfile` | Build the ROS 2 runtime image |
| `docker-compose.yml` | Select Linux or macOS runtime profile |
| `entrypoint.sh` | Source ROS setup files before command execution |
| `start.sh` | Host-side launcher for live ingest with OS-specific networking behavior |
| `lidar/launch_vlp16.py` | Live LiDAR ingest launch file |
| `lidar/launch_pcap.py` | Offline PCAP playback launch file |
| `lidar/launch_slam.py` | SLAM, mapping, and optional MAVROS integration launch file |
| `lidar/udp_relay.py` | macOS network adaptation for Velodyne UDP packets |
| `lidar/vlp16_live_params.yaml` | Live driver configuration |
| `lidar/vlp16_pcap_params.yaml` | Playback driver configuration |
| `lidar/config/static_transforms.yaml` | Vehicle-to-sensor rigid transform |
| `lidar/config/rtabmap.yaml` | RTAB-Map and ICP odometry settings |
| `lidar/config/octomap.yaml` | OctoMap settings |
| `lidar/config/ekf.yaml` | EKF fusion settings for MAVROS inputs |
| `lidar/config/mavros.yaml` | MAVROS connection settings |

## 9. Change Boundaries

The following settings are expected to vary by aircraft or deployment:

- `static_transforms.yaml`
- `mavros.yaml`
- `ekf.yaml`
- `vlp16_live_params.yaml`
- `rtabmap.yaml`
- `octomap.yaml`

The following files are infrastructure and should only change when the runtime model changes:

- `Dockerfile`
- `docker-compose.yml`
- `entrypoint.sh`
- `start.sh`
- `udp_relay.py`
