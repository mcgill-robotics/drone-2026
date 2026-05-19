# OA on Gazebo + PX4 SITL + ROS 2

This is the Gazebo port of the obstacle-avoidance bridge. The original
`oa_server.py` spoke to **Webots** over a raw TCP socket; this version runs the
same OA core (`oa_core/robot3d.py`) but uses **ROS 2 topics** end-to-end and
commands the drone through PX4 via MAVROS.

```
 PX4 SITL ── gz_x500_lidar_2d ──┐
   │                            │ gz.msgs.LaserScan
   │ MAVLink (udp)              ▼
   ▼                       ros_gz_bridge
 MAVROS  ──────────┐            │ sensor_msgs/LaserScan
   /local_position │            ▼
   /setpoint_vel  ◄┼──── oa_ros2_node.py  (Robot3D potential-field OA)
                   │     · LaserScan      -> world-frame Obstacle3D
                   └──── · MAVROS pose    -> drone position / yaw
                         · Robot3D.update -> velocity setpoint (OFFBOARD)
```

Nothing in `oa_core/` changed — only the I/O shell (`oa_ros2_node.py`) is new.
Control reuses `mission_controller.PX4Interface`, the project's existing MAVROS
wrapper.

## Files

| File | Purpose |
| ---- | ------- |
| `oa_ros2_node.py` | The OA controller node (replaces `oa_server.py`). |
| `launch/oa_gazebo.launch.py` | Brings up MAVROS + lidar bridge + OA node. |
| `README_GAZEBO.md` | This file. |

## Prerequisites

- ROS 2 (Humble or newer) with `rclpy`
- `mavros` + `mavros_extras` (`ros-$ROS_DISTRO-mavros*`) — already used by `mission_controller`
- `ros_gz_bridge` (`ros-$ROS_DISTRO-ros-gz-bridge`)
- PX4-Autopilot with Gazebo (gz-sim) SITL targets built

Install the GeographicLib datasets MAVROS needs once:

```bash
ros2 run mavros install_geographiclib_datasets.sh
```

## 1. Start PX4 SITL + Gazebo

PX4 ships an airframe with a 2-D lidar already mounted — `x500_lidar_2d`:

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500_lidar_2d
```

This opens Gazebo with an X500 quad carrying a planar lidar, and PX4 starts
offering MAVLink on UDP `14540`.

> **No lidar model?** Older PX4 trees may not have `x500_lidar_2d`. Either
> update PX4-Autopilot, or attach a `gpu_lidar` sensor to the `x500` model SDF
> yourself — anything that publishes `gz.msgs.LaserScan` works.

Find the exact gz lidar topic (it includes the world and model name):

```bash
gz topic -l | grep scan
```

Copy that topic — you pass it to the launch file as `gz_lidar_topic`.

## 2. Launch the ROS 2 OA stack

```bash
ros2 launch oa_bridge/launch/oa_gazebo.launch.py \
    target_x:=20.0 target_y:=0.0 target_z:=3.0 \
    gz_lidar_topic:=<topic from `gz topic -l`>
```

The launch file starts three things:

1. **MAVROS** — `fcu_url:=udp://:14540@127.0.0.1:14557`
2. **`ros_gz_bridge`** — bridges the gz lidar to `/scan` (`sensor_msgs/LaserScan`)
3. **`oa_ros2_node.py`** — arms, takes off, then avoids obstacles toward the target

### Or run the node by hand

If MAVROS and the bridge are already running:

```bash
python3 oa_bridge/oa_ros2_node.py --target-x 20 --target-y 0 --target-z 3
```

`oa_ros2_node.py --help` lists every flag. Useful ones:

| Flag | Default | Meaning |
| ---- | ------- | ------- |
| `--target-x/y/z` | — | Target in the MAVROS ENU **map** frame (origin = arm point) |
| `--takeoff-alt` | `3.0` | Altitude to climb to before OA starts (m) |
| `--lidar-topic` | `/scan` | LaserScan topic |
| `--beam-stride` | `1` | Use every Nth lidar beam (raise it to thin dense scans) |
| `--max-range` | `8.0` | Ignore lidar returns past this range (m) |
| `--voxel-size` | `0.4` | Downsample lidar obstacles to one point per this XY cell (m) |
| `--scan-timeout` | `0.5` | Hover in place if no LaserScan arrives within this many seconds |
| `--brake-distance` | `0.8` | Hard-stop: brake to a hover if any lidar return is closer than this (m) |
| `--fcu-url` | `udp://:14540@127.0.0.1:14557` | MAVROS FCU URL |
| `--boot-mavros` | off | Launch MAVROS itself instead of assuming it runs |
| `--no-land` | off | Hover at the target instead of landing |

## Coordinate frames

Everything runs in the **MAVROS local ENU frame** ("map"): `x` = East,
`y` = North, `z` = Up, origin at the home/arming point.

- **Targets** are given in this frame. `target_x:=20` means 20 m East of where
  the drone armed.
- **Lidar** beams are measured in the drone body frame; `oa_ros2_node.py`
  rotates each beam by the live drone yaw and translates it by the drone
  position so obstacles land in the same ENU frame as the drone (Robot3D
  requires obstacles and the drone to share one frame).
- A 2-D lidar has no elevation, so every return is placed at the drone's
  current altitude — OA reacts in the horizontal plane only. `target_z` is
  still tracked by the goal force, so the drone holds/changes altitude
  independently of avoidance.

## How a control tick works

At 10 Hz (matching `PhysicsConfig.dt = 0.1`):

1. Read drone position + yaw from MAVROS (`/mavros/local_position/pose`).
2. Convert the latest `LaserScan` into world-frame `Obstacle3D` points.
3. Feed both into `Robot3D` and call `.update()` — potential-field avoidance.
4. Publish `Robot3D.vel` + `yaw_rate` as a velocity setpoint via
   `PX4Interface.send_velocity_setpoint()` (`/mavros/setpoint_velocity/cmd_vel`).
5. When the drone is within `arrival_radius` (0.5 m) of the target, stop and
   land (or hover with `--no-land`).

A background heartbeat thread keeps the >2 Hz setpoint stream that PX4 OFFBOARD
requires alive even if a tick stalls.

## Tuning

Same knobs as the Webots bridge — all in `oa_bridge/config/`:

| File | What |
| ---- | ---- |
| `physics_config.py` | `max_speed` (3 m/s), `max_force`, `dt` |
| `avoidance_config.py` | `scan_radius` — how far ahead obstacles are reacted to |
| `navigation_config.py` | `arrival_radius`, `slow_radius` |

If avoidance feels too twitchy in dense scans, raise `--beam-stride` (e.g. `3`)
to subsample the lidar before it reaches the OA core.

## Troubleshooting

| Symptom | Check |
| ------- | ----- |
| `Could not connect to MAVROS` | Is PX4 SITL running? Does `ros2 topic echo /mavros/state` show `connected: true`? |
| `obstacles=0` always, `scans=0` | Wrong `gz_lidar_topic`. Re-run `gz topic -l \| grep scan` and `ros2 topic echo /scan`. |
| Drone arms but won't move | OFFBOARD not held — confirm `/mavros/setpoint_velocity/cmd_vel` is publishing at ~10 Hz. |
| Drone flies through obstacles | Lidar returning `inf`/out of range, or `--max-range` too small for the scene. |
| Takeoff times out | `/mavros/local_position/pose` not publishing — PX4 needs an EKF position estimate (SITL has one by default). |

## Not ported

- **Mission integration** — this node flies a single point-to-point OA run.
  Wiring OA into `MissionController.safe_goto` (see the static-avoider design
  in `README.md`) is a separate task.
- **3-D avoidance** — a 2-D lidar only sees one plane. Swap in a 3-D lidar or
  depth camera (`PointCloud2`) and elevation handling for full 3-D OA.
