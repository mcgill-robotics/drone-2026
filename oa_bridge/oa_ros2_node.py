"""
oa_ros2_node.py  —  Obstacle Avoidance node for Gazebo / PX4 SITL / ROS 2
=========================================================================
This is the Gazebo replacement for `oa_server.py` (which spoke to Webots over
a raw TCP socket). Instead of a socket it uses ROS 2 end-to-end:

    sensor_msgs/LaserScan   ─┐
    /mavros/local_position  ─┼─►  Robot3D (3-D potential-field OA)  ─► vel
    drone yaw (MAVROS)      ─┘                                        │
                                                                      ▼
                                  PX4Interface.send_velocity_setpoint()
                                  (/mavros/setpoint_velocity/cmd_vel, OFFBOARD)

The OA core (`oa_core.robot3d.Robot3D`) is reused unchanged — only the I/O
shell is different. Control goes through `mission_controller.PX4Interface`,
the same MAVROS wrapper the rest of the project uses.

Run:  python3 oa_ros2_node.py --target-x 20 --target-y 0 --target-z 3
(see `python3 oa_ros2_node.py --help`; PX4 SITL + Gazebo + MAVROS must be up —
 see README_GAZEBO.md)
"""

import argparse
import math
import os
import sys
import time

# ── import paths ─────────────────────────────────────────────────────────
# Make `oa_core` and `config` importable as top-level packages (same trick as
# oa_server.py), and the repo root importable so `mission_controller` resolves.
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_THIS_DIR)
sys.path.insert(0, _THIS_DIR)
sys.path.insert(0, _REPO_ROOT)

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from oa_core.robot3d import Robot3D
from oa_core.obstacle3d import Obstacle3D
from oa_core.vector3d import Vector3D
from config.robot_config import RobotConfig

from mission_controller.px4_interface import init_px4, boot_px4, stop_px4


# ── defaults ─────────────────────────────────────────────────────────────
DEFAULT_LIDAR_TOPIC = "/scan"          # ros_gz_bridge output (see README_GAZEBO.md)
DEFAULT_FCU_URL = "udp://:14540@127.0.0.1:14557"   # PX4 SITL <-> MAVROS
MAX_DETECT_RANGE = 8.0                 # metres — ignore lidar returns past this
LOOP_HZ = 10.0                         # control loop rate (matches PhysicsConfig.dt = 0.1)


class OAController:
    """
    Glues a LaserScan stream + MAVROS telemetry into the Robot3D OA core and
    pushes the resulting velocity to PX4 as an OFFBOARD setpoint.

    Frames
    ------
    Everything is done in the MAVROS local ENU frame ("map"): x = East,
    y = North, z = Up, origin at the home/arming point. The 2-D lidar scans in
    the drone body frame; each beam is rotated by the drone yaw and translated
    by the drone position into this ENU frame so obstacles and the drone share
    one coordinate system (Robot3D requires that).
    """

    def __init__(self, px4, target, lidar_topic=DEFAULT_LIDAR_TOPIC,
                 beam_stride=1, max_range=MAX_DETECT_RANGE):
        self.px4 = px4
        self.beam_stride = max(1, int(beam_stride))
        self.max_range = float(max_range)

        # OA core. Starting position is overwritten every tick from MAVROS, so
        # the (0,0,0) here is just a placeholder.
        self.robot = Robot3D(0.0, 0.0, 0.0, config=RobotConfig())
        self.robot.set_target(*target)
        self.target = target

        self.latest_scan = None
        self._scan_count = 0

        # Subscribe on the PX4 node itself — it is already an rclpy Node, so a
        # single executor/spin covers both MAVROS telemetry and the lidar.
        self.px4.create_subscription(
            LaserScan, lidar_topic, self._scan_cb, qos_profile_sensor_data
        )
        print(f"[OA] Subscribed to LaserScan on {lidar_topic}")
        print(f"[OA] Target (ENU map frame): x={target[0]} y={target[1]} z={target[2]}")

    # ── callbacks ────────────────────────────────────────────────────────
    def _scan_cb(self, msg):
        self.latest_scan = msg
        self._scan_count += 1

    # ── lidar → world-frame obstacles ────────────────────────────────────
    def scan_to_obstacles(self, scan, drone_pos, yaw):
        """
        Convert a sensor_msgs/LaserScan into a list of Obstacle3D in the ENU
        map frame.

        A 2-D lidar has no elevation, so every return is placed at the drone's
        current altitude — the OA core then treats them as obstacles in the
        horizontal plane, which is what a planar lidar actually measures.
        """
        obstacles = []
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        rmin = scan.range_min
        rmax = min(scan.range_max, self.max_range)

        for i, r in enumerate(scan.ranges):
            if i % self.beam_stride != 0:
                continue
            if not math.isfinite(r) or r < rmin or r > rmax:
                continue

            a = scan.angle_min + i * scan.angle_increment

            # Beam endpoint in body frame (lidar 0 rad = drone forward = body +x)
            bx = r * math.cos(a)
            by = r * math.sin(a)

            # Rotate body → ENU by yaw, translate by drone position
            wx = drone_pos.x + bx * cos_y - by * sin_y
            wy = drone_pos.y + bx * sin_y + by * cos_y
            wz = drone_pos.z

            obstacles.append(Obstacle3D(wx, wy, wz, charge=300.0, radius=0.2))

        return obstacles

    # ── one control tick ─────────────────────────────────────────────────
    def step(self):
        """Run one OA iteration and publish a velocity setpoint. Returns True
        once the target has been reached."""
        loc = self.px4.get_location()
        if loc is None:
            # No local pose yet — hold position, do not command motion.
            self.px4.send_velocity_setpoint(0.0, 0.0, 0.0, 0.0)
            return False

        pos = Vector3D(loc["x"], loc["y"], loc["z"])
        yaw = self.px4.get_current_yaw()

        # Feed real telemetry into the OA core. Robot3D integrates its own
        # model velocity from the avoidance force; position/yaw are corrected
        # from MAVROS each tick so the model never drifts from reality.
        self.robot.physics.position = pos.copy()
        self.robot.current_yaw = yaw

        if self.latest_scan is not None:
            self.robot.obstacles = self.scan_to_obstacles(self.latest_scan, pos, yaw)

        self.robot.update()

        if self.robot.course_completed:
            self.px4.send_velocity_setpoint(0.0, 0.0, 0.0, 0.0)
            return True

        v = self.robot.vel
        self.px4.send_velocity_setpoint(v.x, v.y, v.z, self.robot.yaw_rate)
        return False


def parse_args(argv):
    p = argparse.ArgumentParser(description="Obstacle-avoidance OA node for Gazebo/PX4/ROS2")
    p.add_argument("--target-x", type=float, required=True, help="Target East (m, ENU map frame)")
    p.add_argument("--target-y", type=float, required=True, help="Target North (m, ENU map frame)")
    p.add_argument("--target-z", type=float, default=3.0, help="Target Up / altitude (m)")
    p.add_argument("--takeoff-alt", type=float, default=3.0, help="Takeoff altitude (m)")
    p.add_argument("--lidar-topic", default=DEFAULT_LIDAR_TOPIC, help="LaserScan topic")
    p.add_argument("--beam-stride", type=int, default=1, help="Use every Nth lidar beam")
    p.add_argument("--max-range", type=float, default=MAX_DETECT_RANGE, help="Ignore returns past this range (m)")
    p.add_argument("--fcu-url", default=DEFAULT_FCU_URL, help="MAVROS FCU URL")
    p.add_argument("--boot-mavros", action="store_true",
                   help="Launch MAVROS via `ros2 launch mavros px4.launch` (otherwise assume it is already running)")
    p.add_argument("--no-land", action="store_true", help="Hover at target instead of landing")
    # rclpy/ROS args (e.g. --ros-args ...) are passed through and ignored here
    args, _ = p.parse_known_args(argv)
    return args


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    args = parse_args(argv)

    mavros_proc = None
    if args.boot_mavros:
        mavros_proc = boot_px4(fcu_url=args.fcu_url, suppress_output=False)
        if mavros_proc is None:
            print("[OA] Failed to boot MAVROS — aborting")
            return 1

    rclpy.init(args=argv)

    # PX4Interface (init_px4 also calls connect() and waits for the FCU heartbeat)
    px4 = init_px4(node_name="oa_gazebo_node")
    if px4 is None or not px4.connected:
        print("[OA] Could not connect to MAVROS. Is PX4 SITL + MAVROS running?")
        rclpy.shutdown()
        if mavros_proc is not None:
            stop_px4()
        return 1

    oa = OAController(
        px4,
        target=(args.target_x, args.target_y, args.target_z),
        lidar_topic=args.lidar_topic,
        beam_stride=args.beam_stride,
        max_range=args.max_range,
    )

    dt = 1.0 / LOOP_HZ
    exit_code = 0
    try:
        # ── arm + take off ────────────────────────────────────────────────
        # Background heartbeat keeps the >2 Hz setpoint stream OFFBOARD needs
        # alive even if a control tick stalls.
        px4.start_offboard_stream_background(rate_hz=LOOP_HZ)
        if not px4.start_offboard():
            print("[OA] Failed to enter OFFBOARD")
            return 1
        if not px4.arm_vehicle():
            print("[OA] Failed to arm")
            return 1
        if not px4.takeoff(args.takeoff_alt):
            print("[OA] Takeoff failed")
            return 1

        print(f"[OA] Airborne — running obstacle avoidance toward target "
              f"({args.target_x}, {args.target_y}, {args.target_z})")

        # ── main OA loop ──────────────────────────────────────────────────
        step = 0
        while rclpy.ok():
            rclpy.spin_once(px4, timeout_sec=dt)  # processes MAVROS + lidar callbacks

            arrived = oa.step()
            step += 1

            if step % int(LOOP_HZ * 5) == 0:
                v = oa.robot.vel
                loc = px4.get_location() or {"x": 0, "y": 0, "z": 0}
                print(f"[OA] t={step/LOOP_HZ:6.1f}s  "
                      f"pos=({loc['x']:+.1f},{loc['y']:+.1f},{loc['z']:+.1f})  "
                      f"vel=({v.x:+.2f},{v.y:+.2f},{v.z:+.2f})  "
                      f"obstacles={len(oa.robot.obstacles)}  scans={oa._scan_count}")

            if arrived:
                print(f"\n{'='*50}\n[OA] *** TARGET REACHED ***\n{'='*50}\n")
                break

        # ── finish ────────────────────────────────────────────────────────
        if args.no_land:
            print("[OA] Holding position at target")
            px4.hold_current_position()
        else:
            print("[OA] Landing")
            px4.land()

    except KeyboardInterrupt:
        print("\n[OA] Interrupted — holding position")
        px4.hold_current_position()
    except Exception as e:  # noqa: BLE001 - top-level guard
        print(f"[OA] ERROR: {e}")
        import traceback
        traceback.print_exc()
        exit_code = 1
    finally:
        px4.stop_offboard_stream_background()
        rclpy.shutdown()
        if mavros_proc is not None:
            stop_px4()

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
