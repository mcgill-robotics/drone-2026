"""
mavic2pro.py  —  Webots Python controller with integrated obstacle avoidance
=============================================================================
Replaces mavic2pro.c + oa_server.py.  Runs entirely in-process:
  - reads Webots sensors (IMU, GPS, gyro, LiDAR, keyboard)
  - feeds LiDAR points directly into Robot3D OA logic
  - applies resulting velocity commands through the PID motor mixer

Set this file as the Webots controller for the Mavic 2 Pro node.
"""

import math
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from controller import Robot, InertialUnit, GPS, Gyro, Keyboard, Lidar, Camera, Motor

from oa_core.robot3d     import Robot3D
from oa_core.obstacle3d  import Obstacle3D
from oa_core.vector3d    import Vector3D
from config.robot_config import RobotConfig

# ── target: point behind the SmallManor (-50.35, 11.25, 0) ────────────────
TARGET_X = -70.0
TARGET_Y =  15.0
TARGET_Z =   2.0

# ── desired cruise speed (m/s) ─────────────────────────────────────────────
MAX_SPEED = 20.0

# ── flight constants ────────────────────────────────────────────────────────
K_VERTICAL_THRUST = 68.5
K_VERTICAL_OFFSET = 0.6
K_VERTICAL_P      = 3.0
K_ROLL_P          = 50.0
K_PITCH_P         = 30.0

CLAMP = lambda v, lo, hi: max(lo, min(hi, v))


def filter_lidar_points(cloud, drone_x, drone_y, drone_z, drone_yaw) -> list[Obstacle3D]:
    """Transform body-frame LiDAR points into world frame and filter."""
    obstacles = []
    cos_y = math.cos(drone_yaw)
    sin_y = math.sin(drone_yaw)

    for pt in cloud:
        x, y, z = pt.x, pt.y, pt.z

        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue

        # Body-frame distance filtering (range gating)
        dist = math.sqrt(x*x + y*y + z*z)
        if dist < 0.5 or dist > 18.0:   # match LiDAR maxRange=20m
            continue
        if z < -0.5:                    # ignore ground returns
            continue

        # Body → world (rotate around Z by yaw, translate by drone pos)
        wx = drone_x + cos_y * x - sin_y * y
        wy = drone_y + sin_y * x + cos_y * y
        wz = drone_z + z

        # Larger radius/charge → avoidance triggers sooner at high speed
        obstacles.append(Obstacle3D(wx, wy, wz, charge=600.0, radius=0.5))

    return obstacles


def motor_mix(vert, roll_in, pitch_in, yaw_in) -> tuple:
    fl = K_VERTICAL_THRUST + vert - roll_in + pitch_in - yaw_in
    fr = -(K_VERTICAL_THRUST + vert + roll_in + pitch_in + yaw_in)
    rl = -(K_VERTICAL_THRUST + vert - roll_in - pitch_in + yaw_in)
    rr = K_VERTICAL_THRUST + vert + roll_in - pitch_in - yaw_in
    return fl, fr, rl, rr


def run():
    robot    = Robot()
    timestep = int(robot.getBasicTimeStep())

    # ── devices ───────────────────────────────────────────────────────────
    imu   = robot.getDevice("inertial unit")
    imu.enable(timestep)

    gps   = robot.getDevice("gps")
    gps.enable(timestep)

    gyro  = robot.getDevice("gyro")
    gyro.enable(timestep)

    kb = robot.getKeyboard()
    kb.enable(timestep)

    lidar = robot.getDevice("lidar")
    lidar.enable(timestep)
    lidar.enablePointCloud()

    camera       = robot.getDevice("camera")
    camera.enable(timestep)
    camera_roll  = robot.getDevice("camera roll")
    camera_pitch = robot.getDevice("camera pitch")

    motor_names = [
        "front left propeller",
        "front right propeller",
        "rear left propeller",
        "rear right propeller",
    ]
    motors = [robot.getDevice(n) for n in motor_names]
    for m in motors:
        m.setPosition(float("inf"))
        m.setVelocity(1.0)

    # ── wait for sensors to settle ────────────────────────────────────────
    while robot.step(timestep) != -1:
        if robot.getTime() > 1.0:
            break

    # ── OA logic ──────────────────────────────────────────────────────────
    config  = RobotConfig()
    config.physics.max_speed = MAX_SPEED
    oa_robot = Robot3D(0.0, 0.0, 1.0, config=config)
    oa_robot.set_target(TARGET_X, TARGET_Y, TARGET_Z)

    target_altitude = 1.0
    oa_pitch = oa_roll = oa_yaw = oa_vz = 0.0
    prev_pos = None        # for finite-difference velocity estimate

    print("[OA] Mavic 2 Pro Python controller active")
    print(f"[OA] Target: ({TARGET_X}, {TARGET_Y}, {TARGET_Z})")

    step = 0
    while robot.step(timestep) != -1:

        # ── 1. sensors ────────────────────────────────────────────────
        rpy       = imu.getRollPitchYaw()
        roll      = rpy[0]
        pitch     = rpy[1]
        yaw_imu   = rpy[2]
        gps_vals  = gps.getValues()
        altitude  = gps_vals[2]
        roll_vel  = gyro.getValues()[0]
        pitch_vel = gyro.getValues()[1]

        # ── 2. LiDAR → OA ─────────────────────────────────────────────
        cloud = lidar.getPointCloud()
        if cloud:
            oa_robot.obstacles = filter_lidar_points(
                cloud, gps_vals[0], gps_vals[1], gps_vals[2], yaw_imu
            )

        # Sync OA position AND velocity to reality so internal physics doesn't fight GPS
        dt = timestep / 1000.0
        if prev_pos is None:
            actual_vel = Vector3D(0, 0, 0)
        else:
            actual_vel = Vector3D(
                (gps_vals[0] - prev_pos[0]) / dt,
                (gps_vals[1] - prev_pos[1]) / dt,
                (gps_vals[2] - prev_pos[2]) / dt,
            )
        prev_pos = (gps_vals[0], gps_vals[1], gps_vals[2])

        oa_robot.physics.position.x = gps_vals[0]
        oa_robot.physics.position.y = gps_vals[1]
        oa_robot.physics.position.z = gps_vals[2]
        oa_robot.physics.velocity   = actual_vel
        oa_robot.smoothed_vel       = actual_vel
        oa_robot.current_yaw        = yaw_imu

        # Smooth takeoff: gain ramps 0 → 1 between altitude 0.8m and 1.5m
        gain = CLAMP((altitude - 0.8) / 0.7, 0.0, 1.0)
        if gain <= 0.0:
            oa_robot.last_force = Vector3D(0, 0, 0)

        oa_robot.update()

        if oa_robot.course_completed:
            print(f"[OA] *** ARRIVED AT TARGET ***  pos={oa_robot.pos}")

        # Use the OA's *desired* velocity delta (target - current) for pitch/roll
        desired = oa_robot.vel - actual_vel
        # Rotate world-frame delta into body frame using current yaw
        cos_y = math.cos(yaw_imu)
        sin_y = math.sin(yaw_imu)
        body_vx =  cos_y * desired.x + sin_y * desired.y
        body_vy = -sin_y * desired.x + cos_y * desired.y

        oa_pitch = CLAMP(-body_vx * 0.6 * gain, -2.0, 2.0)
        oa_roll  = CLAMP( body_vy * 0.6 * gain, -2.0, 2.0)
        oa_yaw   = CLAMP(oa_robot.yaw_rate * gain, -1.0, 1.0)
        oa_vz    = desired.z * gain

        target_altitude += oa_vz * (timestep / 1000.0)
        target_altitude  = CLAMP(target_altitude, 0.3, 20.0)

        # ── 3. keyboard override ───────────────────────────────────────
        roll_disturbance  = oa_roll
        pitch_disturbance = oa_pitch
        yaw_disturbance   = oa_yaw

        key = kb.getKey()
        while key > 0:
            if key == ord("Q"):
                robot.simulationQuit(0)
            elif key == Keyboard.UP:
                pitch_disturbance = -2.0
            elif key == Keyboard.DOWN:
                pitch_disturbance =  2.0
            elif key == Keyboard.RIGHT:
                yaw_disturbance   = -1.3
            elif key == Keyboard.LEFT:
                yaw_disturbance   =  1.3
            elif key == Keyboard.SHIFT + Keyboard.UP:
                target_altitude  += 0.05
            elif key == Keyboard.SHIFT + Keyboard.DOWN:
                target_altitude  -= 0.05
            key = kb.getKey()

        # ── 4. PID + motor mix ─────────────────────────────────────────
        roll_input  = K_ROLL_P  * CLAMP(roll,  -1.0, 1.0) + roll_vel  + roll_disturbance
        pitch_input = K_PITCH_P * CLAMP(pitch, -1.0, 1.0) + pitch_vel + pitch_disturbance
        yaw_input   = yaw_disturbance

        clamped_alt = CLAMP(target_altitude - altitude + K_VERTICAL_OFFSET, -1.0, 1.0)
        vert_input  = K_VERTICAL_P * (clamped_alt ** 3)

        speeds = motor_mix(vert_input, roll_input, pitch_input, yaw_input)
        for m, spd in zip(motors, speeds):
            m.setVelocity(spd)

        camera_roll.setPosition( -0.115 * roll_vel)
        camera_pitch.setPosition(-0.1   * pitch_vel)

        step += 1
        if step % 50 == 0:
            print(f"[OA] step={step:5d}  "
                  f"vx={actual_vel.x:+.2f}  vy={actual_vel.y:+.2f}  vz={actual_vel.z:+.2f}  "
                  f"obs={len(oa_robot.obstacles)}")


if __name__ == "__main__":
    run()
