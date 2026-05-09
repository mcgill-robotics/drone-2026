"""
mavic2pro.py  —  Webots Python controller with integrated obstacle avoidance
=============================================================================
Replaces mavic2pro.c + oa_server.py.  Runs entirely in-process:
  - reads Webots sensors (IMU, GPS, gyro, LiDAR, keyboard)
  - feeds LiDAR points directly into Robot3D OA logic
  - applies resulting velocity commands through the PID motor mixer
"""

import math
import sys
import os

# Allow imports from oa_bridge/ (two levels up from controllers/mavic2pro/)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from controller import Robot, Keyboard

from oa_core.robot3d     import Robot3D
from oa_core.obstacle3d  import Obstacle3D
from config.robot_config import RobotConfig

# ── target waypoint ────────────────────────────────────────────────────────
TARGET_X = 5.0
TARGET_Y = 0.0
TARGET_Z = 1.5

# ── flight constants ────────────────────────────────────────────────────────
K_VERTICAL_THRUST = 68.5
K_VERTICAL_OFFSET = 0.6
K_VERTICAL_P      = 3.0
K_ROLL_P          = 50.0
K_PITCH_P         = 30.0

CLAMP = lambda v, lo, hi: max(lo, min(hi, v))


def filter_lidar_points(cloud) -> list[Obstacle3D]:
    obstacles = []
    for pt in cloud:
        x, y, z = pt.x, pt.y, pt.z

        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        if abs(x) > 50 or abs(y) > 50 or abs(z) > 50:
            continue
        if z < -0.5:
            continue

        dist = math.sqrt(x*x + y*y + z*z)
        if dist < 0.5 or dist > 6.0:
            continue

        obstacles.append(Obstacle3D(x, y, z, charge=300.0, radius=0.2))

    return obstacles


def motor_mix(vert, roll_in, pitch_in, yaw_in):
    fl = K_VERTICAL_THRUST + vert - roll_in + pitch_in - yaw_in
    fr = -(K_VERTICAL_THRUST + vert + roll_in + pitch_in + yaw_in)
    rl = -(K_VERTICAL_THRUST + vert - roll_in - pitch_in + yaw_in)
    rr = K_VERTICAL_THRUST + vert + roll_in - pitch_in - yaw_in
    return fl, fr, rl, rr


def run():
    robot    = Robot()
    timestep = int(robot.getBasicTimeStep())

    # ── devices ───────────────────────────────────────────────────────────
    imu = robot.getDevice("inertial unit")
    imu.enable(timestep)

    gps = robot.getDevice("gps")
    gps.enable(timestep)

    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)

    kb = robot.getDevice("keyboard")
    kb.enable(timestep)

    lidar = robot.getDevice("lidar")
    lidar.enable(timestep)
    lidar.enablePointCloud()

    camera = robot.getDevice("camera")
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
    config   = RobotConfig()
    oa_robot = Robot3D(0.0, 0.0, 1.0, config=config)
    oa_robot.set_target(TARGET_X, TARGET_Y, TARGET_Z)

    target_altitude = 1.0
    oa_pitch = oa_roll = oa_yaw = oa_vz = 0.0

    print("[OA] Mavic 2 Pro Python controller active")

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
            oa_robot.obstacles = filter_lidar_points(cloud)

        oa_robot.physics.position.x = gps_vals[0]
        oa_robot.physics.position.y = gps_vals[1]
        oa_robot.physics.position.z = gps_vals[2]
        oa_robot.current_yaw        = yaw_imu

        oa_robot.update()

        if oa_robot.course_completed:
            print(f"[OA] *** ARRIVED AT TARGET ***  pos={oa_robot.pos}")

        vel      = oa_robot.vel
        oa_pitch = -vel.x * 0.5
        oa_roll  =  vel.y * 0.5
        oa_yaw   =  oa_robot.yaw_rate
        oa_vz    =  vel.z

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
                  f"vx={vel.x:+.2f}  vy={vel.y:+.2f}  vz={vel.z:+.2f}  "
                  f"obs={len(oa_robot.obstacles)}")


if __name__ == "__main__":
    run()
