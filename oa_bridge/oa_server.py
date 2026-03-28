"""
oa_server.py  —  Standalone Obstacle Avoidance Server
======================================================
Replaces the pygame Simulation loop.
- Listens on TCP port 65432 for a connection from mavic2pro.c (Webots)
- Each cycle:
    1. Receives a JSON packet of LiDAR points from Webots
    2. Feeds them into the 3-D OA logic
    3. Sends back a JSON movement command  { vx, vy, vz, yaw_rate }

Run BEFORE starting the Webots simulation:
    python oa_server.py
"""

import socket
import json
import math
import sys
import os

# Allow imports from the rest of the OA project
sys.path.insert(0, os.path.dirname(__file__))

from oa_core.robot3d        import Robot3D
from oa_core.obstacle3d     import Obstacle3D
from config.robot_config    import RobotConfig

HOST = "127.0.0.1"
PORT = 65432
BUFFER_SIZE = 65536      # 64 KB  – enough for a dense LiDAR scan

'''
def parse_lidar_packet(raw: str) -> list[Obstacle3D]:
    """
    Convert the JSON sent by mavic2pro.c into a list of Obstacle3D objects.

    Expected JSON format (sent by C controller):
    {
        "points": [
            {"x": 1.2, "y": 0.5, "z": -0.3},
            ...
        ],
        "drone_pos": {"x": 0.0, "y": 0.0, "z": 1.0},
        "target":    {"x": 5.0, "y": 0.0, "z": 1.5}   // optional
    }
    """
    data = json.loads(raw)
    obstacles = []
    for pt in data.get("points", []):
        # Each LiDAR point becomes a tiny virtual obstacle
        obs = Obstacle3D(pt["x"], pt["y"], pt["z"], charge=300.0, radius=0.2)
        obstacles.append(obs)
    return obstacles, data
'''

def parse_lidar_packet(raw: str) -> list[Obstacle3D]:
    data = json.loads(raw)
    obstacles = []
    max_obstacles = 400
    
    for pt in data.get("points", []):
        x, y, z = pt["x"], pt["y"], pt["z"]
        
        # Filter 1: skip clearly invalid points
        if abs(x) > 50 or abs(y) > 50 or abs(z) > 50:
            continue

        # Filter 1b: skip non-finite values
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        
        # Filter 2: skip ground returns
        # In local LiDAR frame, ground points have large negative z
        if z < -0.2:
            continue

        # Filter 2b: ignore very high/low outliers
        if abs(z) > 2.5:
            continue
            
        # Filter 3: skip points that are too close (self-detection)
        dist = (x**2 + y**2 + z**2) ** 0.5
        if dist < 1.0:
            continue

        # Filter 3b: mask drone body neighborhood
        if abs(x) < 0.45 and abs(y) < 0.45:
            continue
            
        # Filter 4: skip points beyond useful range
        if dist > 6.0:
            continue
            
        obstacles.append(Obstacle3D(x, y, z, charge=300.0, radius=0.2))

        # Cap obstacle count to reduce frame-to-frame command jitter
        if len(obstacles) >= max_obstacles:
            break
    
    return obstacles, data

def build_movement_packet(robot: "Robot3D") -> str:
    """
    Serialise the robot's current velocity as a movement command for Webots.

    Returns JSON string:
    { "vx": float, "vy": float, "vz": float, "yaw_rate": float }
    """
    vel = robot.vel          # Vector3D
    cmd = {
        "vx":       round(vel.x, 4),
        "vy":       round(vel.y, 4),
        "vz":       round(vel.z, 4),
        "yaw_rate": round(robot.yaw_rate, 4),
    }
    return json.dumps(cmd)


def run_server():
    config  = RobotConfig()
    robot   = Robot3D(0.0, 0.0, 1.0, config=config)   # start at (0,0,1 m altitude)

    print(f"[OA Server] Listening on {HOST}:{PORT} – waiting for Webots …")

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((HOST, PORT))
        srv.listen(1)

        conn, addr = srv.accept()
        print(f"[OA Server] Connected: {addr}")

        with conn:
            step = 0
            while True:
                # ── 1. Receive from Webots ────────────────────────────────
                raw = b""
                while not raw.endswith(b"\n"):
                    chunk = conn.recv(BUFFER_SIZE)
                    if not chunk:
                        print("[OA Server] Connection closed by Webots.")
                        return
                    raw += chunk

                try:
                    obstacles, data = parse_lidar_packet(raw.decode())
                except json.JSONDecodeError as e:
                    print(f"[OA Server] Bad JSON on step {step}: {e}")
                    conn.sendall(b'{"vx":0,"vy":0,"vz":0,"yaw_rate":0}\n')
                    continue

                # ── 2. Update robot state ─────────────────────────────────
                robot.obstacles = obstacles

                # Optionally update drone's known position from GPS
                if "drone_pos" in data:
                    dp = data["drone_pos"]
                    robot.physics.position.x = dp["x"]
                    robot.physics.position.y = dp["y"]
                    robot.physics.position.z = dp["z"]

                # Optionally update target waypoint
                if "target" in data:
                    t = data["target"]
                    robot.set_target(t["x"], t["y"], t["z"])

                # Run one OA step (no pygame, no display)
                robot.update()

                # ── Arrival check ─────────────────────────────────────────────────
                if robot.course_completed:
                    print(f"\n{'='*50}")
                    print(f"[OA Server] *** DRONE ARRIVED AT TARGET ***")
                    print(f"[OA Server] Final position: {robot.pos}")
                    print(f"[OA Server] Target was: {robot._target}")
                    print(f"{'='*50}\n")

                    
                # ── 3. Send movement command back ─────────────────────────
                reply = build_movement_packet(robot) + "\n"
                conn.sendall(reply.encode())

                step += 1
                if step % 50 == 0:
                    v = robot.vel
                    print(f"[OA Server] step={step:5d}  "
                          f"vx={v.x:+.2f}  vy={v.y:+.2f}  vz={v.z:+.2f}  "
                          f"obstacles={len(obstacles)}")


if __name__ == "__main__":
    run_server()