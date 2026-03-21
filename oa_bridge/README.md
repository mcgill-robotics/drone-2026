# OA ↔ Webots Bridge

Connects the Python obstacle avoidance (OA) script to the Webots Mavic 2 Pro
simulation over a local TCP socket.

---

## Folder structure

```
oa_bridge/
├── oa_server.py            ← entry point: run this first
├── mavic2pro.c             ← drop into your Webots controller folder
│
├── oa_core/
│   ├── __init__.py
│   ├── vector3d.py         ← replaces pygame.Vector2
│   ├── obstacle3d.py       ← replaces obstacle.py
│   ├── physics3d.py        ← replaces physics_engine.py
│   └── robot3d.py          ← replaces robot.py (3-D OA logic, no pygame)
│
└── config/                 ← copy your existing config/ folder here unchanged
    ├── robot_config.py
    ├── physics_config.py
    ├── navigation_config.py
    ├── environment_config.py
    └── avoidance_config.py
```

---

## How to run

### 1. Copy config files

Copy your existing `config/` folder into `oa_bridge/config/`.
No changes needed – the 3-D code reuses the same dataclasses.

### 2. Start the Python server

```bash
cd oa_bridge
python oa_server.py
```

You should see:

```
[OA Server] Listening on 127.0.0.1:65432 – waiting for Webots …
```

### 3. Start Webots

Open your world, paste `mavic2pro.c` into the controller, rebuild, and hit Play.
You should see in the Webots console:

```
[C] Connected to Python OA server at 127.0.0.1:65432
[C] Mavic 2 Pro: Obstacle Avoidance Bridge Active
```

And in the Python terminal:

```
[OA Server] Connected: ('127.0.0.1', …)
```

---

## Communication protocol

Each timestep the C controller sends one JSON line:

```json
{"points":[{"x":0.12,"y":-0.5,"z":0.03}, ...],
 "drone_pos":{"x":1.0,"y":0.0,"z":1.5},
 "target":{"x":5.0,"y":0.0,"z":1.5}}
```

Python processes it and replies with one JSON line:

```json
{ "vx": -0.12, "vy": 0.05, "vz": 0.0, "yaw_rate": 0.03 }
```

---

## Tuning knobs

| Location                     | Variable                 | What it does                               |
| ---------------------------- | ------------------------ | ------------------------------------------ |
| `mavic2pro.c`                | `oa_pitch = -vx * 0.5`   | How aggressively forward motion is applied |
| `mavic2pro.c`                | `oa_roll  =  vy * 0.5`   | How aggressively lateral motion is applied |
| `mavic2pro.c`                | `TARGET_X/Y/Z`           | Where the drone tries to fly to            |
| `oa_server.py`               | `PORT`                   | TCP port (must match in both files)        |
| `config/physics_config.py`   | `max_speed`, `max_force` | OA speed limits (now in m/s not pixels)    |
| `config/avoidance_config.py` | `scan_radius` entries    | How far ahead obstacles are detected       |

The scan radii in `avoidance_config.py` were originally in pixel units (~38–98).
`robot3d.py` automatically divides them by 20 if they are > 10, converting to
metres. If avoidance feels too tight or too loose, adjust the `/= 20.0` divisor
in `robot3d._scan_for_obstacles`.

---

## Known limitations (expected for this phase)

- The target is a single hardcoded point in `mavic2pro.c`. Extend `TARGET_X/Y/Z`
  to an array and advance the index when the drone is within arrival radius.
- The yaw controller is proportional only; a full PID will feel smoother.
- LiDAR points are treated as individual obstacles; clustering them (e.g. voxel
  grid) before sending will reduce socket payload and improve OA stability.
