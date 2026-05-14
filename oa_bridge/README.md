# oa_bridge

Obstacle avoidance for the mission controller. Currently provides static
(map-based) no-fly-zone avoidance for Mission One; designed so a sensor-backed
avoider (Mission Two, lidar) can drop in behind the same interface.

## Design

`MissionController.safe_goto` is the single chokepoint for every navigation
command. It asks an `Avoider` for the next waypoint each FSM tick (~10 Hz).
Because the FSM re-enters `safe_goto` until the drone is `at_position(target)`,
returning a sub-waypoint gives reactive replanning for free — no separate
replan trigger needed.

```
                ┌─────────────────────────────────┐
  every tick →  │ safe_goto(target, boundary)     │
                │   └─ avoider.get_safe_waypoint  │
                │        ├─ target  (clear)       │
                │        ├─ detour  (blocked)     │
                │        └─ None    (stuck)       │
                └─────────────────────────────────┘
```

## Avoider interface

`oa_bridge/oa_core/avoider.py`

```python
class Avoider:
    def path_clear(self, current, target) -> bool: ...
    def get_safe_waypoint(self, current, target, boundary):
        # Returns target | detour-Point | None (hover-and-wait)
        ...
```

Implementations shipped:

| Class                   | Use                                                |
| ----------------------- | -------------------------------------------------- |
| `NullAvoider`           | Default. Always clear, target unchanged.           |
| `StaticObstacleAvoider` | Map-based. Circles + polygons. Thread-safe mutate. |

## Static obstacles

Two shapes, both in WGS84 lat/lon. Radii and buffers are in **meters** —
geometry is done in a local flat-earth frame around each obstacle, which is
accurate to within centimeters at field-mission scales.

```python
from oa_bridge.oa_core import CircleObstacle, PolygonObstacle

CircleObstacle(lat=45.5048, lon=-73.5721, radius_m=4.0, obs_id="tree-north")
PolygonObstacle(
    vertices=[(lat1, lon1), (lat2, lon2), (lat3, lon3), ...],
    obs_id="barn",
)
```

Every obstacle has an `id`; if you don't pass one, a UUID is generated. The
id is what `remove_obstacle` takes.

## Quick start

```python
from oa_bridge.oa_core import (
    StaticObstacleAvoider, CircleObstacle, PolygonObstacle, load_obstacles
)
from mission_controller.controller import MissionController

# Option A: build in code
avoider = StaticObstacleAvoider(
    obstacles=[
        CircleObstacle(45.50480, -73.57210, radius_m=4.0, obs_id="tree-north"),
        CircleObstacle(45.50420, -73.57180, radius_m=3.5, obs_id="tree-south"),
        PolygonObstacle([
            (45.50455, -73.57250),
            (45.50455, -73.57230),
            (45.50445, -73.57230),
            (45.50445, -73.57250),
        ], obs_id="barn"),
    ],
    buffer_m=5.0,  # extra clearance on top of each obstacle
)

# Option B: load from YAML
avoider = load_obstacles("oa_bridge/config/field_obstacles.yaml")

mc = MissionController(
    mission_number=1,
    site_gps=...,
    mission_boundary=...,
    home_position=...,
    avoider=avoider,   # omit for NullAvoider (no OA)
)
mc.run()
```

## YAML config

`oa_bridge/config/field_obstacles.example.yaml`

```yaml
buffer_m: 5.0

obstacles:
  - type: circle
    id: tree-north
    lat: 45.50480
    lon: -73.57210
    radius_m: 4.0

  - type: polygon
    id: barn
    vertices:
      - [45.50455, -73.57250]
      - [45.50455, -73.57230]
      - [45.50445, -73.57230]
      - [45.50445, -73.57250]
```

Requires PyYAML (`pip install pyyaml`). Already present on ROS images.

## Mid-flight mutation

`StaticObstacleAvoider.add_obstacle` and `remove_obstacle` are thread-safe.
Any thread (e.g. a ground-station listener) can mutate the obstacle list while
the FSM is running — the next `safe_goto` tick automatically replans.

```python
oid = avoider.add_obstacle(CircleObstacle(lat, lon, radius_m=3.0))
# ... later ...
avoider.remove_obstacle(oid)
```

A ground-station → drone command channel is **not yet implemented**; you'd
wrap these calls in whatever transport you use (ROS topic, MAVLink, socket).

## Hover-and-wait

If `get_safe_waypoint` returns `None`, the controller calls `hover()` instead
of `goto_drone` — it switches `current_mode = Mode.HOVER` and asks PX4 to hold
position. `None` is returned when:

- the drone's current position is inside an obstacle, or
- the only detour candidate is itself inside another obstacle.

Replanning happens automatically on the next FSM tick, so once an obstacle is
removed (or moves) the drone resumes toward the target.

## Detour algorithm

For each call to `get_safe_waypoint`:

1. If `current` is inside any obstacle → `None` (hover).
2. Filter obstacles whose interior (plus `buffer_m`) intersects the segment
   `current → target`.
3. If none → return `target` unchanged.
4. Pick the blocker nearest to `current`. Compute a tangent point:
   - **Circle**: the foot of perpendicular from the obstacle center to the
     line, pushed outward by `radius + buffer + 2m`.
   - **Polygon**: the vertex with greatest perpendicular distance from the
     `current → target` line, pushed outward from the polygon centroid.
5. If the candidate is itself inside an obstacle → `None` (hover).
6. Otherwise return the candidate.

This is intentionally simple, not optimal. It handles single isolated
obstacles well; clusters of overlapping obstacles can produce sub-optimal
detours, but the per-tick replanning keeps the drone making progress.

## Files

```
oa_bridge/
├── README.md                 # this file
├── config/
│   └── field_obstacles.example.yaml
└── oa_core/
    ├── avoider.py            # Avoider, NullAvoider
    ├── static_avoider.py     # StaticObstacleAvoider + Circle/Polygon
    └── loader.py             # load_obstacles(path)
```

## Tests

`tests/test_avoider.py` covers:

- null avoider passthrough
- circle/polygon segment intersection (hit, miss, buffer expansion)
- detour validity (waypoint is outside the blocking obstacle)
- thread-safe add/remove under concurrent mutation
- hover-and-wait when stuck inside an obstacle
- recovery after the offending obstacle is removed
- YAML loader (skipped if PyYAML not installed)

Run with `pytest tests/test_avoider.py -v`.

## Not yet built

- **Ground-station command channel** — transport for `add_obstacle` /
  `remove_obstacle` from outside the process. Decision pending on
  ROS topic vs MAVLink vs socket.
- **Sensor-backed avoider** — Mission Two's lidar will plug in here as
  e.g. `LidarAvoider(Avoider)` with no controller changes.
- **Obstacle composition** — once both static and sensor avoiders exist,
  a `CompositeAvoider` that blocks if either says blocked.
```

