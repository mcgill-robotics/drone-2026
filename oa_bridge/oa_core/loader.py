"""Load static obstacles from a YAML file into a StaticObstacleAvoider.

Expected schema (see config/field_obstacles.example.yaml):

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
          - [lat, lon]
          - ...
"""

from .static_avoider import CircleObstacle, PolygonObstacle, StaticObstacleAvoider


def load_obstacles(path):
    """Parse a YAML file and return a configured StaticObstacleAvoider."""
    import yaml  # lazy: only required when actually loading from disk
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}

    buffer_m = float(data.get("buffer_m", 5.0))
    obstacles = [_parse_obstacle(entry) for entry in data.get("obstacles", [])]
    return StaticObstacleAvoider(obstacles=obstacles, buffer_m=buffer_m)


def _parse_obstacle(entry):
    kind = entry.get("type")
    obs_id = entry.get("id")
    if kind == "circle":
        return CircleObstacle(
            lat=float(entry["lat"]),
            lon=float(entry["lon"]),
            radius_m=float(entry["radius_m"]),
            obs_id=obs_id,
        )
    if kind == "polygon":
        vertices = [(float(lat), float(lon)) for lat, lon in entry["vertices"]]
        return PolygonObstacle(vertices=vertices, obs_id=obs_id)
    raise ValueError(f"Unknown obstacle type: {kind!r}")
