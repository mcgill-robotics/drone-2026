"""Static no-fly-zone avoider for Mission One.

Obstacles are known up front (trees, barns) and may be added or removed
mid-flight via add_obstacle / remove_obstacle (thread-safe). For small
mission areas, lat/lon are converted to local meters with a flat-earth
approximation around the obstacle so geometry can be done in meters.
"""

import math
import threading
import uuid

from mission_controller.types import Point

from .avoider import Avoider


_LAT_M_PER_DEG = 111_320.0


def _lon_m_per_deg(lat_deg):
    return _LAT_M_PER_DEG * math.cos(math.radians(lat_deg))


def _coords(p):
    """Extract (lat, lon, alt) from Point or dict."""
    if hasattr(p, "x") and hasattr(p, "y"):
        return p.x, p.y, getattr(p, "z", 0)
    if isinstance(p, dict):
        return p.get("lat", 0), p.get("lon", 0), p.get("alt", 0)
    raise TypeError(f"Unsupported coordinate type: {type(p)}")


def _make_like(template, lat, lon, alt):
    """Return a value shaped like `template` with new coordinates."""
    if isinstance(template, dict):
        out = dict(template)
        out["lat"] = lat
        out["lon"] = lon
        if "alt" in template or alt:
            out["alt"] = alt
        return out
    return Point(lat, lon, alt)


def _to_local_m(lat, lon, ref_lat, ref_lon):
    """Convert (lat, lon) to local meters east/north of a reference."""
    dx = (lat - ref_lat) * _LAT_M_PER_DEG
    dy = (lon - ref_lon) * _lon_m_per_deg(ref_lat)
    return dx, dy


def _from_local_m(dx, dy, ref_lat, ref_lon):
    lat = ref_lat + dx / _LAT_M_PER_DEG
    lon = ref_lon + dy / _lon_m_per_deg(ref_lat)
    return lat, lon


class CircleObstacle:
    """Circular no-fly zone centered at (lat, lon) with radius in meters."""

    def __init__(self, lat, lon, radius_m, obs_id=None):
        self.lat = lat
        self.lon = lon
        self.radius_m = radius_m
        self.id = obs_id or str(uuid.uuid4())

    def contains(self, lat, lon, buffer_m=0.0):
        dx, dy = _to_local_m(lat, lon, self.lat, self.lon)
        return math.hypot(dx, dy) <= self.radius_m + buffer_m

    def intersects_segment(self, a, b, buffer_m=0.0):
        """True if the segment a→b passes within radius+buffer of center."""
        a_lat, a_lon, _ = _coords(a)
        b_lat, b_lon, _ = _coords(b)
        ax, ay = _to_local_m(a_lat, a_lon, self.lat, self.lon)
        bx, by = _to_local_m(b_lat, b_lon, self.lat, self.lon)
        r = self.radius_m + buffer_m

        # Distance from origin (the obstacle center, in local frame) to segment ab.
        dx, dy = bx - ax, by - ay
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq == 0:
            return math.hypot(ax, ay) <= r
        t = max(0.0, min(1.0, -(ax * dx + ay * dy) / seg_len_sq))
        cx, cy = ax + t * dx, ay + t * dy
        return math.hypot(cx, cy) <= r

    def tangent_waypoint(self, current, target, buffer_m=0.0):
        """Return a detour point that goes around this circle.

        Picks the side of the obstacle closer to a straight line from
        current → target. Result is offset by radius + buffer + small
        clearance, in lat/lon shaped like `target`.
        """
        c_lat, c_lon, _ = _coords(current)
        t_lat, t_lon, alt = _coords(target)

        # Work in local meters around the obstacle center.
        cx, cy = _to_local_m(c_lat, c_lon, self.lat, self.lon)
        tx, ty = _to_local_m(t_lat, t_lon, self.lat, self.lon)

        # Unit vector along current→target.
        dx, dy = tx - cx, ty - cy
        L = math.hypot(dx, dy) or 1.0
        ux, uy = dx / L, dy / L
        # Perpendicular (two options); pick the one farther from origin
        # projection so we go around, not through.
        # Projection of -current onto u gives closest approach along the line.
        proj = -(cx * ux + cy * uy)
        closest_x = cx + proj * ux
        closest_y = cy + proj * uy
        side = math.hypot(closest_x, closest_y)
        # Detour distance: push to the side by radius + buffer + 2m clearance.
        push = self.radius_m + buffer_m + 2.0
        if side < 1e-6:
            # Line goes straight through center; pick an arbitrary perpendicular.
            px, py = -uy, ux
        else:
            px, py = closest_x / side, closest_y / side
        wx = closest_x + px * push
        wy = closest_y + py * push
        lat, lon = _from_local_m(wx, wy, self.lat, self.lon)
        return _make_like(target, lat, lon, alt)

    def to_dict(self):
        return {
            "type": "circle",
            "id": self.id,
            "lat": self.lat,
            "lon": self.lon,
            "radius_m": self.radius_m,
        }


class PolygonObstacle:
    """Polygonal no-fly zone defined by an ordered list of (lat, lon) vertices."""

    def __init__(self, vertices, obs_id=None):
        if len(vertices) < 3:
            raise ValueError("Polygon needs at least 3 vertices")
        self.vertices = list(vertices)
        self.id = obs_id or str(uuid.uuid4())
        # Reference point for local-meter conversion = centroid.
        self._ref_lat = sum(v[0] for v in vertices) / len(vertices)
        self._ref_lon = sum(v[1] for v in vertices) / len(vertices)
        self._local = [
            _to_local_m(lat, lon, self._ref_lat, self._ref_lon)
            for lat, lon in vertices
        ]

    def contains(self, lat, lon, buffer_m=0.0):
        x, y = _to_local_m(lat, lon, self._ref_lat, self._ref_lon)
        return self._point_in_poly(x, y) or (
            buffer_m > 0 and self._dist_to_edges(x, y) <= buffer_m
        )

    def intersects_segment(self, a, b, buffer_m=0.0):
        a_lat, a_lon, _ = _coords(a)
        b_lat, b_lon, _ = _coords(b)
        ax, ay = _to_local_m(a_lat, a_lon, self._ref_lat, self._ref_lon)
        bx, by = _to_local_m(b_lat, b_lon, self._ref_lat, self._ref_lon)
        # If either endpoint is inside (with buffer), it intersects.
        if self._point_in_poly(ax, ay) or self._point_in_poly(bx, by):
            return True
        # Otherwise check segment vs each edge, with buffer.
        for i in range(len(self._local)):
            x1, y1 = self._local[i]
            x2, y2 = self._local[(i + 1) % len(self._local)]
            if _segments_close(ax, ay, bx, by, x1, y1, x2, y2, buffer_m):
                return True
        return False

    def tangent_waypoint(self, current, target, buffer_m=0.0):
        """Detour: head toward the polygon vertex farthest off the direct line."""
        c_lat, c_lon, _ = _coords(current)
        t_lat, t_lon, alt = _coords(target)
        cx, cy = _to_local_m(c_lat, c_lon, self._ref_lat, self._ref_lon)
        tx, ty = _to_local_m(t_lat, t_lon, self._ref_lat, self._ref_lon)
        dx, dy = tx - cx, ty - cy
        L = math.hypot(dx, dy) or 1.0
        # Pick vertex with greatest perpendicular distance from the c→t line.
        best = None
        best_d = -1.0
        for vx, vy in self._local:
            perp = abs((vx - cx) * dy - (vy - cy) * dx) / L
            if perp > best_d:
                best_d = perp
                best = (vx, vy)
        # Push slightly outward from polygon centroid (origin in local frame).
        vx, vy = best
        norm = math.hypot(vx, vy) or 1.0
        push = buffer_m + 2.0
        wx = vx + (vx / norm) * push
        wy = vy + (vy / norm) * push
        lat, lon = _from_local_m(wx, wy, self._ref_lat, self._ref_lon)
        return _make_like(target, lat, lon, alt)

    def _point_in_poly(self, x, y):
        inside = False
        n = len(self._local)
        j = n - 1
        for i in range(n):
            xi, yi = self._local[i]
            xj, yj = self._local[j]
            if ((yi > y) != (yj > y)) and (
                x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi
            ):
                inside = not inside
            j = i
        return inside

    def _dist_to_edges(self, x, y):
        best = float("inf")
        for i in range(len(self._local)):
            x1, y1 = self._local[i]
            x2, y2 = self._local[(i + 1) % len(self._local)]
            best = min(best, _point_segment_dist(x, y, x1, y1, x2, y2))
        return best

    def to_dict(self):
        return {
            "type": "polygon",
            "id": self.id,
            "vertices": [list(v) for v in self.vertices],
        }


def _point_segment_dist(px, py, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    L2 = dx * dx + dy * dy
    if L2 == 0:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / L2))
    cx, cy = x1 + t * dx, y1 + t * dy
    return math.hypot(px - cx, py - cy)


def _segments_close(ax, ay, bx, by, x1, y1, x2, y2, buffer_m):
    """True if segment ab and segment (x1,y1)-(x2,y2) come within buffer_m."""
    if _segments_intersect(ax, ay, bx, by, x1, y1, x2, y2):
        return True
    if buffer_m <= 0:
        return False
    return min(
        _point_segment_dist(ax, ay, x1, y1, x2, y2),
        _point_segment_dist(bx, by, x1, y1, x2, y2),
        _point_segment_dist(x1, y1, ax, ay, bx, by),
        _point_segment_dist(x2, y2, ax, ay, bx, by),
    ) <= buffer_m


def _segments_intersect(ax, ay, bx, by, cx, cy, dx, dy):
    def ccw(x1, y1, x2, y2, x3, y3):
        return (y3 - y1) * (x2 - x1) > (y2 - y1) * (x3 - x1)
    return (
        ccw(ax, ay, cx, cy, dx, dy) != ccw(bx, by, cx, cy, dx, dy)
        and ccw(ax, ay, bx, by, cx, cy) != ccw(ax, ay, bx, by, dx, dy)
    )


class StaticObstacleAvoider(Avoider):
    """Avoider backed by a list of known static obstacles.

    Obstacles are checked against the straight line from current → target.
    On a hit, returns a tangent/detour waypoint that goes around the
    closest blocking obstacle. The FSM replans on the next tick.
    """

    def __init__(self, obstacles=None, buffer_m=5.0):
        self._obstacles = list(obstacles or [])
        self._buffer = buffer_m
        self._lock = threading.Lock()

    def add_obstacle(self, obs):
        with self._lock:
            self._obstacles.append(obs)
        return obs.id

    def remove_obstacle(self, obs_id):
        with self._lock:
            before = len(self._obstacles)
            self._obstacles = [o for o in self._obstacles if o.id != obs_id]
            return len(self._obstacles) != before

    def list_obstacles(self):
        with self._lock:
            return list(self._obstacles)

    def path_clear(self, current, target):
        with self._lock:
            obstacles = list(self._obstacles)
        return not any(
            o.intersects_segment(current, target, self._buffer) for o in obstacles
        )

    def get_safe_waypoint(self, current, target, boundary):
        with self._lock:
            obstacles = list(self._obstacles)

        c_lat, c_lon, _ = _coords(current)

        # If the drone is currently inside any obstacle, no safe detour exists —
        # hover and wait for the obstacle list to change.
        if any(o.contains(c_lat, c_lon, buffer_m=0.0) for o in obstacles):
            return None

        blockers = [
            o for o in obstacles if o.intersects_segment(current, target, self._buffer)
        ]
        if not blockers:
            return target

        def dist_from_current(o):
            if isinstance(o, CircleObstacle):
                dx, dy = _to_local_m(c_lat, c_lon, o.lat, o.lon)
                return math.hypot(dx, dy)
            cx, cy = _to_local_m(c_lat, c_lon, o._ref_lat, o._ref_lon)
            return o._dist_to_edges(cx, cy)

        nearest = min(blockers, key=dist_from_current)
        candidate = nearest.tangent_waypoint(current, target, self._buffer)

        # If the candidate detour itself sits inside another obstacle, give up
        # and hover — replanning next tick may find an escape.
        c_lat2, c_lon2, _ = _coords(candidate)
        if any(o.contains(c_lat2, c_lon2, buffer_m=0.0) for o in obstacles):
            return None

        return candidate
