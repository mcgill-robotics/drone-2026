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
_CLEARANCE_M = 1.0  # extra outward push on tangent points


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


def _inside_boundary(lat, lon, boundary):
    """Mirror of mission_controller.stubs.inside_boundary, in lat/lon.

    Returns True if no boundary is given (callers may pass an empty dict).
    """
    if not isinstance(boundary, dict) or not boundary:
        return True
    return (
        boundary.get("min_lat", -90) <= lat <= boundary.get("max_lat", 90)
        and boundary.get("min_lon", -180) <= lon <= boundary.get("max_lon", 180)
    )


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
        dx, dy = bx - ax, by - ay
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq == 0:
            return math.hypot(ax, ay) <= r
        t = max(0.0, min(1.0, -(ax * dx + ay * dy) / seg_len_sq))
        cx, cy = ax + t * dx, ay + t * dy
        return math.hypot(cx, cy) <= r

    def tangent_candidates(self, current, target, buffer_m=0.0):
        """Return up to two true external tangent-point waypoints.

        Geometry: from external point P (current), the two tangent lines to
        a circle of inflated radius r touch the circle at points T1, T2
        symmetric across the line P→O (O = center). Each tangent point is
        pushed outward by a small clearance so the resulting segment
        P → T grazes — not enters — the inflated obstacle. Returned list is
        sorted so the candidate closer to `target` comes first.
        """
        c_lat, c_lon, _ = _coords(current)
        t_lat, t_lon, alt = _coords(target)
        cx, cy = _to_local_m(c_lat, c_lon, self.lat, self.lon)
        tx, ty = _to_local_m(t_lat, t_lon, self.lat, self.lon)

        r = self.radius_m + buffer_m
        d2 = cx * cx + cy * cy
        if d2 <= r * r:
            # Current is inside inflated circle — no external tangent exists.
            return []

        # Angle from O to current; α is the angle between O→current and O→T.
        base = math.atan2(cy, cx)
        alpha = math.acos(r / math.sqrt(d2))

        out = []
        for sign in (+1, -1):
            theta = base + sign * alpha
            # Tangent point on circle, then pushed outward by clearance.
            r_out = r + _CLEARANCE_M
            wx = r_out * math.cos(theta)
            wy = r_out * math.sin(theta)
            lat, lon = _from_local_m(wx, wy, self.lat, self.lon)
            out.append((wx, wy, lat, lon))

        # Sort by distance from candidate to target (in local frame).
        out.sort(key=lambda w: math.hypot(w[0] - tx, w[1] - ty))
        return [_make_like(target, lat, lon, alt) for _, _, lat, lon in out]

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
        if self._point_in_poly(ax, ay) or self._point_in_poly(bx, by):
            return True
        for i in range(len(self._local)):
            x1, y1 = self._local[i]
            x2, y2 = self._local[(i + 1) % len(self._local)]
            if _segments_close(ax, ay, bx, by, x1, y1, x2, y2, buffer_m):
                return True
        return False

    def tangent_candidates(self, current, target, buffer_m=0.0):
        """Return up to two detour candidates straddling the polygon.

        Strategy: displace the polygon centroid perpendicular to the
        current→target line by (max perpendicular extent of any vertex) +
        buffer + clearance. Two candidates — one each side — guaranteed to
        sit outside the polygon. Vertex-corner detours don't work for
        convex polygons sitting on the path: legs through corners still
        cut through adjacent edges.

        Candidates that aren't visible from current (segment cuts the
        polygon) are filtered out. The list is ordered with the side
        closer to the original path first.
        """
        c_lat, c_lon, _ = _coords(current)
        t_lat, t_lon, alt = _coords(target)
        cx, cy = _to_local_m(c_lat, c_lon, self._ref_lat, self._ref_lon)
        tx, ty = _to_local_m(t_lat, t_lon, self._ref_lat, self._ref_lon)

        dx, dy = tx - cx, ty - cy
        L = math.hypot(dx, dy) or 1.0
        ux, uy = dx / L, dy / L
        # Perpendicular unit vector (rotate +90°).
        px_unit, py_unit = -uy, ux

        # Max perpendicular extent of polygon vertices (centroid is at origin).
        max_perp = max(abs(vx * px_unit + vy * py_unit) for vx, vy in self._local)
        offset = max_perp + buffer_m + _CLEARANCE_M

        # Polygon centroid projected onto current→target line so the
        # detour is alongside the obstacle, not in front or behind.
        proj = (-cx) * ux + (-cy) * uy
        anchor_x = cx + proj * ux
        anchor_y = cy + proj * uy

        out = []
        for sign in (+1, -1):
            wx = anchor_x + sign * offset * px_unit
            wy = anchor_y + sign * offset * py_unit
            lat, lon = _from_local_m(wx, wy, self._ref_lat, self._ref_lon)
            wp = _make_like(target, lat, lon, alt)
            # Reject if either leg cuts the polygon (buffer=0; clearance is
            # already in `offset`).
            if self.intersects_segment(current, wp, 0.0):
                continue
            if self.intersects_segment(wp, target, 0.0):
                continue
            out.append(wp)
        return out

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

    On each call, walks blockers nearest-first; each blocker proposes one
    or more detour candidates; the first candidate that passes validation
    (outside every obstacle, inside the boundary, leg from current → wp
    clears the blocker we're avoiding) wins. The FSM replans on the next
    tick, so a sub-waypoint is enough — no global plan is built.
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
        if any(o.contains(c_lat, c_lon, buffer_m=0.0) for o in obstacles):
            return None

        blockers = [
            o for o in obstacles if o.intersects_segment(current, target, self._buffer)
        ]
        if not blockers:
            return target

        # Walk blockers nearest-first, collecting candidates from each. The
        # first candidate that validates against all obstacles + boundary
        # wins. This handles clusters: if going around blocker A drops us
        # inside blocker B, we try A's other side, then move on to B.
        blockers.sort(key=lambda o: self._dist_from(c_lat, c_lon, o))

        for blocker in blockers:
            for candidate in blocker.tangent_candidates(current, target, self._buffer):
                if self._validate(candidate, blocker, obstacles, boundary, current):
                    return candidate

        return None

    def _dist_from(self, lat, lon, obstacle):
        if isinstance(obstacle, CircleObstacle):
            dx, dy = _to_local_m(lat, lon, obstacle.lat, obstacle.lon)
            return math.hypot(dx, dy)
        cx, cy = _to_local_m(lat, lon, obstacle._ref_lat, obstacle._ref_lon)
        return obstacle._dist_to_edges(cx, cy)

    def _validate(self, candidate, blocker, obstacles, boundary, current):
        c_lat, c_lon, _ = _coords(candidate)
        if not _inside_boundary(c_lat, c_lon, boundary):
            return False
        # Candidate must not sit inside any obstacle (no buffer here — the
        # tangent already includes one).
        if any(o.contains(c_lat, c_lon, buffer_m=0.0) for o in obstacles):
            return False
        # And the leg from current → candidate must not cut through the
        # very obstacle we're trying to avoid (would mean we picked the
        # wrong side). Use buffer=0 — clearance is baked into the candidate,
        # and the inflated-buffer check would falsely reject grazing legs.
        if blocker.intersects_segment(current, candidate, 0.0):
            return False
        return True
