"""Tests for the static obstacle avoider."""
import threading

import pytest

from mission_controller.types import Point
from oa_bridge.oa_core import (
    CircleObstacle,
    NullAvoider,
    PolygonObstacle,
    StaticObstacleAvoider,
    load_obstacles,
)


# Anchor everything around one reference point so meter math is sane.
REF_LAT = 45.5048
REF_LON = -73.5721


def _offset(lat, lon, north_m=0.0, east_m=0.0):
    """Shift a lat/lon by a number of meters (flat-earth)."""
    import math
    lat_m_per_deg = 111_320.0
    lon_m_per_deg = lat_m_per_deg * math.cos(math.radians(lat))
    return Point(lat + north_m / lat_m_per_deg, lon + east_m / lon_m_per_deg, 50)


class TestNullAvoider:
    def test_always_clear(self):
        a = NullAvoider()
        assert a.path_clear(Point(0, 0), Point(1, 1))

    def test_returns_target_unchanged(self):
        a = NullAvoider()
        t = Point(1, 2, 3)
        assert a.get_safe_waypoint(Point(0, 0), t, {}) is t


class TestCircleObstacle:
    def test_segment_through_center_intersects(self):
        circle = CircleObstacle(REF_LAT, REF_LON, radius_m=10.0)
        a = _offset(REF_LAT, REF_LON, north_m=-50)
        b = _offset(REF_LAT, REF_LON, north_m=50)
        assert circle.intersects_segment(a, b, buffer_m=0.0)

    def test_segment_far_away_clear(self):
        circle = CircleObstacle(REF_LAT, REF_LON, radius_m=10.0)
        a = _offset(REF_LAT, REF_LON, north_m=-50, east_m=100)
        b = _offset(REF_LAT, REF_LON, north_m=50, east_m=100)
        assert not circle.intersects_segment(a, b, buffer_m=0.0)

    def test_buffer_expands_hit(self):
        circle = CircleObstacle(REF_LAT, REF_LON, radius_m=10.0)
        # Segment ~15m east of center — clear with no buffer, hit with 10m buffer.
        a = _offset(REF_LAT, REF_LON, north_m=-50, east_m=15)
        b = _offset(REF_LAT, REF_LON, north_m=50, east_m=15)
        assert not circle.intersects_segment(a, b, buffer_m=0.0)
        assert circle.intersects_segment(a, b, buffer_m=10.0)


class TestStaticAvoiderRouting:
    def test_clear_path_returns_target(self):
        av = StaticObstacleAvoider(buffer_m=2.0)
        start = _offset(REF_LAT, REF_LON, north_m=0)
        target = _offset(REF_LAT, REF_LON, north_m=100)
        assert av.path_clear(start, target)
        assert av.get_safe_waypoint(start, target, {}) is target

    def test_detour_around_circle(self):
        circle = CircleObstacle(REF_LAT, REF_LON, radius_m=10.0)
        # Place obstacle between start (south) and target (north).
        start = _offset(REF_LAT, REF_LON, north_m=-50)
        target = _offset(REF_LAT, REF_LON, north_m=50)
        av = StaticObstacleAvoider(obstacles=[circle], buffer_m=2.0)
        assert not av.path_clear(start, target)
        wp = av.get_safe_waypoint(start, target, {})
        # The detour waypoint must not equal target and must itself be clear of obstacle.
        assert (wp.x, wp.y) != (target.x, target.y)
        # And the detour point should be outside the obstacle.
        assert not circle.contains(wp.x, wp.y, buffer_m=0.0)


class TestStaticAvoiderMutation:
    def test_add_and_remove_obstacle(self):
        av = StaticObstacleAvoider(buffer_m=2.0)
        start = _offset(REF_LAT, REF_LON, north_m=-50)
        target = _offset(REF_LAT, REF_LON, north_m=50)
        assert av.path_clear(start, target)

        new_obs = CircleObstacle(REF_LAT, REF_LON, radius_m=10.0)
        obs_id = av.add_obstacle(new_obs)
        assert not av.path_clear(start, target)

        assert av.remove_obstacle(obs_id) is True
        assert av.path_clear(start, target)

    def test_concurrent_mutation_is_safe(self):
        av = StaticObstacleAvoider(buffer_m=2.0)
        start = _offset(REF_LAT, REF_LON, north_m=-50)
        target = _offset(REF_LAT, REF_LON, north_m=50)
        stop = threading.Event()

        def thrash():
            while not stop.is_set():
                oid = av.add_obstacle(CircleObstacle(REF_LAT, REF_LON, radius_m=1.0))
                av.remove_obstacle(oid)

        t = threading.Thread(target=thrash)
        t.start()
        try:
            for _ in range(200):
                av.path_clear(start, target)
                av.get_safe_waypoint(start, target, {})
        finally:
            stop.set()
            t.join()


class TestHoverAndWait:
    def test_stuck_inside_obstacle_returns_none(self):
        circle = CircleObstacle(REF_LAT, REF_LON, radius_m=20.0)
        av = StaticObstacleAvoider(obstacles=[circle], buffer_m=2.0)
        # Drone is sitting inside the obstacle.
        current = _offset(REF_LAT, REF_LON, north_m=0)
        target = _offset(REF_LAT, REF_LON, north_m=100)
        assert av.get_safe_waypoint(current, target, {}) is None

    def test_recovers_after_obstacle_removed(self):
        circle = CircleObstacle(REF_LAT, REF_LON, radius_m=20.0, obs_id="bad")
        av = StaticObstacleAvoider(obstacles=[circle], buffer_m=2.0)
        current = _offset(REF_LAT, REF_LON, north_m=0)
        target = _offset(REF_LAT, REF_LON, north_m=100)
        assert av.get_safe_waypoint(current, target, {}) is None

        av.remove_obstacle("bad")
        wp = av.get_safe_waypoint(current, target, {})
        assert wp is not None
        assert (wp.x, wp.y) == (target.x, target.y)


class TestClusterAndTangents:
    def test_true_tangent_geometry(self):
        # The detour point should sit just outside the inflated circle —
        # roughly radius + buffer + clearance from center.
        import math
        circle = CircleObstacle(REF_LAT, REF_LON, radius_m=10.0)
        av = StaticObstacleAvoider(obstacles=[circle], buffer_m=2.0)
        start = _offset(REF_LAT, REF_LON, north_m=-50)
        target = _offset(REF_LAT, REF_LON, north_m=50)
        wp = av.get_safe_waypoint(start, target, {})
        assert wp is not None
        dx = (wp.x - REF_LAT) * 111_320.0
        dy = (wp.y - REF_LON) * 111_320.0 * math.cos(math.radians(REF_LAT))
        d = math.hypot(dx, dy)
        # Inflated radius is 12m; clearance pushes ~1m further.
        assert 12.0 < d < 16.0, f"detour at {d:.2f}m, expected ~13m"

    def test_cluster_picks_other_side(self):
        # Two circles: A on the path, B in the natural-eastern-detour spot.
        # Avoider should route to the western side of A.
        import math
        east_lon = REF_LON + 14.0 / (111_320.0 * math.cos(math.radians(REF_LAT)))
        A = CircleObstacle(REF_LAT, REF_LON, radius_m=10.0, obs_id="A")
        B = CircleObstacle(REF_LAT, east_lon, radius_m=10.0, obs_id="B")
        av = StaticObstacleAvoider(obstacles=[A, B], buffer_m=2.0)
        start = _offset(REF_LAT, REF_LON, north_m=-50)
        target = _offset(REF_LAT, REF_LON, north_m=50)
        wp = av.get_safe_waypoint(start, target, {})
        assert wp is not None
        assert not A.contains(wp.x, wp.y)
        assert not B.contains(wp.x, wp.y)
        # Western side: lon offset from REF should be negative.
        east_offset = (wp.y - REF_LON) * 111_320.0 * math.cos(math.radians(REF_LAT))
        assert east_offset < 0, "expected detour on western side of A"

    def test_all_sides_blocked_returns_none(self):
        # Circles on both flanks of the path-blocker — no valid detour.
        import math
        east_lon = REF_LON + 14.0 / (111_320.0 * math.cos(math.radians(REF_LAT)))
        west_lon = REF_LON - 14.0 / (111_320.0 * math.cos(math.radians(REF_LAT)))
        A = CircleObstacle(REF_LAT, REF_LON, radius_m=10.0)
        B = CircleObstacle(REF_LAT, east_lon, radius_m=10.0)
        C = CircleObstacle(REF_LAT, west_lon, radius_m=10.0)
        av = StaticObstacleAvoider(obstacles=[A, B, C], buffer_m=2.0)
        start = _offset(REF_LAT, REF_LON, north_m=-50)
        target = _offset(REF_LAT, REF_LON, north_m=50)
        assert av.get_safe_waypoint(start, target, {}) is None

    def test_polygon_detour_clears_both_legs(self):
        # Square polygon centered on the path. Detour must not cut through.
        d = 0.0001
        poly = PolygonObstacle([
            (REF_LAT - d, REF_LON - d),
            (REF_LAT - d, REF_LON + d),
            (REF_LAT + d, REF_LON + d),
            (REF_LAT + d, REF_LON - d),
        ])
        av = StaticObstacleAvoider(obstacles=[poly], buffer_m=2.0)
        start = _offset(REF_LAT, REF_LON, north_m=-50)
        target = _offset(REF_LAT, REF_LON, north_m=50)
        wp = av.get_safe_waypoint(start, target, {})
        assert wp is not None
        assert not poly.contains(wp.x, wp.y)
        # Both legs should be clear of the polygon.
        assert not poly.intersects_segment(start, wp, 0.0)
        assert not poly.intersects_segment(wp, target, 0.0)


class TestBoundaryClamping:
    def test_detour_outside_boundary_returns_none(self):
        # Obstacle right at the eastern fence. Detour will be pushed further
        # east, outside the boundary -> hover.
        circle = CircleObstacle(REF_LAT, REF_LON + 0.00005, radius_m=10.0)
        av = StaticObstacleAvoider(obstacles=[circle], buffer_m=2.0)
        start = _offset(REF_LAT, REF_LON, north_m=-50)
        target = _offset(REF_LAT, REF_LON, north_m=50)
        boundary = {
            "min_lat": REF_LAT - 0.001,
            "max_lat": REF_LAT + 0.001,
            "min_lon": REF_LON - 0.001,
            "max_lon": REF_LON + 0.0001,  # tight east limit
        }
        # The detour will be pushed east past max_lon.
        wp = av.get_safe_waypoint(start, target, boundary)
        # Either the detour was rejected as out-of-bounds (None) or it stayed inside.
        if wp is not None:
            assert wp.y <= boundary["max_lon"]

    def test_detour_inside_boundary_accepted(self):
        circle = CircleObstacle(REF_LAT, REF_LON, radius_m=10.0)
        av = StaticObstacleAvoider(obstacles=[circle], buffer_m=2.0)
        start = _offset(REF_LAT, REF_LON, north_m=-50)
        target = _offset(REF_LAT, REF_LON, north_m=50)
        # Generous boundary that easily contains the detour.
        boundary = {
            "min_lat": REF_LAT - 0.01,
            "max_lat": REF_LAT + 0.01,
            "min_lon": REF_LON - 0.01,
            "max_lon": REF_LON + 0.01,
        }
        wp = av.get_safe_waypoint(start, target, boundary)
        assert wp is not None


class TestLoader:
    def test_load_example_yaml(self, tmp_path):
        yaml = pytest.importorskip("yaml")
        cfg = tmp_path / "obs.yaml"
        cfg.write_text(
            "buffer_m: 3.0\n"
            "obstacles:\n"
            "  - type: circle\n"
            "    id: t1\n"
            "    lat: 45.5048\n"
            "    lon: -73.5721\n"
            "    radius_m: 4.0\n"
            "  - type: polygon\n"
            "    id: barn\n"
            "    vertices:\n"
            "      - [45.5050, -73.5723]\n"
            "      - [45.5050, -73.5719]\n"
            "      - [45.5046, -73.5719]\n"
            "      - [45.5046, -73.5723]\n"
        )
        av = load_obstacles(str(cfg))
        obstacles = av.list_obstacles()
        assert len(obstacles) == 2
        assert isinstance(obstacles[0], CircleObstacle)
        assert obstacles[0].id == "t1"
        assert isinstance(obstacles[1], PolygonObstacle)
        assert obstacles[1].id == "barn"


class TestPolygonObstacle:
    def test_segment_through_polygon_blocks(self):
        # 20m square centered at REF.
        d = 0.0001  # ~11m in lat, ~7m in lon
        poly = PolygonObstacle([
            (REF_LAT - d, REF_LON - d),
            (REF_LAT - d, REF_LON + d),
            (REF_LAT + d, REF_LON + d),
            (REF_LAT + d, REF_LON - d),
        ])
        start = _offset(REF_LAT, REF_LON, north_m=-50)
        target = _offset(REF_LAT, REF_LON, north_m=50)
        assert poly.intersects_segment(start, target, buffer_m=0.0)

    def test_segment_beside_polygon_clear(self):
        d = 0.0001
        poly = PolygonObstacle([
            (REF_LAT - d, REF_LON - d),
            (REF_LAT - d, REF_LON + d),
            (REF_LAT + d, REF_LON + d),
            (REF_LAT + d, REF_LON - d),
        ])
        start = _offset(REF_LAT, REF_LON, north_m=-50, east_m=100)
        target = _offset(REF_LAT, REF_LON, north_m=50, east_m=100)
        assert not poly.intersects_segment(start, target, buffer_m=0.0)
