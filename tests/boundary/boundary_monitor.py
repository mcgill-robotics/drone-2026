#!/usr/bin/env python3
"""
Standalone AEAC-style boundary safety prototype.

This file is intentionally NOT integrated into the main flight code yet.

Prototype behavior:
- Inside boundary:
    Do nothing.

- Outside soft boundary:
    Print warning.
    Switch aircraft to manual Position mode / POSCTL.

- Outside hard boundary:
    Print critical warning.
    Switch aircraft to Position mode.
    Trigger emergency landing / termination path placeholder.

This is for algorithm testing first. Final integration should be updated
according to the real competition boundary file and actual PX4 kill/termination setup.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


EARTH_RADIUS_M = 6_378_137.0


class BoundaryStatus:
    INSIDE = "inside"

    OUTSIDE_SOFT = "outside_soft_boundary"
    OUTSIDE_HARD = "outside_hard_boundary"

    SOFT_ALTITUDE_VIOLATION = "soft_altitude_violation"
    HARD_ALTITUDE_VIOLATION = "hard_altitude_violation"


def load_boundary_file(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    if "soft_boundary" not in data:
        raise ValueError("Boundary file is missing 'soft_boundary'.")

    if "hard_boundary" not in data:
        raise ValueError("Boundary file is missing 'hard_boundary'.")

    if "altitude" not in data:
        raise ValueError("Boundary file is missing 'altitude'.")

    return data


def polygon_contains_point(point: tuple[float, float], polygon: list[tuple[float, float]]) -> bool:
    """
    Ray-casting point-in-polygon check.

    point:
        (x, y), where x = longitude/east-like value, y = latitude/north-like value

    polygon:
        List of (x, y) points.

    Returns:
        True if point is inside polygon.
        False if point is outside polygon.
    """
    x, y = point
    inside = False

    j = len(polygon) - 1

    for i in range(len(polygon)):
        xi, yi = polygon[i]
        xj, yj = polygon[j]

        crosses_y_level = (yi > y) != (yj > y)

        if crosses_y_level:
            x_intersection = (xj - xi) * (y - yi) / ((yj - yi) + 1e-12) + xi

            if x < x_intersection:
                inside = not inside

        j = i

    return inside


def gps_polygon_to_xy(boundary: list[dict[str, float]]) -> list[tuple[float, float]]:
    """
    For boundary checking, lon is used as x and lat is used as y.

    This is fine for small competition-area geofence checks because we only need
    consistent polygon inclusion logic, not exact distance.
    """
    return [(float(p["lon"]), float(p["lat"])) for p in boundary]


def check_boundary_status(
    current_lat: float,
    current_lon: float,
    current_alt_agl_m: float,
    boundary_config: dict[str, Any],
) -> str:
    """
    Check whether current drone position is inside, outside soft boundary,
    outside hard boundary, or violating altitude limits.

    Priority:
    1. Hard altitude violation
    2. Hard boundary violation
    3. Soft altitude violation
    4. Soft boundary violation
    5. Inside
    """

    altitude = boundary_config["altitude"]

    soft_min_alt = float(altitude["soft_min_m"])
    soft_max_alt = float(altitude["soft_max_m"])
    hard_min_alt = float(altitude["hard_min_m"])
    hard_max_alt = float(altitude["hard_max_m"])

    if current_alt_agl_m < hard_min_alt or current_alt_agl_m > hard_max_alt:
        return BoundaryStatus.HARD_ALTITUDE_VIOLATION

    soft_polygon = gps_polygon_to_xy(boundary_config["soft_boundary"])
    hard_polygon = gps_polygon_to_xy(boundary_config["hard_boundary"])

    current_point = (float(current_lon), float(current_lat))

    inside_hard = polygon_contains_point(current_point, hard_polygon)
    inside_soft = polygon_contains_point(current_point, soft_polygon)

    if not inside_hard:
        return BoundaryStatus.OUTSIDE_HARD

    if current_alt_agl_m < soft_min_alt or current_alt_agl_m > soft_max_alt:
        return BoundaryStatus.SOFT_ALTITUDE_VIOLATION

    if not inside_soft:
        return BoundaryStatus.OUTSIDE_SOFT

    return BoundaryStatus.INSIDE


def handle_boundary_status(status: str, px4: Any | None = None) -> None:
    """
    Standalone prototype handler.

    px4 is optional because this file is meant to be testable without the real drone.
    Later, px4 can be your PX4Interface object.
    """

    if status == BoundaryStatus.INSIDE:
        print("[BOUNDARY] OK: Aircraft is inside allowed boundary.")
        return

    if status in {
        BoundaryStatus.OUTSIDE_SOFT,
        BoundaryStatus.SOFT_ALTITUDE_VIOLATION,
    }:
        print("[BOUNDARY WARNING] Soft boundary violation detected.")
        print("[BOUNDARY WARNING] Switching to manual Position mode / POSCTL.")

        if px4 is not None:
            # TODO: Replace with the exact project function name if different.
            px4.set_mode("POSCTL")

        return

    if status in {
        BoundaryStatus.OUTSIDE_HARD,
        BoundaryStatus.HARD_ALTITUDE_VIOLATION,
    }:
        print("[BOUNDARY CRITICAL] Hard boundary violation detected.")
        print("[BOUNDARY CRITICAL] Leaving autonomous mode immediately.")
        print("[BOUNDARY CRITICAL] Triggering emergency landing / termination prototype.")

        if px4 is not None:
            # Step 1: leave autonomous/offboard mode.
            px4.set_mode("POSCTL")

            # Step 2: prototype emergency response.
            # For the final competition version, replace this with the real
            # flight termination / kill-switch logic required by the team.
            px4.land()

        return

    print(f"[BOUNDARY ERROR] Unknown boundary status: {status}")


def simulate_boundary_check(boundary_file: Path) -> None:
    """
    Small local test without PX4/MAVROS.
    """

    config = load_boundary_file(boundary_file)

    test_points = [
        {
            "name": "inside_example",
            "lat": 45.3180,
            "lon": -75.7580,
            "alt": 50.0,
        },
        {
            "name": "soft_altitude_violation_example",
            "lat": 45.3180,
            "lon": -75.7580,
            "alt": 125.0,
        },
        {
            "name": "hard_altitude_violation_example",
            "lat": 45.3180,
            "lon": -75.7580,
            "alt": 150.0,
        },
        {
            "name": "outside_boundary_example",
            "lat": 45.3300,
            "lon": -75.7700,
            "alt": 50.0,
        },
    ]

    for point in test_points:
        status = check_boundary_status(
            current_lat=point["lat"],
            current_lon=point["lon"],
            current_alt_agl_m=point["alt"],
            boundary_config=config,
        )

        print()
        print(f"Test point: {point['name']}")
        print(f"lat={point['lat']}, lon={point['lon']}, alt={point['alt']} m AGL")
        print(f"status={status}")

        handle_boundary_status(status, px4=None)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Standalone boundary monitor prototype.")
    parser.add_argument(
        "--boundary-file",
        type=Path,
        default=Path(__file__).resolve().parent / "mock_boundaries.json",
    )
    parser.add_argument("--lat", type=float, default=None)
    parser.add_argument("--lon", type=float, default=None)
    parser.add_argument("--alt", type=float, default=None)
    parser.add_argument(
        "--simulate",
        action="store_true",
        help="Run several built-in mock test points.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.simulate:
        simulate_boundary_check(args.boundary_file)
        return 0

    if args.lat is None or args.lon is None or args.alt is None:
        print("Please provide --lat, --lon, and --alt, or use --simulate.")
        return 2

    config = load_boundary_file(args.boundary_file)

    status = check_boundary_status(
        current_lat=args.lat,
        current_lon=args.lon,
        current_alt_agl_m=args.alt,
        boundary_config=config,
    )

    print(f"[BOUNDARY] status={status}")
    handle_boundary_status(status, px4=None)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
