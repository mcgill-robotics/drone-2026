#!/usr/bin/env python3
"""
Mission 1 lap-mode test harness.

Loads a lap JSON file, computes a safe TSP loop through lap waypoints using
pathing/pathfinding.py, then repeatedly flies complete laps until a stop is
requested. Press q then Enter to exit after the current lap completes.
"""

from __future__ import annotations

import argparse
import ast
import json
import os
import select
import sys
import types
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
PATHFINDING_FILE = REPO_ROOT / "pathing" / "pathfinding.py"
DEFAULT_LAP_FILE = SCRIPT_DIR / "lap" / "lap_points.json"
os.environ.setdefault("MPLCONFIGDIR", "/private/tmp/matplotlib")


@dataclass(frozen=True)
class Coordinate:
    lat: float
    lon: float


@dataclass
class LapSummary:
    lap_file: Path
    path_waypoints: int
    laps_completed: int = 0
    coordinates_attempted: int = 0
    stop_requested: bool = False
    transition_attempted: bool = False
    transition_succeeded: bool | None = None
    dry_run: bool = False


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError(f"Mission file must contain a JSON object: {path}")
    return data


def load_pathfinding_classes() -> tuple[type, type]:
    """
    Load Node and Pathfinding without executing pathing/pathfinding.py's demo.

    The current pathfinding module has unguarded plotting/test code at module
    scope. This keeps the lap harness on the real classes while avoiding those
    import-time side effects.
    """
    source = PATHFINDING_FILE.read_text(encoding="utf-8")
    tree = ast.parse(source, filename=str(PATHFINDING_FILE))
    allowed_classes = {"Node", "Edge", "Graph", "Pathfinding"}
    kept: list[ast.stmt] = []

    for stmt in tree.body:
        if isinstance(stmt, ast.Import):
            if any(alias.name == "matplotlib.pyplot" for alias in stmt.names):
                continue
            kept.append(stmt)
        elif isinstance(stmt, ast.ImportFrom):
            kept.append(stmt)
        elif isinstance(stmt, ast.ClassDef) and stmt.name in allowed_classes:
            kept.append(stmt)

    module = types.ModuleType("_mission1_pathfinding")
    module.__file__ = str(PATHFINDING_FILE)
    code = compile(ast.Module(body=kept, type_ignores=[]), str(PATHFINDING_FILE), "exec")
    exec(code, module.__dict__)
    return module.Node, module.Pathfinding


def coordinate_from_json(value: Any, label: str) -> Coordinate:
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be an object with lat/lon")
    try:
        return Coordinate(lat=float(value["lat"]), lon=float(value["lon"]))
    except KeyError as exc:
        raise ValueError(f"{label} is missing key {exc.args[0]!r}") from exc
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} lat/lon must be numeric") from exc


def coordinates_from_json(values: Any, label: str) -> list[Coordinate]:
    if not isinstance(values, list) or not values:
        raise ValueError(f"{label} must be a non-empty list")
    return [coordinate_from_json(value, f"{label}[{i}]") for i, value in enumerate(values)]


def coordinate_to_node(coord: Coordinate, node_class: type) -> Any:
    return node_class(coord.lat, coord.lon)


def node_to_coordinate(node: Any) -> Coordinate:
    return Coordinate(lat=float(node.x), lon=float(node.y))


def load_lap_path(mission: dict[str, Any]) -> list[Coordinate]:
    lap = mission.get("lap", mission)
    if not isinstance(lap, dict):
        raise ValueError("Lap JSON must contain an object")

    boundary = coordinates_from_json(lap.get("boundary"), "lap.boundary")
    waypoints = coordinates_from_json(lap.get("waypoints"), "lap.waypoints")
    clearance = float(lap.get("clearance", 0.0))

    node_class, pathfinding_class = load_pathfinding_classes()
    boundary_nodes = [coordinate_to_node(coord, node_class) for coord in boundary]
    waypoint_nodes = [coordinate_to_node(coord, node_class) for coord in waypoints]

    pathfinder = pathfinding_class(boundary_nodes, waypoint_nodes, clearance=clearance)
    loop_nodes = pathfinder.tsp_loop_path()
    if not loop_nodes:
        raise ValueError("Pathfinding returned an empty lap path")
    return [node_to_coordinate(node) for node in loop_nodes]


def import_movement(dry_run: bool):
    if dry_run:
        return None

    if str(REPO_ROOT) not in sys.path:
        sys.path.insert(0, str(REPO_ROOT))
    if str(SCRIPT_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPT_DIR))

    try:
        import movement
    except ImportError as exc:
        raise ImportError(
            "Could not import Mission 1 movement module. Run with --dry-run "
            "or keep tests/mission1/movement.py available."
        ) from exc

    return movement


def poll_stdin_commands() -> set[str]:
    commands: set[str] = set()
    try:
        readable, _, _ = select.select([sys.stdin], [], [], 0)
    except (OSError, ValueError):
        return commands

    for stream in readable:
        line = stream.readline()
        if not line:
            continue
        commands.update(part.strip().lower() for part in line.split() if part.strip())
    return commands


def resolve_transition(args: argparse.Namespace, mission: dict[str, Any]) -> Coordinate | None:
    cli_lat = args.transition_lat
    cli_lon = args.transition_lon
    if (cli_lat is None) != (cli_lon is None):
        raise ValueError("--transition-lat and --transition-lon must be provided together")
    if cli_lat is not None and cli_lon is not None:
        return Coordinate(lat=float(cli_lat), lon=float(cli_lon))
    transition_value = mission.get("transition") or mission.get("after_lap")
    if transition_value is None:
        return None
    return coordinate_from_json(transition_value, "transition")


def navigate(
    navigator: Callable[[float, float], bool] | None,
    coord: Coordinate,
    *,
    dry_run: bool,
    label: str,
) -> bool:
    print(f"[MISSION1 LAP] {label}: lat={coord.lat:.7f}, lon={coord.lon:.7f}")
    if dry_run:
        return True
    if navigator is None:
        raise RuntimeError("Navigator is not configured")
    return bool(navigator(coord.lat, coord.lon))


def run_laps(
    lap_path: list[Coordinate],
    navigator: Callable[[float, float], bool],
    *,
    dry_run: bool,
    max_laps: int | None,
) -> LapSummary:
    summary = LapSummary(
        lap_file=Path(),
        path_waypoints=len(lap_path),
        dry_run=dry_run,
    )

    effective_max_laps = max_laps
    if dry_run and effective_max_laps is None:
        effective_max_laps = 1
        print("[MISSION1 LAP] Dry-run defaulting to one lap. Use --max-laps to change this.")

    while effective_max_laps is None or summary.laps_completed < effective_max_laps:
        lap_number = summary.laps_completed + 1
        print(f"[MISSION1 LAP] Starting lap {lap_number}")

        for i, coord in enumerate(lap_path, start=1):
            ok = navigate(
                navigator,
                coord,
                dry_run=dry_run,
                label=f"Lap {lap_number} waypoint {i}/{len(lap_path)}",
            )
            summary.coordinates_attempted += 1
            if not ok:
                raise RuntimeError(f"Navigation failed at lap {lap_number}, waypoint {i}")

            commands = poll_stdin_commands()
            if "q" in commands:
                summary.stop_requested = True
                print("[MISSION1 LAP] Stop requested. Completing current lap before transition.")

        summary.laps_completed += 1
        print(f"[MISSION1 LAP] Completed lap {lap_number}")

        if summary.stop_requested:
            break

        commands = poll_stdin_commands()
        if "q" in commands:
            summary.stop_requested = True
            print("[MISSION1 LAP] Stop requested between laps.")
            break

    return summary


def print_plan(lap_path: list[Coordinate], transition: Coordinate | None) -> None:
    print(f"[MISSION1 LAP] Computed loop with {len(lap_path)} coordinate visits:")
    for i, coord in enumerate(lap_path, start=1):
        print(f"  {i:02d}. lat={coord.lat:.7f}, lon={coord.lon:.7f}")
    if transition:
        print(
            "[MISSION1 LAP] Transition configured: "
            f"lat={transition.lat:.7f}, lon={transition.lon:.7f}"
        )
    else:
        print("[MISSION1 LAP] No transition coordinate configured.")


def print_summary(summary: LapSummary) -> None:
    transition = "not configured"
    if summary.transition_attempted:
        transition = "succeeded" if summary.transition_succeeded else "failed"

    print("\n[MISSION1 LAP] Summary")
    print(f"  lap_file: {summary.lap_file}")
    print(f"  dry_run: {summary.dry_run}")
    print(f"  path_waypoints: {summary.path_waypoints}")
    print(f"  laps_completed: {summary.laps_completed}")
    print(f"  coordinates_attempted: {summary.coordinates_attempted}")
    print(f"  stop_requested: {summary.stop_requested}")
    print(f"  transition: {transition}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Mission 1 lap-mode flight harness")
    parser.add_argument(
        "--lap-file",
        type=Path,
        default=DEFAULT_LAP_FILE,
        help=f"Lap JSON file. Default: {DEFAULT_LAP_FILE}",
    )
    parser.add_argument(
        "--mission-file",
        type=Path,
        default=None,
        help="Deprecated alias for --lap-file.",
    )
    parser.add_argument("--transition-lat", type=float, default=None)
    parser.add_argument("--transition-lon", type=float, default=None)
    parser.add_argument(
        "--max-laps",
        type=int,
        default=None,
        help="Maximum full laps to fly before exiting lap mode.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Compute and print the lap path without importing or calling movement.",
    )
    parser.add_argument("--takeoff-altitude", type=float, default=5.0)
    return parser.parse_args()


def main() -> bool:
    args = parse_args()
    lap_file_arg = args.lap_file if args.mission_file is None else args.mission_file
    lap_file = lap_file_arg.expanduser().resolve()

    if args.max_laps is not None and args.max_laps < 1:
        raise ValueError("--max-laps must be at least 1 when provided")

    mission = load_json(lap_file)
    lap_path = load_lap_path(mission)
    transition = resolve_transition(args, mission)
    print_plan(lap_path, transition)

    movement = import_movement(args.dry_run)
    navigator = None if movement is None else movement.navigate_to_coordinate
    summary = LapSummary(lap_file=lap_file, path_waypoints=len(lap_path), dry_run=args.dry_run)

    try:
        if movement is not None and not movement.begin_phase(takeoff_altitude=args.takeoff_altitude):
            return False

        summary = run_laps(
            lap_path,
            navigator,
            dry_run=args.dry_run,
            max_laps=args.max_laps,
        )
        summary.lap_file = lap_file

        if transition is not None:
            summary.transition_attempted = True
            summary.transition_succeeded = navigate(
                navigator,
                transition,
                dry_run=args.dry_run,
                label="Transition",
            )
            if not summary.transition_succeeded:
                print_summary(summary)
                return False

        if movement is not None and not movement.end_phase(land=True):
            return False
    finally:
        if movement is not None:
            movement.cleanup()

    print_summary(summary)
    return True


if __name__ == "__main__":
    try:
        success = main()
    except KeyboardInterrupt:
        print("\n[MISSION1 LAP] Interrupted. Active movement calls are not interrupted by q.")
        success = False
    except Exception as exc:
        print(f"[MISSION1 LAP] Error: {exc}")
        success = False
    sys.exit(0 if success else 1)
