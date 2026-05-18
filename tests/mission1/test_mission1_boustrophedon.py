#!/usr/bin/env python3
"""Mission 1 boustrophedon coverage runner."""

from __future__ import annotations

import argparse
import ast
import json
import math
import os
import queue
import sys
import threading
import types
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_AREA_FILE = SCRIPT_DIR / "boustrophedon" / "area.json"
DEFAULT_STATE_FILE = SCRIPT_DIR / "boustrophedon" / "coverage_state.json"
DEFAULT_SETTINGS_FILE = SCRIPT_DIR / "boustrophedon" / "settings.yaml"
EARTH_RADIUS_M = 6_378_137.0
os.environ.setdefault("MPLCONFIGDIR", "/private/tmp/matplotlib")
BOUSTROPHEDON_FILE = REPO_ROOT / "pathing" / "boustrophedon.py"


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError(f"JSON file must contain an object: {path}")
    return data


def save_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_suffix(path.suffix + ".tmp")
    with tmp_path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
        f.write("\n")
    tmp_path.replace(path)


def load_settings(path: Path) -> dict[str, float]:
    if not path.exists():
        return {}

    settings: dict[str, float] = {}
    with path.open("r", encoding="utf-8") as f:
        for line_number, raw_line in enumerate(f, start=1):
            line = raw_line.split("#", 1)[0].strip()
            if not line:
                continue
            if ":" not in line:
                raise ValueError(f"Invalid settings line {line_number}: {raw_line.rstrip()}")
            key, value = line.split(":", 1)
            key = key.strip()
            value = value.strip()
            if not key:
                raise ValueError(f"Invalid empty settings key on line {line_number}")
            try:
                settings[key] = float(value)
            except ValueError as exc:
                raise ValueError(f"Setting {key!r} must be numeric") from exc
    return settings


def _require_coord(coord: Any, label: str) -> dict[str, float]:
    if not isinstance(coord, dict):
        raise ValueError(f"{label} must be an object with lat/lon keys.")
    try:
        return {"lat": float(coord["lat"]), "lon": float(coord["lon"])}
    except KeyError as exc:
        raise ValueError(f"{label} is missing {exc.args[0]!r}.") from exc


def gps_to_local_m(origin: dict[str, float], coord: dict[str, float]) -> tuple[float, float]:
    origin_lat_rad = math.radians(origin["lat"])
    north = math.radians(coord["lat"] - origin["lat"]) * EARTH_RADIUS_M
    east = math.radians(coord["lon"] - origin["lon"]) * EARTH_RADIUS_M * math.cos(origin_lat_rad)
    return east, north


def local_m_to_gps(origin: dict[str, float], point: Any) -> dict[str, float]:
    if hasattr(point, "x") and hasattr(point, "y"):
        east = float(point.x)
        north = float(point.y)
    elif isinstance(point, (list, tuple)) and len(point) >= 2:
        east = float(point[0])
        north = float(point[1])
    else:
        raise TypeError(f"Unsupported path point type: {type(point)}")

    origin_lat_rad = math.radians(origin["lat"])
    lat = origin["lat"] + math.degrees(north / EARTH_RADIUS_M)
    lon = origin["lon"] + math.degrees(east / (EARTH_RADIUS_M * math.cos(origin_lat_rad)))
    return {"lat": lat, "lon": lon}


def obstacle_to_pathing(obstacle: Any, origin: dict[str, float]) -> Any:
    if isinstance(obstacle, dict):
        return gps_to_local_m(origin, _require_coord(obstacle, "obstacle coordinate"))
    if isinstance(obstacle, (list, tuple)):
        if len(obstacle) == 4 and all(isinstance(v, (int, float)) for v in obstacle):
            return [float(v) for v in obstacle]
        return [obstacle_to_pathing(pt, origin) for pt in obstacle]
    return obstacle


def load_plan_boustrophedon():
    """
    Load plan_boustrophedon without importing the module-level plotting helper.

    pathing/boustrophedon.py imports matplotlib at module scope for its demo
    plotter. The mission runner only needs the planner, so this keeps field
    runs free of plotting/cache side effects.
    """
    source = BOUSTROPHEDON_FILE.read_text(encoding="utf-8")
    tree = ast.parse(source, filename=str(BOUSTROPHEDON_FILE))
    kept: list[ast.stmt] = []

    for stmt in tree.body:
        if isinstance(stmt, ast.Import):
            if any(alias.name == "matplotlib.pyplot" for alias in stmt.names):
                continue
            kept.append(stmt)
        elif isinstance(stmt, ast.ImportFrom):
            kept.append(stmt)
        elif isinstance(stmt, ast.FunctionDef) and stmt.name != "_plot_demo":
            kept.append(stmt)

    module = types.ModuleType("_mission1_boustrophedon")
    module.__file__ = str(BOUSTROPHEDON_FILE)
    code = compile(ast.Module(body=kept, type_ignores=[]), str(BOUSTROPHEDON_FILE), "exec")
    exec(code, module.__dict__)
    return module.plan_boustrophedon


def build_boustrophedon_path(
    area: dict[str, Any],
    settings: dict[str, float],
    *,
    start: dict[str, float] | None,
    goal: dict[str, float] | None,
) -> list[dict[str, float]]:
    plan_boustrophedon = load_plan_boustrophedon()

    boustro = area.get("boustrophedon", area)
    if not isinstance(boustro, dict):
        raise ValueError("Boustrophedon area JSON must contain an object")

    boundary = boustro.get("boundary")
    if not boundary:
        raise ValueError("boundary is required to generate a boustrophedon path.")

    origin = _require_coord(boustro.get("origin") or area.get("origin") or goal or boundary[0], "origin")
    boundary_points = [gps_to_local_m(origin, _require_coord(coord, "boundary coordinate")) for coord in boundary]
    obstacles = [obstacle_to_pathing(ob, origin) for ob in boustro.get("obstacles", [])]

    planner_start = start or boustro.get("start")
    planner_goal = goal or boustro.get("goal") or area.get("return") or area.get("start")
    _, _, path = plan_boustrophedon(
        boundary_points,
        obstacles,
        spacing=float(settings.get("spacing", 1.0)),
        clearance=float(settings.get("clearance", 0.0)),
        resolution=float(settings.get("resolution", 1.0)),
        start=(
            gps_to_local_m(origin, _require_coord(planner_start, "start"))
            if planner_start is not None
            else None
        ),
        goal=(
            gps_to_local_m(origin, _require_coord(planner_goal, "goal"))
            if planner_goal is not None
            else None
        ),
    )

    return [local_m_to_gps(origin, point) for point in path]


def import_movement():
    try:
        import movement
    except ImportError as exc:
        raise RuntimeError(
            "Could not import Mission 1 movement module."
        ) from exc
    return movement


class StdinCommands:
    """Collect line-based stdin commands without blocking waypoint movement."""

    def __init__(self) -> None:
        self._commands: "queue.Queue[str]" = queue.Queue()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def latest_requested_action(self, allowed: set[str] | None = None) -> str | None:
        if allowed is None:
            allowed = {"p", "q"}
        latest: str | None = None
        while True:
            try:
                cmd = self._commands.get_nowait()
            except queue.Empty:
                break
            if cmd in allowed:
                latest = cmd
        return latest

    def wait_for_action(self, allowed: set[str]) -> str:
        while True:
            cmd = self._commands.get()
            if cmd in allowed:
                return cmd

    def _read_loop(self) -> None:
        for line in sys.stdin:
            for cmd in line.strip().lower().split():
                if cmd:
                    self._commands.put(cmd)


def wait_until_resume_or_stop(
    commands: StdinCommands,
    state_file: Path,
    state: dict[str, Any],
    dry_run: bool,
) -> str:
    state["status"] = "paused"
    if not dry_run:
        save_json(state_file, state)

    print("[MISSION1 BOUSTRO] Paused. Type 'r' then Enter to resume, or 'q' then Enter to stop.")
    action = commands.wait_for_action({"r", "q"})
    if action == "q":
        state["status"] = "stopped"
        if not dry_run:
            save_json(state_file, state)
        return "stopped"

    state["status"] = "running"
    if not dry_run:
        save_json(state_file, state)
    print("[MISSION1 BOUSTRO] Resuming.")
    return "running"


def ensure_path_state(
    area: dict[str, Any],
    state_file: Path,
    state: dict[str, Any],
    settings: dict[str, float],
    regenerate: bool,
    dry_run: bool,
    start: dict[str, float] | None,
    goal: dict[str, float] | None,
) -> list[dict[str, float]]:
    path = state.get("path") or []

    if regenerate or not path:
        path = build_boustrophedon_path(area, settings, start=start, goal=goal)
        state["path"] = path
        state["current_index"] = 0
        state["status"] = "ready"
        if not dry_run:
            save_json(state_file, state)

    return [_require_coord(coord, f"path[{i}]") for i, coord in enumerate(path)]


def run_boustrophedon(
    area_file: Path,
    state_file: Path,
    settings_file: Path,
    *,
    regenerate: bool = False,
    dry_run: bool = False,
    start: dict[str, float] | None = None,
    goal: dict[str, float] | None = None,
    takeoff_altitude: float = 5.0,
) -> dict[str, Any]:
    area = load_json(area_file)
    state = load_json(state_file) if state_file.exists() else {}
    settings = load_settings(settings_file)
    path = ensure_path_state(area, state_file, state, settings, regenerate, dry_run, start, goal)

    current_index = int(state.get("current_index", 0))
    if current_index < 0:
        current_index = 0
    if current_index > len(path):
        current_index = len(path)
    state["current_index"] = current_index

    if current_index >= len(path):
        state["status"] = "complete"
        if not dry_run:
            save_json(state_file, state)
        return {"status": "complete", "arrived": 0, "current_index": current_index, "total": len(path)}

    movement = None if dry_run else import_movement()
    navigator = None if movement is None else movement.navigate_to_coordinate
    commands = StdinCommands()
    commands.start()

    status = "complete" if current_index >= len(path) else "running"
    state["status"] = status
    if not dry_run:
        save_json(state_file, state)

    arrived_count = 0
    stop_action: str | None = None
    phase_started = False

    try:
        if movement is not None and not movement.begin_phase(takeoff_altitude=takeoff_altitude):
            state["status"] = "failed"
            if not dry_run:
                save_json(state_file, state)
            return {"status": "failed", "arrived": 0, "current_index": current_index, "total": len(path)}
        phase_started = movement is not None

        print(
            f"{'Dry run' if dry_run else 'Flying'} boustrophedon path from "
            f"index {current_index}/{len(path)}. Type 'p' to pause, 'r' to resume, or 'q' to stop."
        )

        index = current_index
        while index < len(path):
            waypoint = path[index]
            print(
                f"[{index + 1}/{len(path)}] "
                f"lat={waypoint['lat']:.8f}, lon={waypoint['lon']:.8f}"
            )

            result = True if dry_run else navigator(
                waypoint["lat"],
                waypoint["lon"],
                interrupt_check=lambda: commands.latest_requested_action({"p", "q"}),
                monitor_offboard=True,
            )
            if result == "manual":
                state["status"] = "paused"
                if not dry_run:
                    save_json(state_file, state)
                print(
                    "[MISSION1 BOUSTRO] OFFBOARD control was released. "
                    "Saved last confirmed waypoint; resume will revisit it before continuing."
                )
                return {
                    "status": "paused",
                    "arrived": arrived_count,
                    "current_index": state["current_index"],
                    "total": len(path),
                }
            if result == "paused":
                resume_status = wait_until_resume_or_stop(commands, state_file, state, dry_run)
                if resume_status == "stopped":
                    return {
                        "status": "stopped",
                        "arrived": arrived_count,
                        "current_index": state["current_index"],
                        "total": len(path),
                    }
                continue
            if result == "stopped":
                state["status"] = "stopped"
                if not dry_run:
                    save_json(state_file, state)
                return {
                    "status": "stopped",
                    "arrived": arrived_count,
                    "current_index": state["current_index"],
                    "total": len(path),
                }
            success = bool(result)

            if not success:
                state["current_index"] = max(index - 1, 0)
                state["status"] = "failed"
                if not dry_run:
                    save_json(state_file, state)
                return {
                    "status": "failed",
                    "arrived": arrived_count,
                    "current_index": state["current_index"],
                    "total": len(path),
                }

            arrived_count += 1
            next_index = index + 1
            state["current_index"] = index
            state["status"] = "complete" if next_index >= len(path) else "running"
            if not dry_run:
                save_json(state_file, state)

            requested = commands.latest_requested_action({"p", "q"})
            if requested == "p":
                resume_status = wait_until_resume_or_stop(commands, state_file, state, dry_run)
                if resume_status == "running":
                    index += 1
                    continue
                stop_action = "stopped"
            elif requested == "q":
                stop_action = "stopped"
                state["status"] = "stopped"

            if stop_action is not None:
                if not dry_run:
                    save_json(state_file, state)
                return {
                    "status": stop_action,
                    "arrived": arrived_count,
                    "current_index": state["current_index"],
                    "total": len(path),
                }

            index += 1

        state["current_index"] = len(path)
        state["status"] = "complete"
        if not dry_run:
            save_json(state_file, state)
        if movement is not None and not movement.end_phase(land=True):
            state["status"] = "failed"
            if not dry_run:
                save_json(state_file, state)
            return {"status": "failed", "arrived": arrived_count, "current_index": state["current_index"], "total": len(path)}
        phase_started = False
        return {
            "status": "complete",
            "arrived": arrived_count,
            "current_index": state["current_index"],
            "total": len(path),
        }
    finally:
        if movement is not None:
            if phase_started and state.get("status") not in {"paused", "stopped"}:
                movement.cleanup()
            elif state.get("status") in {"paused", "stopped"}:
                movement.cleanup()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run or resume Mission 1 boustrophedon coverage."
    )
    parser.add_argument(
        "--area-file",
        type=Path,
        default=DEFAULT_AREA_FILE,
        help=f"Area JSON file with boundary and obstacles. Default: {DEFAULT_AREA_FILE}",
    )
    parser.add_argument(
        "--state-file",
        type=Path,
        default=DEFAULT_STATE_FILE,
        help=f"Path/progress JSON file to update. Default: {DEFAULT_STATE_FILE}",
    )
    parser.add_argument(
        "--settings-file",
        type=Path,
        default=DEFAULT_SETTINGS_FILE,
        help=f"YAML settings file. Default: {DEFAULT_SETTINGS_FILE}",
    )
    parser.add_argument(
        "--mission-file",
        type=Path,
        default=None,
        help="Deprecated alias used as both --area-file and --state-file.",
    )
    parser.add_argument(
        "--regenerate",
        action="store_true",
        help="Regenerate the state-file path and reset current_index to 0.",
    )
    parser.add_argument("--start-lat", type=float, default=None)
    parser.add_argument("--start-lon", type=float, default=None)
    parser.add_argument("--goal-lat", type=float, default=None)
    parser.add_argument("--goal-lon", type=float, default=None)
    parser.add_argument("--takeoff-altitude", type=float, default=5.0)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print progress without importing movement, flying, or updating state.",
    )
    return parser.parse_args()


def optional_cli_coord(args: argparse.Namespace, prefix: str) -> dict[str, float] | None:
    lat = getattr(args, f"{prefix}_lat")
    lon = getattr(args, f"{prefix}_lon")
    if (lat is None) != (lon is None):
        raise ValueError(f"--{prefix}-lat and --{prefix}-lon must be provided together")
    if lat is None:
        return None
    return {"lat": lat, "lon": lon}


def main() -> int:
    args = parse_args()
    try:
        start = optional_cli_coord(args, "start")
        goal = optional_cli_coord(args, "goal")
        summary = run_boustrophedon(
            args.area_file if args.mission_file is None else args.mission_file,
            args.state_file if args.mission_file is None else args.mission_file,
            args.settings_file,
            regenerate=args.regenerate,
            dry_run=args.dry_run,
            start=start,
            goal=goal,
            takeoff_altitude=args.takeoff_altitude,
        )
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    print(
        "Boustrophedon "
        f"{summary['status']}: arrived {summary['arrived']} waypoint(s); "
        f"current_index={summary['current_index']}/{summary['total']}"
    )
    if summary["status"] == "complete":
        return 0
    if summary["status"] in {"paused", "stopped"}:
        return 3
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
