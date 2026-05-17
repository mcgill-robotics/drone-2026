#!/usr/bin/env python3
"""Mission 1 orchestrator: laps, boustrophedon, return home."""

import argparse
import json
import subprocess
import sys
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_LAP_FILE = SCRIPT_DIR / "lap" / "lap_points.json"
DEFAULT_BOUSTRO_AREA_FILE = SCRIPT_DIR / "boustrophedon" / "area.json"
DEFAULT_BOUSTRO_STATE_FILE = SCRIPT_DIR / "boustrophedon" / "coverage_state.json"
DEFAULT_BOUSTRO_SETTINGS_FILE = SCRIPT_DIR / "boustrophedon" / "settings.yaml"
KEEP_PX4_RUNNING = False


def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def save_json(path, data):
    tmp_path = path.with_suffix(path.suffix + ".tmp")
    with tmp_path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
        f.write("\n")
    tmp_path.replace(path)


def run_phase(script_name, args):
    cmd = [sys.executable, str(SCRIPT_DIR / script_name), *args]
    print(f"[MISSION1] Running: {' '.join(cmd)}", flush=True)
    return subprocess.run(cmd, check=False).returncode


def import_movement():
    try:
        import movement
    except ImportError as exc:
        raise RuntimeError(
            "Cannot import Mission 1 movement module. Use --dry-run or keep movement.py available."
        ) from exc
    return movement


def navigate_or_print(coord, dry_run, label):
    if coord is None:
        print(f"[MISSION1] No {label} coordinate configured")
        return True

    lat = float(coord["lat"])
    lon = float(coord["lon"])
    if dry_run:
        print(f"[MISSION1][DRY] Would navigate to {label}: lat={lat}, lon={lon}")
        return True

    print(f"[MISSION1] Navigating to {label}: lat={lat}, lon={lon}")
    movement = import_movement()
    return bool(movement.navigate_to_coordinate(lat, lon))


def coord_args(prefix, coord):
    return [
        f"--{prefix}-lat",
        str(float(coord["lat"])),
        f"--{prefix}-lon",
        str(float(coord["lon"])),
    ]


def resolve_coord(path, *keys):
    if not path.exists():
        return None
    data = load_json(path)
    for key in keys:
        coord = data.get(key)
        if coord is not None:
            return coord
    return None


def populate_runtime_origin(lap_file, area_file, coord):
    lap_data = load_json(lap_file)
    lap_data["start"] = coord
    lap_data.setdefault("return", coord)
    if lap_data.get("return") is None:
        lap_data["return"] = coord
    save_json(lap_file, lap_data)

    area_data = load_json(area_file)
    boustro = area_data.get("boustrophedon")
    if isinstance(boustro, dict):
        boustro["origin"] = coord
    else:
        area_data["origin"] = coord
    save_json(area_file, area_data)


def main():
    parser = argparse.ArgumentParser(description="Run Mission 1: laps, boustrophedon, return")
    parser.add_argument("--lap-file", default=str(DEFAULT_LAP_FILE))
    parser.add_argument("--boustrophedon-area-file", default=str(DEFAULT_BOUSTRO_AREA_FILE))
    parser.add_argument("--boustrophedon-state-file", default=str(DEFAULT_BOUSTRO_STATE_FILE))
    parser.add_argument("--boustrophedon-settings-file", default=str(DEFAULT_BOUSTRO_SETTINGS_FILE))
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--max-laps", type=int, default=None)
    parser.add_argument("--regenerate-boustrophedon", action="store_true")
    parser.add_argument("--skip-laps", action="store_true")
    parser.add_argument("--skip-boustrophedon", action="store_true")
    parser.add_argument("--skip-return", action="store_true")
    parser.add_argument("--sitl", action="store_true")
    parser.add_argument("--hardware", action="store_true")
    parser.add_argument("--port", default=None)
    parser.add_argument("--takeoff-altitude", type=float, default=5.0)
    parser.add_argument("--skip-setup", action="store_true")
    args = parser.parse_args()

    lap_file = Path(args.lap_file).expanduser().resolve()
    boustro_area_file = Path(args.boustrophedon_area_file).expanduser().resolve()
    boustro_state_file = Path(args.boustrophedon_state_file).expanduser().resolve()
    boustro_settings_file = Path(args.boustrophedon_settings_file).expanduser().resolve()

    global KEEP_PX4_RUNNING
    if not args.dry_run and not args.skip_setup:
        import setup_mission1

        port = "/dev/ttyUSB0" if args.hardware else args.port
        if not setup_mission1.setup(sitl=args.sitl, port=port):
            return False
        movement = import_movement()
        origin_coord = movement.current_gps_coordinate()
        if origin_coord is None:
            print("[MISSION1] Could not read current GPS coordinate for mission origin")
            return False
        print(
            "[MISSION1] Runtime origin: "
            f"lat={origin_coord['lat']:.7f}, lon={origin_coord['lon']:.7f}"
        )
        populate_runtime_origin(lap_file, boustro_area_file, origin_coord)

    return_coord = resolve_coord(lap_file, "return", "start")
    phase2_start = resolve_coord(lap_file, "after_lap", "transition")
    if phase2_start is None:
        print("[MISSION1] Lap file must define after_lap for phase 2 start")
        return False

    resume_boustrophedon = False
    if boustro_state_file.exists() and not args.regenerate_boustrophedon:
        state = load_json(boustro_state_file)
        path = state.get("path") or []
        resume_boustrophedon = bool(path) and state.get("status") in {
            "running",
            "paused",
            "stopped",
            "failed",
        }

    if resume_boustrophedon:
        print("[MISSION1] Resuming phase 2 from saved boustrophedon state; skipping laps")

    if not args.skip_laps and not resume_boustrophedon:
        lap_args = ["--lap-file", str(lap_file)]
        if args.dry_run:
            lap_args.append("--dry-run")
        if args.max_laps is not None:
            lap_args += ["--max-laps", str(args.max_laps)]
        lap_args += ["--takeoff-altitude", str(args.takeoff_altitude)]
        if run_phase("test_mission1_lap.py", lap_args) != 0:
            print("[MISSION1] Lap phase failed")
            return False

    if not args.skip_boustrophedon:
        boust_args = [
            "--area-file",
            str(boustro_area_file),
            "--state-file",
            str(boustro_state_file),
            "--settings-file",
            str(boustro_settings_file),
        ]
        boust_args += coord_args("start", phase2_start)
        if return_coord is not None:
            boust_args += coord_args("goal", return_coord)
        if args.dry_run:
            boust_args.append("--dry-run")
        if args.regenerate_boustrophedon:
            boust_args.append("--regenerate")
        boust_args += ["--takeoff-altitude", str(args.takeoff_altitude)]
        boust_code = run_phase("test_mission1_boustrophedon.py", boust_args)
        if boust_code == 3:
            print("[MISSION1] Boustrophedon paused/stopped; leaving mission for manual resume")
            KEEP_PX4_RUNNING = True
            return True
        if boust_code != 0:
            print("[MISSION1] Boustrophedon phase failed")
            return False

    if not args.skip_return and args.skip_boustrophedon:
        if args.dry_run:
            if not navigate_or_print(return_coord, True, "return"):
                return False
        else:
            movement = import_movement()
            if not movement.begin_phase(takeoff_altitude=args.takeoff_altitude):
                return False
            try:
                if not navigate_or_print(return_coord, False, "return"):
                    print("[MISSION1] Return navigation failed")
                    return False
                if not movement.end_phase(land=True):
                    return False
            finally:
                movement.cleanup()

    print("[MISSION1] Mission 1 orchestration complete")
    return True


if __name__ == "__main__":
    try:
        success = main()
    finally:
        if not KEEP_PX4_RUNNING:
            try:
                import movement

                movement.cleanup(stop_px4_process=True)
            except Exception:
                pass
    sys.exit(0 if success else 1)
