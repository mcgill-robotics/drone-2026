#!/usr/bin/env python3
"""
Mission 1 Object Detection (standalone)

Runs the LAB-based dry/wet target detector from od_bridge2/main_lab.py on the
Intel RealSense depth camera (or a webcam fallback) and shows the annotated
feed in an OpenCV window. Does not touch api_server, MAVROS, or ROS — runs in
its own terminal beside the rest of the mission.

Usage:
    # RealSense (default):
    python3 od_bridge2/mission1_od.py

    # Webcam fallback:
    python3 od_bridge2/mission1_od.py --webcam --cam 0

    # Headless (no window) — just prints lock/lost events:
    python3 od_bridge2/mission1_od.py --headless
"""

import sys
import time
import argparse
from pathlib import Path

import cv2
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
sys.path.insert(0, str(REPO_ROOT))

from od_bridge2.main_lab import detect, load_calibration, annotate_target


CONFIRM_SECONDS = 0.3
LOST_SECONDS = 0.3


class TemporalLock:
    """A candidate must persist CONFIRM_SECONDS before publishing; the lock
    holds for LOST_SECONDS after the last sighting so brief misses (occlusion,
    a frame the detector skipped) do not drop the annotation."""

    def __init__(self, track_radius):
        self.track_radius = track_radius
        self.prev_centroid = None
        self._streak_start = None
        self._last_seen = None
        self._confirmed = False

    def update(self, centroid, now):
        if centroid is not None:
            if self._streak_start is None:
                self._streak_start = now
            self._last_seen = now
            self.prev_centroid = centroid
            if not self._confirmed and now - self._streak_start >= CONFIRM_SECONDS:
                self._confirmed = True
        elif self._last_seen is not None and now - self._last_seen > LOST_SECONDS:
            self._confirmed = False
            self._streak_start = None
            self.prev_centroid = None
        return self._confirmed


def _handle_result(annotated, stats, confirmed, last_log_state, headless):
    """Annotate the frame on confirmation and log state transitions."""
    if confirmed and stats and stats.get("centroid"):
        annotate_target(annotated, stats)
        cx, cy = stats["centroid"]
        dist = stats.get("distance")
        diam = stats.get("real_diameter")
        if dist is not None:
            label = f"{dist:.2f} m"
            if diam is not None:
                label += f"  ~{diam * 100:.0f} cm"
            cv2.putText(annotated, label, (cx + 10, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        state = (stats.get("wetness"),
                 round(dist, 2) if dist is not None else None)
        if state != last_log_state:
            print(f"[OD] LOCK wetness={state[0]} dist={state[1]}m "
                  f"diam={diam * 100 if diam is not None else None}cm "
                  f"centroid=({cx},{cy})")
            last_log_state = state
    elif last_log_state is not None:
        print("[OD] LOST")
        last_log_state = None
    return last_log_state


def run_realsense(calib, headless, verbose):
    import pyrealsense2 as rs

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    try:
        profile = pipeline.start(config)
    except RuntimeError as e:
        print(f"[OD] Could not start RealSense pipeline: {e}")
        print("[OD] Check the camera is connected (run: rs-enumerate-devices).")
        sys.exit(1)

    # Let AE/AWB converge, then freeze both. A locked WB with AE still running
    # lets brightness (and apparent saturation) drift frame to frame, which is
    # what makes the LAB mask need constant re-tuning.
    color_sensor = profile.get_device().first_color_sensor()
    try:
        color_sensor.set_option(rs.option.enable_auto_exposure, 1)
        color_sensor.set_option(rs.option.enable_auto_white_balance, 1)
        for _ in range(30):
            pipeline.wait_for_frames()
        color_sensor.set_option(rs.option.enable_auto_exposure, 0)
        color_sensor.set_option(rs.option.enable_auto_white_balance, 0)
    except Exception as e:
        print(f"[OD] Warning: could not lock color sensor options: {e}")

    fx = profile.get_stream(rs.stream.color).as_video_stream_profile() \
        .get_intrinsics().fx
    align = rs.align(rs.stream.color)
    lock = TemporalLock(calib["track_radius"])

    if headless:
        print("[OD] Headless mode, RealSense backend. Ctrl+C to quit.")
    else:
        print("[OD] RealSense backend. Press 'q' in the window to quit.")

    last_log_state = None
    try:
        while True:
            frames = align.process(pipeline.wait_for_frames())
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            frame = np.asanyarray(color_frame.get_data())
            annotated, mask, stats = detect(
                frame, calib, verbose=verbose,
                depth_frame=depth_frame, fx=fx,
                prev_centroid=lock.prev_centroid, draw=False,
            )

            centroid = stats.get("centroid") if stats else None
            confirmed = lock.update(centroid, time.monotonic())
            last_log_state = _handle_result(
                annotated, stats, confirmed, last_log_state, headless,
            )

            if not headless:
                cv2.imshow("Mission 1 OD", annotated)
                cv2.imshow("LAB mask", mask)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    except KeyboardInterrupt:
        print("\n[OD] Interrupted by user")
    finally:
        pipeline.stop()
        if not headless:
            cv2.destroyAllWindows()


def run_webcam(calib, cam_index, headless, verbose):
    cap = cv2.VideoCapture(cam_index)
    if not cap.isOpened():
        print(f"[OD] Could not open camera (device {cam_index}). "
              f"Try a different --cam index.")
        sys.exit(1)

    lock = TemporalLock(calib["track_radius"])
    if headless:
        print(f"[OD] Headless mode, webcam {cam_index}. Ctrl+C to quit.")
    else:
        print(f"[OD] Webcam {cam_index}. Press 'q' in the window to quit.")

    last_log_state = None
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break

            annotated, mask, stats = detect(
                frame, calib, verbose=verbose,
                prev_centroid=lock.prev_centroid, draw=False,
            )

            centroid = stats.get("centroid") if stats else None
            confirmed = lock.update(centroid, time.monotonic())
            last_log_state = _handle_result(
                annotated, stats, confirmed, last_log_state, headless,
            )

            if not headless:
                cv2.imshow("Mission 1 OD", annotated)
                cv2.imshow("LAB mask", mask)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    except KeyboardInterrupt:
        print("\n[OD] Interrupted by user")
    finally:
        cap.release()
        if not headless:
            cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(
        description="Mission 1 Object Detection — standalone LAB target detector "
                    "(based on od_bridge2/main_lab.py)",
    )
    parser.add_argument("--webcam", action="store_true",
                        help="Use a standard webcam instead of the RealSense")
    parser.add_argument("--cam", type=int, default=0,
                        help="Webcam device index when --webcam is set (default 0)")
    parser.add_argument("--headless", action="store_true",
                        help="No window — print LOCK/LOST events to stdout instead")
    parser.add_argument("--verbose", action="store_true",
                        help="Print per-frame detector diagnostics")
    args = parser.parse_args()

    calib = load_calibration()

    if args.webcam:
        run_webcam(calib, args.cam, args.headless, args.verbose)
    else:
        run_realsense(calib, args.headless, args.verbose)


if __name__ == "__main__":
    main()
