"""
Lightweight API server for UI control of drone functions.

This server provides HTTP endpoints for the manual_controller UI to control
drone functions like spray activation, payload release, etc.

Run: python3 api_server.py [--port 5000] [--sitl] [--fcu-url <url>]

The server will:
1. Connect to PX4 via MAVROS
2. Expose HTTP endpoints for UI commands
3. Handle spray motor control via servo commands
"""

import os
import sys
import argparse
import json
import time
import threading
import shutil
import subprocess
from flask import Flask, request, jsonify, Response, stream_with_context, send_from_directory
from flask_cors import CORS

try:
    import cv2
except Exception:
    cv2 = None

try:
    import numpy as np
except Exception:
    np = None

try:
    import pyrealsense2 as rs
except Exception:
    rs = None

# Add parent directory to path
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_THIS_DIR)
sys.path.insert(0, _REPO_ROOT)

import math

import rclpy
from mission_controller.px4_interface import init_px4, boot_px4, stop_px4
from mission_controller.geo import pixel_to_gps
from mission_controller.target_registry import TargetRegistry

try:
    from od_bridge2.mission1_od import detect as od_detect, load_calibration as od_load_calibration
except Exception as _e:
    od_detect = None
    od_load_calibration = None
    print(f'[API] Object detection module unavailable: {_e}')

# ============================================================================
# Flask app setup
# ============================================================================
app = Flask(__name__)
CORS(app)  # Enable CORS for all routes

# Global state
px4_interface = None
px4_thread = None
mediamtx_process = None
stream_publishers = {
    'depth': None,
    'rgb': None,
}
depth_camera_lock = threading.RLock()
depth_camera_state = {
    'backend': None,
    'pipeline': None,
    'align': None,
    'capture': None,
}

# Shared frame buffer fed by a single capture thread. Publishers and detection
# consumers read from here so the RealSense pipeline is only driven by one
# thread (multi-threaded wait_for_frames calls starve each other).
frame_buffer_lock = threading.Lock()
latest_frames = {
    'depth': None,
    'rgb': None,
    'depth_m': None,  # metric float32 array for OD depth gating
}
latest_target = None  # detected target dict (or None)
latest_frame_version = 0
# RealSense color intrinsics, set by the capture thread on first frame. fy/ppx/ppy
# join fx so the pixel -> GPS projection can run with the true principal point
# instead of guessing image-centre.
camera_intrinsics = {'fx': None, 'fy': None, 'ppx': None, 'ppy': None}
# Last gimbal angles commanded through the API, in radians. The georeferencing
# math assumes the gimbal achieves what was last asked of it (open-loop -- the
# board has no feedback channel yet). Stays at (0, 0) until /gimbal/aim is hit.
last_gimbal_cmd = {'yaw_rad': 0.0, 'pitch_rad': 0.0}
od_calibration = od_load_calibration() if od_load_calibration is not None else None
target_registry = TargetRegistry(
    os.path.join(_THIS_DIR, 'target_registry.json'),
)
# Screenshots land here, one subfolder per capture (depth.png/rgb.png/od.png).
SCREENSHOT_DIR = os.path.join(_REPO_ROOT, 'screenshots')

# Depth band the camera can actually be trusted within. The D455's wide baseline
# puts its minimum range around 0.4 m; readings below this (or beyond the far
# limit) are noise or the sensor returning 0 for objects too close to resolve.
# Anything outside the band is treated as no-data: black in the depth view, and
# measurement refuses instead of reporting a fabricated distance. Override via
# env if a different lens/model needs a different range.
DEPTH_VALID_MIN_M = float(os.environ.get('DEPTH_VALID_MIN_M', '0.4'))
DEPTH_VALID_MAX_M = float(os.environ.get('DEPTH_VALID_MAX_M', '6.0'))
capture_thread_handle = None
detection_thread_handle = None
capture_stop_event = threading.Event()


def _build_depth_filters():
    """RealSense post-processing that fills depth holes for display + measurement.

    Holes are the zero pixels the sensor can't resolve (shiny/edge/low-texture
    surfaces, occlusions). These run in the DEPTH domain on the already-aligned
    frame. We deliberately skip the depth<->disparity round-trip: that transform
    needs the original stereo baseline, which aligning depth to colour strips,
    so doing it post-align fabricated near-depth values (the scattered red
    speckle). A gentle spatial smooth plus the hole-filling filter stays clean.
    Temporal filtering is skipped too — it smears on a moving airframe. Returns
    None without pyrealsense2.
    """
    if rs is None:
        return None
    spatial = rs.spatial_filter()
    spatial.set_option(rs.option.holes_fill, 2)   # bridge small gaps
    hole_filling = rs.hole_filling_filter()        # fill the rest from neighbours
    return {'spatial': spatial, 'hole_filling': hole_filling}


def _fill_depth_holes(depth_frame, filters):
    """Spatial smooth + hole fill over one aligned depth frame (depth domain)."""
    f = filters['spatial'].process(depth_frame)
    return filters['hole_filling'].process(f).as_depth_frame()


def _capture_loop():
    """Single owner of the camera. Just grabs frames at camera rate — no detection."""
    global latest_frame_version
    depth_filters = _build_depth_filters()
    while not capture_stop_event.is_set():
        try:
            rgb_frame = None
            depth_colormap = None
            depth_metric = None

            with depth_camera_lock:
                state = _open_depth_camera()

                if state['backend'] == 'realsense':
                    rs_frames = state['pipeline'].wait_for_frames()
                    aligned = state['align'].process(rs_frames)

                    color = aligned.get_color_frame()
                    if color:
                        rgb_frame = np.asanyarray(color.get_data())
                        if camera_intrinsics['fx'] is None:
                            try:
                                intr = color.profile.as_video_stream_profile().get_intrinsics()
                                camera_intrinsics['fx'] = intr.fx
                                camera_intrinsics['fy'] = intr.fy
                                camera_intrinsics['ppx'] = intr.ppx
                                camera_intrinsics['ppy'] = intr.ppy
                                print(f'[API] RealSense color intrinsics fx={intr.fx:.1f} '
                                      f'fy={intr.fy:.1f} ppx={intr.ppx:.1f} ppy={intr.ppy:.1f}')
                            except Exception:
                                pass

                    depth = aligned.get_depth_frame()
                    if depth:
                        units = depth.get_units()

                        # MEASUREMENT depth: the RAW sensor reading only — never
                        # the hole-filler's guesses — clamped to the band the
                        # D455 can trust. Out-of-band (too close / too far) -> 0,
                        # so a click there is refused rather than handed a
                        # fabricated number. _deproject's window still bridges the
                        # small holes; large no-data regions stay unmeasurable.
                        raw_arr = np.asanyarray(depth.get_data())
                        depth_metric = raw_arr.astype(np.float32) * units
                        depth_metric[(depth_metric < DEPTH_VALID_MIN_M) |
                                     (depth_metric > DEPTH_VALID_MAX_M)] = 0.0

                        # DISPLAY depth: hole-fill for a smooth picture, then
                        # paint anything with no trustworthy reading (an
                        # unrecoverable hole, or depth outside the valid band)
                        # black — JET maps 0 -> dark red, which would otherwise
                        # read as a phantom near surface.
                        disp = depth
                        if depth_filters is not None:
                            try:
                                disp = _fill_depth_holes(depth, depth_filters)
                            except Exception as e:
                                print(f'[API] Depth hole-filling skipped: {e}')
                        disp_arr = np.asanyarray(disp.get_data())
                        disp_metric = disp_arr.astype(np.float32) * units
                        scaled = cv2.convertScaleAbs(disp_arr, alpha=0.03)
                        depth_colormap = cv2.applyColorMap(
                            cv2.bitwise_not(scaled), cv2.COLORMAP_JET
                        )
                        depth_colormap[(disp_metric < DEPTH_VALID_MIN_M) |
                                       (disp_metric > DEPTH_VALID_MAX_M)] = (0, 0, 0)
                else:
                    ok, frame = state['capture'].read()
                    if ok:
                        rgb_frame = frame
                        depth_colormap = frame

            with frame_buffer_lock:
                if rgb_frame is not None:
                    latest_frames['rgb'] = rgb_frame
                    latest_frame_version += 1
                if depth_colormap is not None:
                    latest_frames['depth'] = depth_colormap
                if depth_metric is not None:
                    latest_frames['depth_m'] = depth_metric
        except Exception as e:
            print(f'[API] Capture loop error: {e}')
            time.sleep(0.1)


def _quat_to_euler(qx, qy, qz, qw):
    """Quaternion -> (roll, pitch, yaw) in radians. ZYX intrinsic convention."""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _estimate_target_gps(stats):
    """Project the detected centroid to lat/lon using current drone telemetry.

    Returns (lat_deg, lon_deg) or None if anything required is missing:
      - depth at the centroid (no depth -> no scale)
      - the RealSense intrinsics (set by the first frame in _capture_loop)
      - a GPS fix on the drone (no anchor -> can't georeference)
      - drone attitude (no orientation -> ray direction unknown)
    """
    if px4_interface is None:
        return None
    dist = stats.get('distance')
    cx, cy = stats.get('centroid') or (None, None)
    fx = camera_intrinsics['fx']
    fy = camera_intrinsics['fy']
    ppx = camera_intrinsics['ppx']
    ppy = camera_intrinsics['ppy']
    if None in (dist, cx, cy, fx, fy, ppx, ppy):
        return None

    gps = px4_interface.get_gps_location()
    if not gps:
        return None

    # Use the IMU attitude (the same source the controller trusts for yaw).
    att = getattr(px4_interface, 'current_attitude', None)
    if not att:
        return None
    q = att.orientation
    roll, pitch, yaw = _quat_to_euler(q.x, q.y, q.z, q.w)

    return pixel_to_gps(
        px=float(cx), py=float(cy), depth_m=float(dist),
        fx=fx, fy=fy, ppx=ppx, ppy=ppy,
        drone_lat=gps['latitude'], drone_lon=gps['longitude'],
        drone_roll=roll, drone_pitch=pitch, drone_yaw=yaw,
        gimbal_yaw=last_gimbal_cmd['yaw_rad'],
        gimbal_pitch=last_gimbal_cmd['pitch_rad'],
    )


def _serialize_target(stats):
    """Convert detect()'s stats dict into a JSON-safe payload."""
    if not stats or not stats.get('centroid'):
        return None
    x, y, w, h = stats['bbox']
    cx, cy = stats['centroid']
    payload = {
        'bbox': {'x': int(x), 'y': int(y), 'w': int(w), 'h': int(h)},
        'centroid': {'x': int(cx), 'y': int(cy)},
        'wetness': stats.get('wetness', ''),
        'dry_fraction': float(stats.get('dry_fraction', 0.0) or 0.0),
    }
    dist = stats.get('distance')
    if dist is not None:
        payload['distance_m'] = float(dist)
    diam = stats.get('real_diameter')
    if diam is not None:
        payload['diameter_m'] = float(diam)
    gps_est = _estimate_target_gps(stats)
    if gps_est is not None:
        payload['lat'] = gps_est[0]
        payload['lon'] = gps_est[1]
        # Look the target up in the registry. Same physical target seen again
        # (within MATCH_RADIUS_M) reuses its id; everything else gets a new one.
        # Without GPS we have no key, so the bbox renders without an id rather
        # than misattributing a new sighting to an unrelated cached target.
        payload['id'] = target_registry.assign(
            gps_est[0], gps_est[1], wetness=payload.get('wetness') or None,
        )
    return payload


def _draw_od_overlay(frame, target):
    """Draw the live OD view's overlay (status pill, bbox, centroid, label)
    onto an RGB frame in source pixel coords.

    `target` is the serialized latest_target payload (or None). Mirrors the
    client-side drawDetections() in manual_controller/main.js so the saved
    od.png matches what the operator sees on screen. No scaling/letterboxing
    is needed here: we draw straight onto the full 640x480 source frame.
    """
    font = cv2.FONT_HERSHEY_SIMPLEX
    locked = target is not None

    # Status pill, top-left — always shown so the view reads as "OD" even with
    # nothing locked. Colours are BGR (orange for wet, green for dry/locked).
    badge = 'OD - LOCKED' if locked else 'OD - scanning'
    (tw, th), _ = cv2.getTextSize(badge, font, 0.5, 1)
    cv2.rectangle(frame, (8, 8), (8 + tw + 12, 8 + th + 10),
                  (60, 180, 0) if locked else (40, 40, 40), -1)
    cv2.putText(frame, badge, (14, 8 + th + 4), font, 0.5, (255, 255, 255), 1)

    if not target:
        return

    wet = target.get('wetness') == 'wet'
    color = (0, 128, 255) if wet else (68, 204, 0)  # orange / green, BGR

    b = target['bbox']
    x, y, w, h = b['x'], b['y'], b['w'], b['h']
    cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)

    centroid = target.get('centroid')
    if centroid:
        cv2.circle(frame, (centroid['x'], centroid['y']), 4, (64, 64, 255), -1)

    id_tag = f"#{target['id']} " if isinstance(target.get('id'), int) else ''
    wet_label = (target.get('wetness') or 'target').upper()
    lines = [f"{id_tag}{wet_label}"]
    dist = target.get('distance_m')
    if isinstance(dist, (int, float)):
        diam = target.get('diameter_m')
        diam_str = f"  ~{round(diam * 100)} cm" if isinstance(diam, (int, float)) else ''
        lines.append(f"{dist:.2f} m{diam_str}")
    lat, lon = target.get('lat'), target.get('lon')
    if isinstance(lat, (int, float)) and isinstance(lon, (int, float)):
        lines.append(f"{lat:.6f}, {lon:.6f}")

    # Label block above the box, dropped below if it would clip the top edge.
    line_h, pad = 18, 5
    text_w = max(cv2.getTextSize(l, font, 0.5, 1)[0][0] for l in lines)
    box_w = text_w + pad * 2
    box_h = line_h * len(lines) + pad * 2
    lx = x
    ly = y - box_h - 2
    if ly < 0:
        ly = y + 2
    cv2.rectangle(frame, (lx, ly), (lx + box_w, ly + box_h), color, -1)
    for i, line in enumerate(lines):
        cv2.putText(frame, line, (lx + pad, ly + pad + line_h * (i + 1) - 5),
                    font, 0.5, (255, 255, 255), 1)


def _deproject(u, v, depth_m, intr):
    """Pixel (u, v) + metric depth -> ((X, Y, Z) camera-space metres, Z), or None.

    Grows the sampling window until it finds valid depth, so a click that lands
    in a hole (a zero pixel the sensor couldn't resolve) still resolves to the
    nearest real surface instead of failing. Returns None only if the whole
    frame has no depth at all. Pinhole model, distortion ignored — good enough
    for an on-screen estimate.
    """
    h, w = depth_m.shape[:2]
    if not (0 <= u < w and 0 <= v < h):
        return None
    z = None
    for half in (4, 8, 16, 32, 64):
        win = depth_m[max(0, v - half):min(h, v + half + 1),
                      max(0, u - half):min(w, u + half + 1)]
        valid = win[win > 0]
        if valid.size:
            z = float(np.median(valid))
            break
    if z is None:
        return None
    x = (u - intr['ppx']) / intr['fx'] * z
    y = (v - intr['ppy']) / intr['fy'] * z
    return (x, y, z), z


def _detection_loop():
    """Run main_lab.detect() on the latest RGB frame and confirm with a temporal lock.

    Mirrors the inline state machine in od_bridge2/main.py:run_realsense — a
    candidate must persist CONFIRM_SECONDS before it is published, and the lock
    holds for LOST_SECONDS after the last sighting so brief misses don't drop
    the overlay.
    """
    global latest_target
    if od_detect is None or od_calibration is None:
        print('[API] OD detect() unavailable — detection loop exiting')
        return

    CONFIRM_SECONDS = 0.3
    LOST_SECONDS = 0.3
    prev_centroid = None
    streak_start = None
    last_seen = None
    confirmed = False
    last_target_payload = None
    last_seen_version = -1

    while not capture_stop_event.is_set():
        try:
            with frame_buffer_lock:
                version = latest_frame_version
                if version == last_seen_version:
                    rgb_copy = None
                    depth_copy = None
                else:
                    rgb = latest_frames.get('rgb')
                    depth_m = latest_frames.get('depth_m')
                    rgb_copy = rgb.copy() if rgb is not None else None
                    depth_copy = depth_m.copy() if depth_m is not None else None
                    last_seen_version = version

            if rgb_copy is None:
                time.sleep(0.02)
                continue

            _frame, _mask, stats = od_detect(
                rgb_copy, od_calibration,
                depth_m=depth_copy, fx=camera_intrinsics['fx'],
                prev_centroid=prev_centroid, draw=False,
            )

            now = time.monotonic()
            centroid = stats.get('centroid') if stats else None

            if centroid is not None:
                if prev_centroid is None or streak_start is None:
                    streak_start = now
                last_seen = now
                prev_centroid = centroid
                if not confirmed and now - streak_start >= CONFIRM_SECONDS:
                    confirmed = True
                if confirmed:
                    last_target_payload = _serialize_target(stats)
            else:
                if last_seen is not None and now - last_seen > LOST_SECONDS:
                    confirmed = False
                    streak_start = None
                    prev_centroid = None
                    last_target_payload = None

            with frame_buffer_lock:
                latest_target = last_target_payload if confirmed else None
        except Exception as e:
            print(f'[API] Detection loop error: {e}')
            time.sleep(0.1)


def _start_capture_thread():
    global capture_thread_handle, detection_thread_handle
    capture_stop_event.clear()
    if capture_thread_handle is None or not capture_thread_handle.is_alive():
        capture_thread_handle = threading.Thread(target=_capture_loop, daemon=True)
        capture_thread_handle.start()
        print('[API] Camera capture thread started')
    if detection_thread_handle is None or not detection_thread_handle.is_alive():
        detection_thread_handle = threading.Thread(target=_detection_loop, daemon=True)
        detection_thread_handle.start()
        print('[API] Circle detection thread started')


def _stop_capture_thread():
    global capture_thread_handle, detection_thread_handle
    capture_stop_event.set()
    if capture_thread_handle is not None:
        capture_thread_handle.join(timeout=2)
        capture_thread_handle = None
    if detection_thread_handle is not None:
        detection_thread_handle.join(timeout=2)
        detection_thread_handle = None


def _capture_camera_frame(view='depth'):
    """Return a copy of the latest frame for the requested view, or None."""
    if cv2 is None:
        raise RuntimeError('OpenCV is required for camera streaming')

    with frame_buffer_lock:
        frame = latest_frames.get(view)
        return frame.copy() if frame is not None else None


def _run_mediamtx():
    global mediamtx_process

    if mediamtx_process is not None and mediamtx_process.poll() is None:
        return True

    mediamtx_binary = shutil.which('mediamtx') or (
        os.path.join(_REPO_ROOT, 'mediamtx')
        if os.path.isfile(os.path.join(_REPO_ROOT, 'mediamtx')) else None
    )
    if mediamtx_binary is None:
        print('[API] MediaMTX binary not found on PATH or repo root; WebRTC camera views will not be available')
        return False

    mediamtx_config = os.path.join(_REPO_ROOT, 'mediamtx.yml')
    mediamtx_cmd = [mediamtx_binary]
    if os.path.isfile(mediamtx_config):
        mediamtx_cmd.append(mediamtx_config)

    # MediaMTX logs to stdout, so capture both streams. Write to a file rather
    # than a PIPE: a PIPE we never drain would eventually block the long-running
    # process, and a file keeps the startup error around for diagnosis.
    mediamtx_log_path = os.path.join(_REPO_ROOT, 'mediamtx.log')
    try:
        mediamtx_log = open(mediamtx_log_path, 'w')
        mediamtx_process = subprocess.Popen(
            mediamtx_cmd,
            stdout=mediamtx_log,
            stderr=subprocess.STDOUT,
            text=True,
        )
        time.sleep(1.0)

        if mediamtx_process.poll() is None:
            print('[API] MediaMTX started on the default RTSP/WebRTC ports')
            return True

        try:
            with open(mediamtx_log_path) as f:
                output = f.read().strip()
        except OSError:
            output = ''
        print(f'[API] MediaMTX failed to start: {output or f"(no output; see {mediamtx_log_path})"}')
        mediamtx_process = None
        return False
    except Exception as e:
        print(f'[API] Failed to launch MediaMTX: {e}')
        mediamtx_process = None
        return False


def _start_stream_publisher(view_name):
    existing = stream_publishers.get(view_name)
    if existing is not None and existing.poll() is None:
        return True

    ffmpeg_binary = shutil.which('ffmpeg')
    if ffmpeg_binary is None:
        print(f'[API] ffmpeg not found; cannot publish {view_name} stream to MediaMTX')
        return False

    stream_url = f'rtmp://127.0.0.1:1935/{view_name}'
    cmd = [
        ffmpeg_binary,
        '-loglevel', 'error',
        '-f', 'rawvideo',
        '-pix_fmt', 'bgr24',
        '-s', '640x480',
        '-r', '30',
        '-i', 'pipe:0',
        '-an',
        '-c:v', 'libx264',
        '-preset', 'ultrafast',
        '-tune', 'zerolatency',
        '-g', '1',
        '-sc_threshold', '0',
        '-pix_fmt', 'yuv420p',
        '-f', 'flv',
        stream_url,
    ]

    try:
        proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
        )
        stream_publishers[view_name] = proc
        print(f'[API] Publishing {view_name} camera stream via RTMP to MediaMTX path /{view_name}')

        def _publisher_loop():
            target_dt = 1.0 / 30.0
            next_tick = time.monotonic()
            try:
                while proc.poll() is None:
                    now = time.monotonic()
                    sleep_for = next_tick - now
                    if sleep_for > 0:
                        time.sleep(sleep_for)
                    next_tick = max(next_tick + target_dt, time.monotonic())

                    frame = _capture_camera_frame(view_name)
                    if frame is None:
                        continue

                    if frame.shape[:2] != (480, 640):
                        frame = cv2.resize(frame, (640, 480))

                    if proc.stdin is None:
                        break

                    proc.stdin.write(frame.tobytes())
                    proc.stdin.flush()
            except BrokenPipeError:
                pass
            except Exception as e:
                print(f'[API] {view_name} publisher stopped: {e}')
            finally:
                if proc.stdin is not None:
                    try:
                        proc.stdin.close()
                    except Exception:
                        pass

        thread = threading.Thread(target=_publisher_loop, daemon=True)
        thread.start()
        return True
    except Exception as e:
        print(f'[API] Failed to start {view_name} publisher: {e}')
        return False


def _start_camera_streams():
    _run_mediamtx()
    _start_capture_thread()
    _start_stream_publisher('depth')
    _start_stream_publisher('rgb')


def _stop_camera_streams():
    for view_name, proc in list(stream_publishers.items()):
        if proc is None:
            continue
        try:
            if proc.poll() is None:
                proc.terminate()
                proc.wait(timeout=3)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass
        stream_publishers[view_name] = None

    _stop_capture_thread()

    global mediamtx_process
    if mediamtx_process is not None:
        try:
            if mediamtx_process.poll() is None:
                mediamtx_process.terminate()
                mediamtx_process.wait(timeout=3)
        except Exception:
            try:
                mediamtx_process.kill()
            except Exception:
                pass
        mediamtx_process = None




def init_px4_interface(fcu_url, namespace="mavros"):
    """Initialize the PX4 interface."""
    global px4_interface
    try:
        px4_interface = init_px4(namespace=namespace)
        if px4_interface:
            print(f"[API] PX4 Interface initialized with namespace: {namespace}")
            return True
        else:
            print("[API] Failed to initialize PX4 Interface")
            return False
    except Exception as e:
        print(f"[API] Exception during PX4 initialization: {e}")
        return False


def ros_spin_thread():
    """Background thread to spin ROS 2 node."""
    try:
        while True:
            if px4_interface:
                rclpy.spin_once(px4_interface, timeout_sec=0.01)
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("[API] ROS spin thread interrupted")
    except Exception as e:
        print(f"[API] ROS spin thread error: {e}")


def _close_depth_camera():
    with depth_camera_lock:
        if depth_camera_state['pipeline'] is not None:
            try:
                depth_camera_state['pipeline'].stop()
            except Exception:
                pass
        if depth_camera_state['capture'] is not None:
            try:
                depth_camera_state['capture'].release()
            except Exception:
                pass
        depth_camera_state['backend'] = None
        depth_camera_state['pipeline'] = None
        depth_camera_state['align'] = None
        depth_camera_state['capture'] = None


def _open_depth_camera():
    with depth_camera_lock:
        if depth_camera_state['backend'] is not None:
            return depth_camera_state

        if rs is not None and cv2 is not None and np is not None:
            try:
                pipeline = rs.pipeline()
                config = rs.config()
                config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
                config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                pipeline.start(config)
                depth_camera_state['backend'] = 'realsense'
                depth_camera_state['pipeline'] = pipeline
                depth_camera_state['align'] = rs.align(rs.stream.color)
                print('[API] Depth camera backend: realsense')
                return depth_camera_state
            except Exception as e:
                print(f'[API] RealSense depth camera unavailable: {e}')
                _close_depth_camera()

        if cv2 is not None:
            for index in (0, 1, 2):
                capture = cv2.VideoCapture(index)
                if capture.isOpened():
                    depth_camera_state['backend'] = 'webcam'
                    depth_camera_state['capture'] = capture
                    print(f'[API] Depth camera backend: webcam index {index}')
                    return depth_camera_state
                capture.release()

        raise RuntimeError('No depth camera source available')


def _generate_depth_mjpeg_frames(view='depth'):
    if cv2 is None:
        raise RuntimeError('OpenCV is required for depth camera streaming')

    try:
        while True:
            frame = _capture_camera_frame(view)
            if frame is None:
                continue

            ok, buffer = cv2.imencode('.jpg', frame)
            if not ok:
                continue

            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
    finally:
        _close_depth_camera()


@app.route('/health', methods=['GET'])
def health_check():
    """Health check endpoint."""
    connected = px4_interface.connected if px4_interface else False
    return jsonify({
        'status': 'ok',
        'connected': connected,
        'timestamp': time.time()
    })


@app.route('/spray', methods=['POST'])
def spray_control():
    """
    Spray control endpoint.
    
    POST /spray
    {
        "action": "activate" | "deactivate",
        "channel": 1,        // Optional: actuator slot (default 1 for Actuator Set 1)
        "value": 1.0         // Optional: normalized actuator value for activate (default 1.0)
    }
    """
    if not px4_interface:
        return jsonify({'success': False, 'error': 'PX4 interface not initialized'}), 503
    
    try:
        data = request.get_json()
        action = data.get('action', '').lower()
        channel = data.get('channel', 1)
        value = data.get('value', data.get('pwm', 1.0))
        
        if action == 'activate':
            print(channel, value)
            success = px4_interface.activate_spray(actuator_slot=channel, actuator_value=value)
            return jsonify({
                'success': success,
                'action': 'activate',
                'channel': channel,
                'value': value,
                'message': 'Spray activated' if success else 'Spray activation failed'
            })
        
        elif action == 'deactivate':
            success = px4_interface.deactivate_spray(actuator_slot=channel)
            return jsonify({
                'success': success,
                'action': 'deactivate',
                'channel': channel,
                'message': 'Spray deactivated' if success else 'Spray deactivation failed'
            })
        
        else:
            return jsonify({'success': False, 'error': 'Invalid action. Use "activate" or "deactivate"'}), 400
    
    except Exception as e:
        print(f"[API] Error in spray_control: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/spray/status', methods=['GET'])
def spray_status():
    """Get spray system status."""
    if not px4_interface:
        return jsonify({'connected': False, 'error': 'PX4 interface not initialized'}), 503
    
    return jsonify({
        'connected': px4_interface.connected,
        'armed': px4_interface.is_armed() if px4_interface else False,
        'timestamp': time.time()
    })


@app.route('/camera/depth.mjpg', methods=['GET'])
def depth_camera_stream():
    """Stream the live depth camera feed as MJPEG."""
    try:
        return Response(
            stream_with_context(_generate_depth_mjpeg_frames('depth')),
            mimetype='multipart/x-mixed-replace; boundary=frame'
        )
    except Exception as e:
        print(f'[API] Depth camera stream error: {e}')
        return jsonify({'success': False, 'error': str(e)}), 503


@app.route('/camera/rgb.mjpg', methods=['GET'])
def rgb_camera_stream():
    """Stream the live RGB color feed from the depth camera as MJPEG."""
    try:
        return Response(
            stream_with_context(_generate_depth_mjpeg_frames('rgb')),
            mimetype='multipart/x-mixed-replace; boundary=frame'
        )
    except Exception as e:
        print(f'[API] RGB camera stream error: {e}')
        return jsonify({'success': False, 'error': str(e)}), 503


@app.route('/camera/detections', methods=['GET'])
def detection_stream():
    """SSE stream of the detected target in source frame coords (640x480)."""
    def event_stream():
        while True:
            time.sleep(0.05)
            with frame_buffer_lock:
                target = dict(latest_target) if latest_target else None
            payload = json.dumps({
                'width': 640,
                'height': 480,
                'target': target,
            })
            yield f'data: {payload}\n\n'

    return Response(
        stream_with_context(event_stream()),
        mimetype='text/event-stream',
        headers={'Cache-Control': 'no-cache', 'X-Accel-Buffering': 'no'},
    )


@app.route('/camera/screenshot', methods=['POST'])
def camera_screenshot():
    """Capture the current depth, rgb and OD views into one subfolder.

    POST /camera/screenshot  (no body)
    -> { success, folder, files: {depth, rgb, od}, target }

    Each call drops a new timestamped subfolder under screenshots/ holding
    depth.png (depth colormap), rgb.png (color frame) and od.png (the color
    frame with the live detection overlay baked in).
    """
    if cv2 is None:
        return jsonify({'success': False, 'error': 'OpenCV not available'}), 503

    # Snapshot every view under the lock so they all come from the same instant.
    with frame_buffer_lock:
        rgb = latest_frames.get('rgb')
        depth = latest_frames.get('depth')
        depth_m = latest_frames.get('depth_m')
        rgb = rgb.copy() if rgb is not None else None
        depth = depth.copy() if depth is not None else None
        depth_m = depth_m.copy() if depth_m is not None else None
        target = dict(latest_target) if latest_target else None

    if rgb is None and depth is None:
        return jsonify({'success': False,
                        'error': 'No camera frames available yet'}), 503

    # One subfolder per capture; append a counter if two land in the same second.
    stamp = time.strftime('%Y%m%d_%H%M%S')
    subdir = os.path.join(SCREENSHOT_DIR, stamp)
    suffix = 1
    while os.path.exists(subdir):
        subdir = os.path.join(SCREENSHOT_DIR, f'{stamp}_{suffix}')
        suffix += 1
    os.makedirs(subdir, exist_ok=True)

    files = {}
    if depth is not None:
        cv2.imwrite(os.path.join(subdir, 'depth.png'), depth)
        files['depth'] = 'depth.png'
    if rgb is not None:
        cv2.imwrite(os.path.join(subdir, 'rgb.png'), rgb)
        files['rgb'] = 'rgb.png'
        od = rgb.copy()
        _draw_od_overlay(od, target)
        cv2.imwrite(os.path.join(subdir, 'od.png'), od)
        files['od'] = 'od.png'

    # Persist the metric depth + intrinsics so the capture is self-contained for
    # later distance measurement (pixel -> camera-space deprojection). Aligned
    # to color, so depth_m[y, x] corresponds to rgb.png pixel (x, y).
    intr = {k: camera_intrinsics[k] for k in ('fx', 'fy', 'ppx', 'ppy')}
    measurable = depth_m is not None and np is not None and intr['fx'] is not None
    if measurable:
        np.save(os.path.join(subdir, 'depth.npy'), depth_m)
        with open(os.path.join(subdir, 'intrinsics.json'), 'w') as f:
            json.dump(intr, f)

    print(f'[API] Screenshot saved to {subdir} ({", ".join(files)})')
    return jsonify({
        'success': True,
        'folder': subdir,
        'name': os.path.basename(subdir),
        'files': files,
        'target': target is not None,
        'measurable': measurable,
    })


@app.route('/camera/screenshots/<name>/<path:filename>', methods=['GET'])
def camera_screenshot_file(name, filename):
    """Serve a saved screenshot file (rgb.png/depth.png/od.png) for display.

    os.path.basename strips any traversal in `name`; send_from_directory does
    the same for `filename`.
    """
    directory = os.path.join(SCREENSHOT_DIR, os.path.basename(name))
    if not os.path.isdir(directory):
        return jsonify({'success': False, 'error': 'No such capture'}), 404
    return send_from_directory(directory, filename)


@app.route('/camera/measure', methods=['POST'])
def camera_measure():
    """Estimate the real-world distance between two pixels in a capture.

    POST /camera/measure
    { "name": "<capture folder>", "p1": {"x":.., "y":..}, "p2": {"x":.., "y":..} }
    -> { success, distance_m, p1:{x,y,depth_m}, p2:{x,y,depth_m} }

    Deprojects each pixel to camera-space XYZ using the capture's saved metric
    depth and intrinsics, then returns the Euclidean distance between them.
    """
    if np is None:
        return jsonify({'success': False, 'error': 'NumPy not available'}), 503

    data = request.get_json(silent=True) or {}
    name = os.path.basename(str(data.get('name', '')))
    subdir = os.path.join(SCREENSHOT_DIR, name)
    depth_path = os.path.join(subdir, 'depth.npy')
    intr_path = os.path.join(subdir, 'intrinsics.json')
    if not name or not os.path.isfile(depth_path) or not os.path.isfile(intr_path):
        return jsonify({'success': False,
                        'error': 'Capture has no depth/intrinsics to measure'}), 404

    try:
        p1, p2 = data['p1'], data['p2']
        u1, v1 = int(round(p1['x'])), int(round(p1['y']))
        u2, v2 = int(round(p2['x'])), int(round(p2['y']))
    except (KeyError, TypeError, ValueError):
        return jsonify({'success': False,
                        'error': 'p1 and p2 with x,y are required'}), 400

    depth_m = np.load(depth_path)
    with open(intr_path) as f:
        intr = json.load(f)

    a = _deproject(u1, v1, depth_m, intr)
    b = _deproject(u2, v2, depth_m, intr)
    if a is None or b is None:
        which = [name for name, pt in (('point 1', a), ('point 2', b)) if pt is None]
        return jsonify({
            'success': False,
            'error': (f"No valid depth at {' and '.join(which)} — too close or out of "
                      f"range (trusted band {DEPTH_VALID_MIN_M:.1f}–{DEPTH_VALID_MAX_M:.1f} m; "
                      f"the D455 can't read closer than ~0.4 m)."),
        }), 422

    dist = float(np.linalg.norm(np.array(a[0]) - np.array(b[0])))
    return jsonify({
        'success': True,
        'distance_m': dist,
        'p1': {'x': u1, 'y': v1, 'depth_m': a[1]},
        'p2': {'x': u2, 'y': v2, 'depth_m': b[1]},
    })


@app.route('/camera/captures', methods=['GET'])
def camera_captures():
    """List saved captures, newest first, for the gallery page.

    Each entry reports which view images exist, whether it carries the depth +
    intrinsics needed for measurement, and whether a notes.txt is present.
    """
    if not os.path.isdir(SCREENSHOT_DIR):
        return jsonify({'success': True, 'captures': []})

    captures = []
    for name in os.listdir(SCREENSHOT_DIR):
        subdir = os.path.join(SCREENSHOT_DIR, name)
        if not os.path.isdir(subdir):
            continue
        files = {view: fn for view, fn in
                 (('rgb', 'rgb.png'), ('depth', 'depth.png'), ('od', 'od.png'))
                 if os.path.isfile(os.path.join(subdir, fn))}
        captures.append({
            'name': name,
            'files': files,
            'measurable': (os.path.isfile(os.path.join(subdir, 'depth.npy')) and
                           os.path.isfile(os.path.join(subdir, 'intrinsics.json'))),
            'has_notes': os.path.isfile(os.path.join(subdir, 'notes.txt')),
            'mtime': os.path.getmtime(subdir),
        })

    captures.sort(key=lambda c: c['mtime'], reverse=True)
    return jsonify({'success': True, 'captures': captures})


@app.route('/camera/captures/<name>/notes', methods=['GET', 'POST'])
def camera_capture_notes(name):
    """Read (GET) or write (POST {notes}) the per-capture notes.txt.

    The text file lives inside the capture's own subfolder, so notes travel
    with the images.
    """
    subdir = os.path.join(SCREENSHOT_DIR, os.path.basename(name))
    if not os.path.isdir(subdir):
        return jsonify({'success': False, 'error': 'No such capture'}), 404
    notes_path = os.path.join(subdir, 'notes.txt')

    if request.method == 'GET':
        notes = ''
        if os.path.isfile(notes_path):
            with open(notes_path, encoding='utf-8') as f:
                notes = f.read()
        return jsonify({'success': True, 'notes': notes})

    data = request.get_json(silent=True) or {}
    notes = data.get('notes', '')
    if not isinstance(notes, str):
        return jsonify({'success': False, 'error': 'notes must be a string'}), 400
    with open(notes_path, 'w', encoding='utf-8') as f:
        f.write(notes)
    print(f'[API] Saved notes for capture {os.path.basename(name)} ({len(notes)} chars)')
    return jsonify({'success': True})


@app.route('/payload/release', methods=['POST'])
def payload_release():
    """
    Payload release endpoint.

    POST /payload/release
    {
        "channels": [6, 7],     // Optional: servo channels to pulse
        "release_pwm": 1900,   // Optional: PWM value for release pulse
        "neutral_pwm": 1500,    // Optional: PWM value to reset to
        "pulse_seconds": 0.5   // Optional: pulse duration
    }
    """
    if not px4_interface:
        return jsonify({'success': False, 'error': 'PX4 interface not initialized'}), 503

    try:
        data = request.get_json(silent=True) or {}
        channels = data.get('channels', [6, 7])
        release_pwm = data.get('release_pwm', 1900)
        neutral_pwm = data.get('neutral_pwm', 1500)
        pulse_seconds = data.get('pulse_seconds', 0.5)

        success = px4_interface.release_payload(
            servo_channels=tuple(channels),
            release_pwm=release_pwm,
            neutral_pwm=neutral_pwm,
            pulse_seconds=pulse_seconds,
        )

        return jsonify({
            'success': success,
            'action': 'release',
            'channels': channels,
            'release_pwm': release_pwm,
            'neutral_pwm': neutral_pwm,
            'pulse_seconds': pulse_seconds,
            'message': 'Payload released' if success else 'Payload release failed'
        })
    except Exception as e:
        print(f"[API] Error in payload_release: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/payload/small-release', methods=['POST'])
def payload_small_release():
    """
    Small payload release endpoint.

    POST /payload/small-release
    {
        "channel": 4,          // Optional: servo channel (default 4)
        "release_pwm": 1900,   // Optional: PWM value for release pulse
        "neutral_pwm": 1500,   // Optional: PWM value to reset to
        "pulse_seconds": 0.5   // Optional: pulse duration
    }
    """
    if not px4_interface:
        return jsonify({'success': False, 'error': 'PX4 interface not initialized'}), 503

    try:
        data = request.get_json(silent=True) or {}
        channel = data.get('channel', 4)
        release_pwm = data.get('release_pwm', 1900)
        neutral_pwm = data.get('neutral_pwm', 1500)
        pulse_seconds = data.get('pulse_seconds', 0.5)

        success = px4_interface.release_small_payload(
            servo_channel=channel,
            release_pwm=release_pwm,
            neutral_pwm=neutral_pwm,
            pulse_seconds=pulse_seconds,
        )

        return jsonify({
            'success': success,
            'action': 'small-release',
            'channel': channel,
            'release_pwm': release_pwm,
            'neutral_pwm': neutral_pwm,
            'pulse_seconds': pulse_seconds,
            'message': 'Small payload released' if success else 'Small payload release failed'
        })
    except Exception as e:
        print(f"[API] Error in payload_small_release: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/payload/small', methods=['POST'])
def payload_small_control():
    """
    Small payload actuator/motor control endpoint.

    POST /payload/small
    {
        "action": "start" | "stop" | "toggle",  // default: toggle
        "channel": 4,
        "pwm_on": 1900,
        "neutral_pwm": 1500
    }

    - For outputs configured as a motor (ESC) this will start/stop the motor by
      writing PWM values. For servo-type small payloads, continue to use
      /payload/small-release for a one-shot pulse.
    """
    if not px4_interface:
        return jsonify({'success': False, 'error': 'PX4 interface not initialized'}), 503

    try:
        data = request.get_json(silent=True) or {}
        action = (data.get('action') or 'toggle').lower()
        channel = data.get('channel', 4)
        pwm_on = data.get('pwm_on', 1900)
        neutral_pwm = data.get('neutral_pwm', 1500)

        if action == 'start':
            success = px4_interface.start_small_motor(servo_channel=channel, pwm_value=pwm_on)
        elif action == 'stop':
            success = px4_interface.stop_small_motor(servo_channel=channel, neutral_pwm=neutral_pwm)
        else:
            success = px4_interface.toggle_small_motor(servo_channel=channel, pwm_on=pwm_on, neutral_pwm=neutral_pwm)

        return jsonify({
            'success': success,
            'action': action,
            'channel': channel,
            'pwm_on': pwm_on,
            'neutral_pwm': neutral_pwm,
            'message': 'Small motor control executed' if success else 'Small motor control failed'
        })
    except Exception as e:
        print(f"[API] Error in payload_small_control: {e}")
@app.route('/gimbal/aim', methods=['POST'])
def gimbal_aim():
    """Record the gimbal's commanded angles for georeferencing.

    POST /gimbal/aim
    {
        "yaw_deg": 0.0,    // 0 = forward, + = right (about body-down)
        "pitch_deg": 0.0   // 0 = level, + = nose up
    }

    NOTE: this only updates the angles the GPS projection uses. Wire the actual
    serial command through GimbalInterface.set_angles separately; until the
    board has a feedback channel, we trust commanded == achieved.
    """
    try:
        data = request.get_json(silent=True) or {}
        yaw_deg = float(data.get('yaw_deg', 0.0))
        pitch_deg = float(data.get('pitch_deg', 0.0))
        last_gimbal_cmd['yaw_rad'] = math.radians(yaw_deg)
        last_gimbal_cmd['pitch_rad'] = math.radians(pitch_deg)
        return jsonify({
            'success': True,
            'yaw_deg': yaw_deg,
            'pitch_deg': pitch_deg,
        })
    except Exception as e:
        print(f"[API] Error in gimbal_aim: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/telemetry/position', methods=['GET'])
def get_position():
    """Get current drone position."""
    if not px4_interface or not px4_interface.connected:
        return jsonify({'error': 'Not connected'}), 503
    
    try:
        if px4_interface.current_position:
            pos = px4_interface.current_position.pose.position
            return jsonify({
                'x': pos.x,
                'y': pos.y,
                'z': pos.z,
                'timestamp': time.time()
            })
        else:
            return jsonify({'error': 'Position data not available'}), 503
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/telemetry/status', methods=['GET'])
def get_status():
    """Get drone status."""
    if not px4_interface or not px4_interface.connected:
        return jsonify({'error': 'Not connected'}), 503
    
    try:
        return jsonify({
            'connected': px4_interface.connected,
            'armed': px4_interface.is_armed(),
            'landed': px4_interface.is_landed(),
            'timestamp': time.time()
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/gimbal/set', methods=['POST'])
def gimbal_set():
    """
    Gimbal control endpoint.

    POST /gimbal/set
    {
        "yaw_pwm": 1500,
        "pitch_pwm": 1500,
        "yaw_channel": 8,
        "pitch_channel": 9
    }
    """

    if not px4_interface:
        return jsonify({
            'success': False,
            'error': 'PX4 interface not initialized'
        }), 503

    try:
        data = request.get_json(silent=True) or {}

        yaw_pwm = data.get('yaw_pwm', 1500)
        pitch_pwm = data.get('pitch_pwm', 1500)

        # PLACEHOLDER CHANNELS.
        # Update after confirming real servo output ports.
        yaw_channel = data.get('yaw_channel', 8)
        pitch_channel = data.get('pitch_channel', 9)

        success = px4_interface.set_gimbal(
            yaw_pwm=yaw_pwm,
            pitch_pwm=pitch_pwm,
            yaw_channel=yaw_channel,
            pitch_channel=pitch_channel,
        )

        return jsonify({
            'success': success,
            'yaw_pwm': yaw_pwm,
            'pitch_pwm': pitch_pwm,
            'yaw_channel': yaw_channel,
            'pitch_channel': pitch_channel,
        })

    except Exception as e:
        print(f"[API] Error in gimbal_set: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/camera/depth-distance', methods=['GET'])
def depth_distance_stream():
    """
    SSE stream for center-area depth distance.

    It measures the median distance near the center of the depth frame.
    If no valid depth is available, it returns a meaningful message.
    """

    def event_stream():
        while True:
            time.sleep(0.1)

            with frame_buffer_lock:
                depth_m = latest_frames.get('depth_m')
                depth_copy = depth_m.copy() if depth_m is not None else None

            payload = {
                'valid': False,
                'distance_m': None,
                'message': 'Depth distance unavailable'
            }

            if depth_copy is not None:
                h, w = depth_copy.shape[:2]

                # Center region of the image.
                # This avoids using only one noisy pixel.
                box_size = 40
                cx = w // 2
                cy = h // 2

                x1 = max(0, cx - box_size)
                x2 = min(w, cx + box_size)
                y1 = max(0, cy - box_size)
                y2 = min(h, cy + box_size)

                roi = depth_copy[y1:y2, x1:x2]

                # Keep only meaningful RealSense depth values.
                valid_depths = roi[
                    (roi > 0.1) &
                    (roi < 10.0)
                ]

                if valid_depths.size > 0:
                    distance_m = float(np.median(valid_depths))

                    payload = {
                        'valid': True,
                        'distance_m': distance_m,
                        'message': f'Distance: {distance_m:.2f} m'
                    }
                else:
                    payload = {
                        'valid': False,
                        'distance_m': None,
                        'message': 'No valid object distance detected'
                    }

            yield f"data: {json.dumps(payload)}\n\n"

    return Response(
        stream_with_context(event_stream()),
        mimetype='text/event-stream',
        headers={
            'Cache-Control': 'no-cache',
            'X-Accel-Buffering': 'no',
        },
    )

def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Lightweight API server for drone control UI'
    )
    parser.add_argument('--port', type=int, default=5000,
                        help='Flask server port (default 5000)')
    parser.add_argument('--sitl', action='store_true',
                        help='Use SITL simulation (localhost:14540)')
    parser.add_argument('--fcu-url', type=str, default=None,
                        help='FCU URL (default: auto-detect based on --sitl)')
    parser.add_argument('--namespace', type=str, default='mavros',
                        help='MAVROS namespace (default mavros)')
    parser.add_argument('--no-boot', action='store_true',
                        help='Do not boot PX4 (assume it\'s already running)')
    
    args = parser.parse_args()
    
    # Determine FCU URL
    fcu_url = args.fcu_url
    if not fcu_url:
        if args.sitl:
            fcu_url = "udp://127.0.0.1:14540"
        else:
            # fcu_url = "serial:///dev/ttyTHS1:921600"
            # Bind locally on 0.0.0.0:14550 (the port the FCU broadcasts MAVLink
            # to) and send commands back to the FCU at 192.168.144.10:14550.
            # Grammar: udp://[bind_host]:[bind_port]@[remote_host]:[remote_port].
            # The old "udp://@14550:14550" left the bind side (before '@') empty,
            # so MAVROS bound its default port (14555) instead of 14550 and never
            # saw the heartbeat. Drop the remote half (use "udp://0.0.0.0:14550@")
            # to auto-learn the FCU address if its IP isn't fixed.
            fcu_url = "udp://0.0.0.0:14550@192.168.144.10:14550"
    print(f"[API] Starting API server...")
    print(f"[API] Port: {args.port}")
    print(f"[API] FCU URL: {fcu_url}")
    print(f"[API] Namespace: {args.namespace}")
    
    # Initialize ROS 2
    try:
        rclpy.init()
    except RuntimeError:
        # Already initialized
        pass
    
    # Boot PX4 if needed
    if not args.no_boot:
        print("[API] Booting PX4...")
        boot_px4(fcu_url=fcu_url, namespace=args.namespace)
        time.sleep(2)  # Wait for boot
    # Initialize PX4 interface
    if not init_px4_interface(fcu_url, args.namespace):
        print("[API] Failed to initialize PX4 interface")
        sys.exit(1)

    _start_camera_streams()
    
    # Start ROS spin thread
    global px4_thread
    px4_thread = threading.Thread(target=ros_spin_thread, daemon=True)
    px4_thread.start()
    print("[API] ROS spin thread started")
    
    # Wait for connection
    print("[API] Waiting for PX4 connection...")
    start_time = time.time()
    while not px4_interface.connected and (time.time() - start_time) < 30:
        time.sleep(0.5)
    
    if px4_interface.connected:
        print("[API] Connected to PX4")
    else:
        print("[API] Failed to connect to PX4 (timeout)")
    
    # Start Flask server
    print(f"[API] Starting Flask server on 0.0.0.0:{args.port}")
    try:
        app.run(host='0.0.0.0', port=args.port, debug=False, threaded=True)
    except KeyboardInterrupt:
        print("\n[API] Shutting down...")
    finally:
        _stop_camera_streams()
        target_registry.flush()
        stop_px4()


if __name__ == '__main__':
    main()
