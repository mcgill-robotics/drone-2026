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
from flask import Flask, request, jsonify, Response, stream_with_context
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

import rclpy
from mission_controller.px4_interface import init_px4, boot_px4, stop_px4

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
camera_fx = None  # RealSense color intrinsic, set by capture thread
od_calibration = od_load_calibration() if od_load_calibration is not None else None
capture_thread_handle = None
detection_thread_handle = None
capture_stop_event = threading.Event()


def _capture_loop():
    """Single owner of the camera. Just grabs frames at camera rate — no detection."""
    global latest_frame_version, camera_fx
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
                        if camera_fx is None:
                            try:
                                vp = color.profile.as_video_stream_profile()
                                camera_fx = vp.get_intrinsics().fx
                                print(f'[API] RealSense color fx={camera_fx:.1f}')
                            except Exception:
                                pass

                    depth = aligned.get_depth_frame()
                    if depth:
                        depth_raw = np.asanyarray(depth.get_data())
                        depth_metric = depth_raw.astype(np.float32) * depth.get_units()
                        scaled = cv2.convertScaleAbs(depth_raw, alpha=0.03)
                        depth_colormap = cv2.applyColorMap(
                            cv2.bitwise_not(scaled), cv2.COLORMAP_JET
                        )
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
    return payload


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
                depth_m=depth_copy, fx=camera_fx,
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

    try:
        mediamtx_process = subprocess.Popen(
            mediamtx_cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
        )
        time.sleep(1.0)

        if mediamtx_process.poll() is None:
            print('[API] MediaMTX started on the default RTSP/WebRTC ports')
            return True

        stderr = mediamtx_process.stderr.read() if mediamtx_process.stderr else ''
        print(f'[API] MediaMTX failed to start: {stderr.strip()}')
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
        "channel": 3,        // Optional: servo channel (default 3)
        "pwm": 1900         // Optional: PWM value for activate (default 1900)
    }
    """
    if not px4_interface:
        return jsonify({'success': False, 'error': 'PX4 interface not initialized'}), 503
    
    try:
        data = request.get_json()
        action = data.get('action', '').lower()
        channel = data.get('channel', 3)
        pwm = data.get('pwm', 1900)
        
        if action == 'activate':
            success = px4_interface.activate_spray(servo_channel=channel, pwm_value=pwm)
            return jsonify({
                'success': success,
                'action': 'activate',
                'channel': channel,
                'pwm': pwm,
                'message': 'Spray activated' if success else 'Spray activation failed'
            })
        
        elif action == 'deactivate':
            success = px4_interface.deactivate_spray(servo_channel=channel)
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

@app.route('/camera/screenshot', methods=['POST'])
def camera_screenshot():
    """
    Save RGB or depth screenshot.

    POST /camera/screenshot
    {
        "view": "rgb" | "depth",
        "mission": "mission1" | "mission2"
    }
    """

    try:
        data = request.get_json(silent=True) or {}

        view = data.get("view", "rgb")
        mission = data.get("mission", "mission1")

        if view not in ("rgb", "depth"):
            return jsonify({
                "success": False,
                "error": "Invalid view. Use 'rgb' or 'depth'."
            }), 400

        if mission not in ("mission1", "mission2"):
            return jsonify({
                "success": False,
                "error": "Invalid mission. Use 'mission1' or 'mission2'."
            }), 400

        frame = _capture_camera_frame(view)

        if frame is None:
            return jsonify({
                "success": False,
                "error": "No camera frame available yet."
            }), 503

        save_dir = os.path.join(
            _REPO_ROOT,
            "screenshots",
            mission,
            view,
        )

        os.makedirs(save_dir, exist_ok=True)

        existing_files = [
            f for f in os.listdir(save_dir)
            if f.startswith(f"{view}_") and f.endswith(".jpg")
        ]

        next_index = len(existing_files) + 1

        filename = f"{view}_{next_index:04d}.jpg"
        filepath = os.path.join(save_dir, filename)

        ok = cv2.imwrite(filepath, frame)

        if not ok:
            return jsonify({
                "success": False,
                "error": "Failed to write screenshot file."
            }), 500

        return jsonify({
            "success": True,
            "mission": mission,
            "view": view,
            "filename": filename,
            "filepath": filepath,
        })

    except Exception as e:
        print(f"[API] Screenshot error: {e}")
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500

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
            fcu_url = "serial:///dev/ttyTHS1:921600"
    
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
        stop_px4()


if __name__ == '__main__':
    main()
