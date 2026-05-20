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
import time
import threading
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

# ============================================================================
# Flask app setup
# ============================================================================
app = Flask(__name__)
CORS(app)  # Enable CORS for all routes

# Global state
px4_interface = None
px4_thread = None
depth_camera_lock = threading.Lock()
depth_camera_state = {
    'backend': None,
    'pipeline': None,
    'align': None,
    'capture': None,
}




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

    state = _open_depth_camera()
    try:
        while True:
            if state['backend'] == 'realsense':
                frames = state['pipeline'].wait_for_frames()
                aligned = state['align'].process(frames)

                if view == 'rgb':
                    color_frame = aligned.get_color_frame()
                    if not color_frame:
                        continue
                    frame = np.asanyarray(color_frame.get_data())
                else:
                    depth_frame = aligned.get_depth_frame()
                    if not depth_frame:
                        continue
                    depth_image = np.asanyarray(depth_frame.get_data())
                    frame = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            else:
                ok, frame = state['capture'].read()
                if not ok:
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
    print("TESTTT")
    # Initialize PX4 interface
    print("fcu is", fcu_url)
    if not init_px4_interface(fcu_url, args.namespace):
        print("[API] Failed to initialize PX4 interface")
        sys.exit(1)
    
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
        stop_px4()


if __name__ == '__main__':
    main()
