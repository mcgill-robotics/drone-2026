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
from flask import Flask, request, jsonify
from flask_cors import CORS

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


# ============================================================================
# Helper functions
# ============================================================================

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


# ============================================================================
# API Endpoints
# ============================================================================

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


# ============================================================================
# Main
# ============================================================================

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
        print("[API] ✓ Connected to PX4")
    else:
        print("[API] ✗ Failed to connect to PX4 (timeout)")
    
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
