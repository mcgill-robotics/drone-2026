"""
Unified PX4 interface

Combines:
- PX4Getters: telemetry subscriptions + getter APIs
- PX4Setters: service-based and publisher-based control APIs

This is the main public entry point used by the rest of the system.
"""

import time
import rclpy
import subprocess

from .px4_getters import PX4Getters
from .px4_setters import PX4Setters


class PX4Interface(PX4Setters, PX4Getters):
    """
    Public PX4 interface.

    Inherits:
    - PX4Getters: subscriptions + telemetry getters
    - PX4Setters: services + setpoint publishers

    This lets the rest of the project interact with one object only.
    """
    pass


# Global instance
_autopilot = None

# Global PX4 boot process tracker
_px4_process = None


def boot_px4(fcu_url="serial:///dev/ttyTHS1:921600", namespace="mavros"):
    """
    Boot PX4 via ros2 launch

    Args:
        fcu_url: Flight Control Unit URL
                - For SITL: "udp://127.0.0.1:14540"
                - For hardware: "serial:///dev/ttyUSB0:921600" or similar
        namespace: MAVROS namespace (default "mavros")

    Returns:
        subprocess.Popen object if successful, None if failed
    """
    global _px4_process

    # Check if PX4 is already running
    if _px4_process is not None and _px4_process.poll() is None:
        print("[PX4] PX4 is already running (PID: {})".format(_px4_process.pid))
        return _px4_process

    print(f"[PX4] Booting PX4 with FCU URL: {fcu_url}")

    try:
        # Build the ros2 launch command
        cmd = [
            "ros2", "launch", "mavros", "px4.launch",
            f"fcu_url:={fcu_url}"
        ]

        # Start the process
        _px4_process = subprocess.Popen(cmd)

        print(f"[PX4] PX4 booting process started (PID: {_px4_process.pid})")
        print("[PX4] Waiting for PX4 to initialize...")
        # Wait up to 30 seconds for process to stabilize
        time.sleep(10)
        # Check if process crashed
        if _px4_process.poll() is not None:
            print(f"[PX4] ERROR: Process exited with code {_px4_process.poll()}")
            return None
        
        print("[PX4] ✓ MAVROS boot complete")
        return _px4_process

    except Exception as e:
        print(f"[PX4] Failed to boot PX4: {str(e)}")
        return None


def stop_px4():
    """
    Stop the PX4/MAVROS process

    Returns:
        True if process was stopped, False otherwise
    """
    global _px4_process

    if _px4_process is None:
        print("[PX4] No PX4 process to stop")
        return False

    if _px4_process.poll() is None:  # Process is still running
        print(f"[PX4] Stopping PX4 process (PID: {_px4_process.pid})")
        _px4_process.terminate()

        try:
            _px4_process.wait(timeout=5)
            print("[PX4] PX4 process stopped gracefully")
        except subprocess.TimeoutExpired:
            print("[PX4] Force killing PX4 process")
            _px4_process.kill()
            _px4_process.wait()

        return True
    else:
        print("[PX4] PX4 process already stopped")
        return False


def get_px4_status():
    """Get status of PX4 boot process"""
    global _px4_process

    if _px4_process is None:
        return "Not started"

    poll_result = _px4_process.poll()
    if poll_result is None:
        return f"Running (PID: {_px4_process.pid})"
    else:
        return f"Stopped (exit code: {poll_result})"


def init_px4(node_name="px4_interface", namespace="mavros"):
    """Initialize global PX4 interface"""
    global _autopilot

    # Initialize ROS 2 if not already done
    if not rclpy.ok():
        rclpy.init()

    _autopilot = PX4Interface(node_name=node_name, namespace=namespace)
    if _autopilot.connect():
        return _autopilot
    return _autopilot


def get_px4():
    """Get global PX4 interface"""
    global _autopilot
    return _autopilot