"""
Unified PX4 interface

Combines:
- PX4Getters: telemetry subscriptions + getter APIs
- PX4Setters: service-based and publisher-based control APIs

This is the main public entry point used by the rest of the system.
"""

import time
import os
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


def boot_px4(fcu_url="serial:///dev/ttyTHS1:921600", namespace="mavros", suppress_output=True):
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
        # Honor environment override for convenience in test runs
        if os.getenv("PX4_SUPPRESS_MAVROS_OUTPUT", "").lower() in {"1", "true", "yes"}:
            suppress_output = True
        # Build the ros2 launch command
        cmd = [
            "ros2", "launch", "mavros", "px4.launch",
            f"fcu_url:={fcu_url}"
        ]

        # Start the process. Optionally suppress stdout/stderr to hide MAVROS
        # launch logs in the test terminal.
        stdout = subprocess.DEVNULL if suppress_output else None
        stderr = subprocess.DEVNULL if suppress_output else None
        _px4_process = subprocess.Popen(cmd, stdout=stdout, stderr=stderr)

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


# =========================================================
# RTSP Camera Stream Utilities
# =========================================================

def validate_rtsp_url(rtsp_url, timeout=5):
    """
    Check if an RTSP stream is accessible and responding.
    
    Uses ffprobe to check stream availability without displaying it.
    
    Args:
        rtsp_url: RTSP URL to validate (e.g., rtsp://192.168.1.100:8554/live)
        timeout: Timeout in seconds for the check
    
    Returns:
        True if stream is accessible, False otherwise
    
    Usage:
        if validate_rtsp_url("rtsp://192.168.1.100:8554/live"):
            print("Camera is online")
        else:
            print("Camera is offline")
    """
    try:
        cmd = [
            "ffprobe",
            "-v", "error",
            "-select_streams", "v:0",
            "-show_entries", "stream=codec_type",
            "-of", "default=noprint_wrappers=1:nokey=1:noval=1",
            rtsp_url
        ]
        
        result = subprocess.run(
            cmd,
            timeout=timeout,
            capture_output=True,
            text=True
        )
        
        return result.returncode == 0
        
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False
    except Exception:
        return False


def start_rtsp_viewer_subprocess(rtsp_url, window_name="RTSP Camera Feed"):
    """
    Start the RTSP camera viewer in a background subprocess.
    
    Launches test_mission1_camera_viewer.py as a separate process,
    allowing independent monitoring of the camera feed.
    
    Args:
        rtsp_url: RTSP stream URL
        window_name: Window title for the viewer
    
    Returns:
        subprocess.Popen object if successful, None if failed
    
    Usage:
        # Start viewer in background
        viewer_proc = start_rtsp_viewer_subprocess("rtsp://192.168.1.100:8554/live")
        
        # Do mission stuff...
        
        # Stop viewer when done
        if viewer_proc:
            viewer_proc.terminate()
            viewer_proc.wait()
    """
    try:
        # Find the camera viewer script
        script_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        viewer_script = os.path.join(
            script_dir,
            "tests",
            "mission1",
            "test_mission1_camera_viewer.py"
        )
        
        if not os.path.exists(viewer_script):
            print(f"[PX4] Camera viewer script not found: {viewer_script}")
            return None
        
        cmd = [
            "python3",
            viewer_script,
            "--rtsp-url", rtsp_url,
            "--window-name", window_name,
        ]
        
        print(f"[PX4] Starting RTSP viewer: {' '.join(cmd)}")
        
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        
        # Give it a moment to start
        time.sleep(1)
        
        if proc.poll() is None:
            print(f"[PX4] ✓ RTSP viewer started (PID: {proc.pid})")
            return proc
        else:
            stdout, stderr = proc.communicate()
            print(f"[PX4] RTSP viewer failed to start:")
            if stderr:
                print(f"[PX4] {stderr.decode()}")
            return None
            
    except Exception as e:
        print(f"[PX4] Failed to start RTSP viewer: {str(e)}")
        return None


def stop_rtsp_viewer(viewer_proc, timeout=5):
    """
    Stop the RTSP camera viewer subprocess.
    
    Args:
        viewer_proc: subprocess.Popen object from start_rtsp_viewer_subprocess()
        timeout: Timeout in seconds to wait for graceful shutdown
    
    Returns:
        True if stopped successfully, False if timeout or error
    """
    if viewer_proc is None:
        return True
    
    if viewer_proc.poll() is not None:
        print("[PX4] RTSP viewer already stopped")
        return True
    
    try:
        print("[PX4] Stopping RTSP viewer...")
        viewer_proc.terminate()
        viewer_proc.wait(timeout=timeout)
        print("[PX4] ✓ RTSP viewer stopped")
        return True
        
    except subprocess.TimeoutExpired:
        print("[PX4] RTSP viewer did not stop gracefully, killing...")
        viewer_proc.kill()
        viewer_proc.wait()
        return False
    except Exception as e:
        print(f"[PX4] Error stopping RTSP viewer: {str(e)}")
        return False