"""
Unified PX4/MAVROS interface

This file is the main public entry point for the project.

What it owns:
- PX4Interface: combines telemetry getters + control setters
- MAVROS lifecycle helpers for tests/scripts
- Clean setup/shutdown wrappers for SITL and real hardware

Important naming note:
- PX4 itself is either already running in SITL, or already running on the Pixhawk.
- The helper below launches MAVROS, not PX4 firmware.
- Old names such as boot_px4()/stop_px4() are kept as compatibility aliases so older tests
  do not immediately break.
"""

from __future__ import annotations

import os
import signal
import subprocess
import time
from dataclasses import dataclass
from typing import Optional

import rclpy

from .px4_getters import PX4Getters
from .px4_setters import PX4Setters


DEFAULT_NAMESPACE = "mavros"
DEFAULT_HARDWARE_PORT = "/dev/ttyTHS1"
DEFAULT_HARDWARE_BAUD = 921600
DEFAULT_SITL_FCU_URL = "udp://:14540@localhost:14580"
DEFAULT_MAVROS_LAUNCH_PACKAGE = "mavros"
DEFAULT_MAVROS_LAUNCH_FILE = "px4.launch"


@dataclass(frozen=True)
class PX4RuntimeConfig:
    """Resolved runtime settings for MAVROS/PX4 connection."""

    mode: str
    fcu_url: str
    namespace: str = DEFAULT_NAMESPACE
    boot_mavros: bool = True
    node_name: str = "px4_interface"
    connect_timeout_s: float = 30.0
    mavros_startup_delay_s: float = 5.0


class PX4Interface(PX4Setters, PX4Getters):
    """
    Public PX4 interface.

    Inherits:
    - PX4Getters: MAVROS subscriptions + telemetry getter APIs
    - PX4Setters: MAVROS services + setpoint publisher APIs

    This lets the rest of the project interact with one object only.
    """

    pass


# Global interface instance used by older scripts.
_autopilot: Optional[PX4Interface] = None

# Global MAVROS process tracker. The name is intentionally MAVROS-specific;
# boot_px4()/stop_px4() below are compatibility aliases only.
_mavros_process: Optional[subprocess.Popen] = None
_mavros_started_by_this_process = False


# ---------------------------------------------------------------------------
# URL/config helpers
# ---------------------------------------------------------------------------


def normalize_fcu_url(fcu_url: str) -> str:
    """
    Normalize common FCU URL formatting mistakes.

    The most common bug is building serial URLs like this:
        f"serial:///{'/dev/ttyUSB0'}:921600"
    which becomes:
        serial:////dev/ttyUSB0:921600

    MAVROS expects:
        serial:///dev/ttyUSB0:921600
    """
    if not fcu_url:
        return fcu_url

    # Fix accidental four-slash Linux serial URL.
    if fcu_url.startswith("serial:////dev/"):
        return "serial:///dev/" + fcu_url[len("serial:////dev/") :]

    return fcu_url


def build_fcu_url(
    mode: str,
    *,
    port: Optional[str] = None,
    baud: int = DEFAULT_HARDWARE_BAUD,
    sitl_url: str = DEFAULT_SITL_FCU_URL,
) -> str:
    """
    Build the MAVROS fcu_url for either SITL or real hardware.

    mode:
        - "sitl": connect to an already-running PX4 SITL/Gazebo instance
        - "hardware": connect to Pixhawk/PX4 over a serial device
    """
    mode_l = mode.lower().strip()

    if mode_l == "sitl":
        return normalize_fcu_url(sitl_url)

    if mode_l in {"hardware", "real", "serial"}:
        selected_port = port or DEFAULT_HARDWARE_PORT
        # selected_port normally starts with /dev/..., so use serial:// + selected_port.
        # This intentionally produces serial:///dev/ttyXXX:baud.
        return normalize_fcu_url(f"serial://{selected_port}:{int(baud)}")

    raise ValueError("mode must be either 'sitl' or 'hardware'")


def make_runtime_config(
    mode: str,
    *,
    port: Optional[str] = None,
    baud: int = DEFAULT_HARDWARE_BAUD,
    sitl_url: str = DEFAULT_SITL_FCU_URL,
    fcu_url: Optional[str] = None,
    namespace: str = DEFAULT_NAMESPACE,
    boot_mavros: bool = True,
    node_name: str = "px4_interface",
    connect_timeout_s: float = 30.0,
    mavros_startup_delay_s: float = 5.0,
) -> PX4RuntimeConfig:
    """Resolve user/script options into one config object."""
    resolved_fcu_url = normalize_fcu_url(
        fcu_url
        if fcu_url is not None
        else build_fcu_url(mode, port=port, baud=baud, sitl_url=sitl_url)
    )

    return PX4RuntimeConfig(
        mode=mode.lower().strip(),
        fcu_url=resolved_fcu_url,
        namespace=namespace,
        boot_mavros=bool(boot_mavros),
        node_name=node_name,
        connect_timeout_s=float(connect_timeout_s),
        mavros_startup_delay_s=float(mavros_startup_delay_s),
    )


# ---------------------------------------------------------------------------
# MAVROS lifecycle helpers
# ---------------------------------------------------------------------------


def boot_mavros(
    fcu_url: str = f"serial://{DEFAULT_HARDWARE_PORT}:{DEFAULT_HARDWARE_BAUD}",
    *,
    namespace: str = DEFAULT_NAMESPACE,
    launch_package: str = DEFAULT_MAVROS_LAUNCH_PACKAGE,
    launch_file: str = DEFAULT_MAVROS_LAUNCH_FILE,
    startup_delay_s: float = 5.0,
) -> Optional[subprocess.Popen]:
    """
    Launch MAVROS for PX4.

    This does NOT boot PX4 firmware. It only starts the MAVROS ROS 2 bridge.

    Typical FCU URLs:
        SITL:     udp://:14540@localhost:14580
        Hardware: serial:///dev/ttyTHS1:921600
                  serial:///dev/ttyUSB0:921600
    """
    global _mavros_process, _mavros_started_by_this_process

    fcu_url = normalize_fcu_url(fcu_url)

    if _mavros_process is not None and _mavros_process.poll() is None:
        print(f"[MAVROS] Already running from this script (PID: {_mavros_process.pid})")
        return _mavros_process

    print(f"[MAVROS] Launching MAVROS: fcu_url={fcu_url}, namespace={namespace}")

    cmd = [
        "ros2",
        "launch",
        launch_package,
        launch_file,
        f"fcu_url:={fcu_url}",
    ]

    # Some MAVROS launch files accept namespace, some ignore it. Passing it is harmless
    # only if supported, so we avoid forcing it here to preserve compatibility with the
    # current repo's previous launch command.

    try:
        _mavros_process = subprocess.Popen(
            cmd,
            stdout=None,
            stderr=None,
            preexec_fn=os.setsid if hasattr(os, "setsid") else None,
        )
        _mavros_started_by_this_process = True

        print(f"[MAVROS] Process started (PID: {_mavros_process.pid})")
        time.sleep(max(0.0, float(startup_delay_s)))

        if _mavros_process.poll() is not None:
            print(f"[MAVROS][ERROR] Launch process exited with code {_mavros_process.poll()}")
            return None

        print("[MAVROS] Launch command is still running")
        return _mavros_process

    except FileNotFoundError:
        print("[MAVROS][ERROR] Could not find 'ros2'. Did you source ROS 2? Example:")
        print("  source /opt/ros/jazzy/setup.bash")
        return None
    except Exception as exc:
        print(f"[MAVROS][ERROR] Failed to launch MAVROS: {exc}")
        return None


def stop_mavros(timeout_s: float = 5.0) -> bool:
    """Stop the MAVROS process launched by boot_mavros()."""
    global _mavros_process, _mavros_started_by_this_process

    if _mavros_process is None:
        print("[MAVROS] No MAVROS process tracked by this script")
        return False

    if _mavros_process.poll() is not None:
        print("[MAVROS] Tracked MAVROS process is already stopped")
        _mavros_process = None
        _mavros_started_by_this_process = False
        return False

    print(f"[MAVROS] Stopping MAVROS process (PID: {_mavros_process.pid})")

    try:
        # Stop the process group so child launch processes are also cleaned up.
        if hasattr(os, "killpg"):
            os.killpg(os.getpgid(_mavros_process.pid), signal.SIGTERM)
        else:
            _mavros_process.terminate()

        _mavros_process.wait(timeout=float(timeout_s))
        print("[MAVROS] Stopped gracefully")
    except subprocess.TimeoutExpired:
        print("[MAVROS] Force killing MAVROS process")
        try:
            if hasattr(os, "killpg"):
                os.killpg(os.getpgid(_mavros_process.pid), signal.SIGKILL)
            else:
                _mavros_process.kill()
            _mavros_process.wait()
        except Exception as exc:
            print(f"[MAVROS][WARN] Force kill failed: {exc}")
    except Exception as exc:
        print(f"[MAVROS][WARN] Stop failed: {exc}")
        return False
    finally:
        _mavros_process = None
        _mavros_started_by_this_process = False

    return True


def get_mavros_status() -> str:
    """Return a human-readable status for the tracked MAVROS process."""
    if _mavros_process is None:
        return "Not started by this process"

    poll_result = _mavros_process.poll()
    if poll_result is None:
        return f"Running (PID: {_mavros_process.pid})"
    return f"Stopped (exit code: {poll_result})"


# ---------------------------------------------------------------------------
# PX4Interface setup/shutdown helpers
# ---------------------------------------------------------------------------


def init_px4(
    node_name: str = "px4_interface",
    namespace: str = DEFAULT_NAMESPACE,
    *,
    connect_timeout_s: Optional[float] = None,
) -> PX4Interface:
    """
    Initialize the global PX4 interface and connect to MAVROS.

    MAVROS must already be running before this is called.
    """
    global _autopilot

    if not rclpy.ok():
        rclpy.init()

    _autopilot = PX4Interface(node_name=node_name, namespace=namespace)

    if connect_timeout_s is not None:
        _autopilot.timeout = float(connect_timeout_s)

    _autopilot.connect()
    return _autopilot


def setup_px4(
    mode: str = "sitl",
    *,
    port: Optional[str] = None,
    baud: int = DEFAULT_HARDWARE_BAUD,
    sitl_url: str = DEFAULT_SITL_FCU_URL,
    fcu_url: Optional[str] = None,
    namespace: str = DEFAULT_NAMESPACE,
    node_name: str = "px4_interface",
    boot_mavros: bool = True,
    connect_retries: int = 3,
    retry_sleep_s: float = 2.0,
    connect_timeout_s: float = 30.0,
    mavros_startup_delay_s: float = 5.0,
) -> PX4Interface:
    """
    One-call setup helper for tests.

    This is the function new tests should prefer.

    Example SITL workflow:
        # Terminal 1: manually start PX4 + Gazebo
        # Terminal 2:
        px4 = setup_px4(mode="sitl", boot_mavros=True)

    Example hardware workflow:
        # QGroundControl opened manually
        px4 = setup_px4(mode="hardware", port="/dev/ttyUSB0", boot_mavros=True)
    """
    config = make_runtime_config(
        mode,
        port=port,
        baud=baud,
        sitl_url=sitl_url,
        fcu_url=fcu_url,
        namespace=namespace,
        boot_mavros=boot_mavros,
        node_name=node_name,
        connect_timeout_s=connect_timeout_s,
        mavros_startup_delay_s=mavros_startup_delay_s,
    )

    print("[PX4-SETUP] Runtime configuration")
    print(f"[PX4-SETUP] mode        = {config.mode}")
    print(f"[PX4-SETUP] namespace   = {config.namespace}")
    print(f"[PX4-SETUP] fcu_url     = {config.fcu_url}")
    print(f"[PX4-SETUP] boot_mavros = {config.boot_mavros}")

    if config.boot_mavros:
        proc = boot_mavros(
            fcu_url=config.fcu_url,
            namespace=config.namespace,
            startup_delay_s=config.mavros_startup_delay_s,
        )
        if proc is None:
            raise RuntimeError("Failed to launch MAVROS")

    last_px4: Optional[PX4Interface] = None
    attempts = max(1, int(connect_retries))

    for attempt in range(1, attempts + 1):
        print(f"[PX4-SETUP] Connecting to MAVROS/PX4: attempt {attempt}/{attempts}")
        last_px4 = init_px4(
            node_name=config.node_name,
            namespace=config.namespace,
            connect_timeout_s=config.connect_timeout_s,
        )

        if last_px4.connected:
            print("[PX4-SETUP] Connected to MAVROS/PX4")
            return last_px4

        # Avoid leaving multiple failed nodes alive across retries.
        try:
            last_px4.disconnect()
        except Exception:
            pass

        if attempt < attempts:
            time.sleep(float(retry_sleep_s))

    raise RuntimeError("Failed to connect to MAVROS/PX4 after retries")


def get_px4() -> Optional[PX4Interface]:
    """Get the current global PX4 interface, if one exists."""
    return _autopilot


def shutdown_px4(*, stop_mavros_process: bool = True, shutdown_ros: bool = True) -> None:
    """
    Clean up the PX4 interface and optionally stop MAVROS/ROS.

    New tests should call this in finally blocks.
    """
    global _autopilot

    if _autopilot is not None:
        try:
            _autopilot.disconnect()
        except Exception as exc:
            print(f"[PX4-SHUTDOWN][WARN] Interface disconnect failed: {exc}")
        finally:
            _autopilot = None

    if stop_mavros_process:
        try:
            stop_mavros()
        except Exception as exc:
            print(f"[PX4-SHUTDOWN][WARN] MAVROS stop failed: {exc}")

    if shutdown_ros and rclpy.ok():
        try:
            rclpy.shutdown()
        except Exception as exc:
            print(f"[PX4-SHUTDOWN][WARN] rclpy shutdown failed: {exc}")


# ---------------------------------------------------------------------------
# Backward-compatible aliases for old tests
# ---------------------------------------------------------------------------


def boot_px4(
    fcu_url: str = f"serial://{DEFAULT_HARDWARE_PORT}:{DEFAULT_HARDWARE_BAUD}",
    namespace: str = DEFAULT_NAMESPACE,
):
    """
    Compatibility alias for old scripts.

    This launches MAVROS, not PX4 firmware. Prefer boot_mavros() in new code.
    """
    print("[PX4][compat] boot_px4() called; launching MAVROS via boot_mavros().")
    return boot_mavros(fcu_url=fcu_url, namespace=namespace)


def stop_px4() -> bool:
    """
    Compatibility alias for old scripts.

    This stops the MAVROS process launched by this Python process.
    Prefer stop_mavros() or shutdown_px4() in new code.
    """
    print("[PX4][compat] stop_px4() called; stopping MAVROS via stop_mavros().")
    return stop_mavros()


def get_px4_status() -> str:
    """Compatibility alias. Prefer get_mavros_status()."""
    return get_mavros_status()
