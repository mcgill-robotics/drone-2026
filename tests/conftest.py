"""
Shared pytest configuration.

The drone code imports ROS 2, MAVROS, pyrealsense2, cv_bridge, and a (now
deleted) ardupilot_interface module. None of those are available in a normal
Python environment, so we install fake modules into sys.modules BEFORE any
test file imports from mission_controller.
"""
import os
import sys
from unittest.mock import MagicMock

# Make the project root importable so `import mission_controller` works
_PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)


# ---------------------------------------------------------------------------
# Mock ROS 2 / MAVROS / hardware modules
# ---------------------------------------------------------------------------
_ALWAYS_MOCK = [
    "rclpy",
    "rclpy.node",
    "rclpy.qos",
    "geometry_msgs",
    "geometry_msgs.msg",
    "nav_msgs",
    "nav_msgs.msg",
    "mavros_msgs",
    "mavros_msgs.srv",
    "mavros_msgs.msg",
    "sensor_msgs",
    "sensor_msgs.msg",
    "cv_bridge",
    "pyrealsense2",
]
for _mod in _ALWAYS_MOCK:
    sys.modules[_mod] = MagicMock()

# PX4Getters inherits from rclpy.node.Node. Give it a real class so `class X(Node)`
# works even though we've mocked rclpy. `object` is sufficient for unit tests.
sys.modules["rclpy"].node = sys.modules["rclpy.node"]
sys.modules["rclpy.node"].Node = object

# cv2 and numpy: only mock if not installed so real tests still use real libs
for _opt in ["cv2", "numpy"]:
    if _opt not in sys.modules:
        try:
            __import__(_opt)
        except ImportError:
            sys.modules[_opt] = MagicMock()

# ardupilot_interface was deleted on this branch but stubs.py / __init__.py
# still import from it. Provide a fake module so imports succeed.
_ardu = MagicMock()
sys.modules["mission_controller.ardupilot_interface"] = _ardu
