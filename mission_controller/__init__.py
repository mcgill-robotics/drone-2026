"""
This file allows simpler import rules, converting the folder into packageable code.
example: instead of writing "from mission_controller.types import Point", you can write 
"from mission_controller import Point".
"""
# PX4 Interface
from .px4_interface import PX4Interface, init_px4, boot_px4, stop_px4, get_px4

# Core types
from .types import Point

__version__ = "1.0.0"

__all__ = [
    # PX4 Interface
    "PX4Interface", "init_px4", "boot_px4", "stop_px4", "get_px4",
    
    # Types
    "Point",
]
