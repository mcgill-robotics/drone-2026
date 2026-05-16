"""
This file allows simpler import rules, converting the folder into packageable code.
example: instead of writing "from mission_controller.controller import MissionController", you can write 
"from mission_controller import MissionController". This is ONLY for external package users.
"""
# PX4 Interface
from .px4_interface import PX4Interface, init_px4, boot_px4, stop_px4, get_px4

# Core types
from .types import MissionState, Mode, Point

# Objectives
from .objectives import Objective, ExtinguishObjective, SurveyObjective, PayloadDeliveryObjective

# Pathfinding
from .pathfinding import PathfindingStrategy, PathPrinting, PotentialFieldPathfinding

# Mission Strategies
from .strategies import MissionStrategy, MissionOne, MissionTwo

# Core Controller
from .controller import MissionController

# Driver
from .driver import Driver

# Stub functions
from stubs import (
    goto_drone, run_lap_algorithm,
    boustrophedon_search, at_position, pad_has_extinguisher,
    drop_payload, inside_boundary, extinguish_fire, take_survey_photos,
    release_payload, generate_print_pattern, generate_potential_field_path
)

__version__ = "1.0.0"

"""
this prevents the import of everything that was defined in the file,, ie types, objectives...
"""
__all__ = [
    # PX4 Interface
    "PX4Interface", "init_px4", "boot_px4", "stop_px4", "get_px4",
    
    # Types
    "MissionState", "Mode", "Point",
    
    # Objectives
    "Objective", "ExtinguishObjective", "SurveyObjective", "PayloadDeliveryObjective",
    
    # Pathfinding
    "PathfindingStrategy", "PathPrinting", "PotentialFieldPathfinding",
    
    # Strategies
    "MissionStrategy", "MissionOne", "MissionTwo",
    
    # Core
    "MissionController",
    
    # Driver
    "Driver",
    
    # Stubs
    "goto_drone", "run_lap_algorithm",
    "boustrophedon_search", "at_position", "pad_has_extinguisher",
    "drop_payload", "inside_boundary", "extinguish_fire", "take_survey_photos",
    "release_payload", "generate_print_pattern", "generate_potential_field_path",
]
