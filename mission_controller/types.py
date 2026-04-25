"""
Core data types and enumerations for the mission controller system
"""
from enum import Enum, auto


class MissionState(Enum):
    """Enumeration of all possible mission states"""
    INIT = auto()
    TAKEOFF = auto()
    # Mission One states
    LAPS = auto()
    TRANSIT_TO_SITE = auto()
    SEARCH_SITE = auto()
    DROP_PAYLOAD = auto()
    # Mission Two states
    ENTER_BUILDING = auto()
    SEARCH_BUILDING = auto()
    SPRAY_PADS = auto()
    EXIT_BUILDING = auto()
    # Common end states
    RETURN_HOME = auto()
    LAND = auto()
    COMPLETE = auto()


class MissionType(Enum):
    """Enumeration of available mission types"""
    MISSION_ONE = auto()  # Lap-based outdoor mission with drop payload
    MISSION_TWO = auto()  # Building entry/search/spray mission


class Mode(Enum):
    """Drone flight mode enumeration"""
    HOVER = auto()
    LAND = auto()
    ASCEND = auto()
    RETURN = auto()
    AIRBORNE = auto()


class Point:
    """Simple 2D/3D point representation for GPS and waypoint coordinates"""
    
    def __init__(self, x, y, z=0):
        """
        Initialize a point
        
        Args:
            x: Latitude or X coordinate
            y: Longitude or Y coordinate
            z: Altitude or Z coordinate (default 0)
        """
        self.x = x
        self.y = y
        self.z = z
    
    def __repr__(self):
        return f"Point({self.x}, {self.y}, {self.z})"
    
    def __eq__(self, other):
        if not isinstance(other, Point):
            return False
        return self.x == other.x and self.y == other.y and self.z == other.z
    
    def distance_to(self, other):
        """
        Calculate Euclidean distance to another point
        
        Args:
            other: Another Point object
            
        Returns:
            Distance in same units as coordinates
        """
        return ((self.x - other.x)**2 + (self.y - other.y)**2 + (self.z - other.z)**2)**0.5
    
    def to_dict(self):
        """Convert to dictionary for JSON serialization"""
        return {"x": self.x, "y": self.y, "z": self.z}
    
    @staticmethod
    def from_dict(data):
        """Create Point from dictionary"""
        return Point(data["x"], data["y"], data.get("z", 0))
