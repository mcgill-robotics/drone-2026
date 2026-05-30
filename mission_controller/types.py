"""
Core data types for the mission controller system.

Point: Represents GPS coordinates and waypoints (latitude, longitude, altitude)
"""


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

