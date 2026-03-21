"""
Pathfinding strategy classes for navigation and path planning
"""
from abc import ABC, abstractmethod
from .types import Point
from .stubs import generate_print_pattern, generate_potential_field_path


class PathfindingStrategy(ABC):
    """Abstract base class for pathfinding algorithms"""
    
    def __init__(self, waypoints=None):
        """
        Initialize pathfinding strategy
        
        Args:
            waypoints: List of Point objects representing waypoints
        """
        self.waypoints = waypoints if waypoints else []
    
    @abstractmethod
    def calculate_path(self, start, goal):
        """
        Calculate optimal path from start to goal
        
        Args:
            start: Point object for starting position
            goal: Point object for destination
            
        Returns:
            List of Point objects representing path
        """
        pass
    
    def get_next_waypoint(self):
        """Get the next waypoint in the path"""
        if self.waypoints:
            return self.waypoints.pop(0)
        return None
    
    def is_path_clear(self, point):
        """Check if a point is clear of obstacles"""
        print(f"  [STUB] is_path_clear({point})")
        # TODO: Implement obstacle checking
        return True


class PathPrinting(PathfindingStrategy):
    """Simple pathfinding strategy for printing/surveying patterns"""
    
    def __init__(self, waypoints=None):
        """Initialize path printing strategy"""
        super().__init__(waypoints)
    
    def calculate_path(self, start, goal):
        """
        Calculate boustrophedon (back-and-forth) path between start and goal
        Good for surveying rectangular areas
        
        Args:
            start: Starting position
            goal: Goal position
            
        Returns:
            List of waypoint Path
        """
        print(f"[PATHFINDING] Calculating print pattern from {start} to {goal}")
        waypoints = generate_print_pattern(start, goal)
        self.waypoints = waypoints
        return waypoints
    
    def heuristic(self, current, goal):
        """A* heuristic for path planning"""
        return current.distance_to(goal)


class PotentialFieldPathfinding(PathfindingStrategy):
    """Pathfinding using potential field (attractive and repulsive forces)"""
    
    def __init__(self, waypoints=None, obstacle_radius=10.0):
        """
        Initialize potential field pathfinding
        
        Args:
            waypoints: Initial waypoints
            obstacle_radius: Radius around obstacles to avoid
        """
        super().__init__(waypoints)
        self.obstacle_radius = obstacle_radius
        self.obstacles = []
        self.attractive_force_gain = 1.0
        self.repulsive_force_gain = 1.0
    
    def calculate_path(self, start, goal):
        """
        Calculate path using potential field method
        Attracts towards goal, repels from obstacles
        
        Args:
            start: Starting position
            goal: Goal position
            
        Returns:
            List of waypoints from start to goal
        """
        print(f"[PATHFINDING] Calculating potential field path from {start} to {goal}")
        waypoints = generate_potential_field_path(start, goal, self.obstacles)
        self.waypoints = waypoints
        return waypoints
    
    def add_obstacle(self, obstacle_point):
        """Add an obstacle to avoid"""
        self.obstacles.append(obstacle_point)
        print(f"[PATHFINDING] Added obstacle at {obstacle_point}")
    
    def attractive_force(self, current, goal):
        """Calculate attractive force towards goal"""
        direction = (goal.x - current.x, goal.y - current.y)
        distance = current.distance_to(goal)
        return (direction[0] / distance * self.attractive_force_gain, 
                direction[1] / distance * self.attractive_force_gain)
    
    def repulsive_force(self, current):
        """Calculate repulsive force from obstacles"""
        force_x, force_y = 0, 0
        for obstacle in self.obstacles:
            distance = current.distance_to(obstacle)
            if distance < self.obstacle_radius:
                direction = (current.x - obstacle.x, current.y - obstacle.y)
                dist_norm = (direction[0]**2 + direction[1]**2)**0.5
                if dist_norm > 0:
                    force_x += direction[0] / dist_norm * self.repulsive_force_gain
                    force_y += direction[1] / dist_norm * self.repulsive_force_gain
        return (force_x, force_y)
