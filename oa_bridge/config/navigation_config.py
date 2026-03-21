"""Navigation and waypoint configuration."""
from dataclasses import dataclass

'''
@dataclass
class NavigationConfig:
    """Configuration for navigation and waypoint management."""

    # Arrival thresholds
    arrival_radius: float = 10.0  # Radius for arriving at final waypoint
    corner_radius: float = 40.0   # Radius for cutting corners at intermediate waypoints

    # Loop detection
    loop_threshold: float = 20.0  # Distance threshold for closed-loop detection

    # Goal-seeking behavior
    slow_radius: float = 150.0    # Distance at which to start slowing down for final waypoint
'''
@dataclass
class NavigationConfig:
    """Configuration for navigation and waypoint management."""

    # Arrival thresholds
    arrival_radius: float = 0.5   # metres - arrive when within 0.5m of target
    corner_radius: float = 1.0    # metres - cutting corners at intermediate waypoints

    # Loop detection
    loop_threshold: float = 1.0   # metres - distance threshold for closed-loop detection

    # Goal-seeking behavior
    slow_radius: float = 3.0      # metres - start slowing down 3m before target