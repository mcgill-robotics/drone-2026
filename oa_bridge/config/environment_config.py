"""Environment classification configuration."""
from dataclasses import dataclass


@dataclass
class EnvironmentConfig:
    """Configuration for environment classification and sensing."""

    # Scanning
    '''
    scan_area_radius: float = 160.0  # Radius for environment classification
    look_ahead_distance_base: float = 40.0  # Base look-ahead distance for predictive steering
    look_ahead_velocity_mult: float = 1.5   # Velocity multiplier for look-ahead distance
    '''
    scan_area_radius: float = 5.0         # was 160.0 pixels, now metres
    look_ahead_distance_base: float = 1.0  # was 40.0 pixels
    look_ahead_velocity_mult: float = 1.5  # fine, it's a multiplier

    # Environment classification thresholds
    density_threshold_dense: float = 0.08      # Density above this is DENSE
    density_threshold_moderate: float = 0.03   # Density above this is MODERATE
    corridor_confidence_threshold: float = 0.6 # Corridor detection confidence threshold

    # Corridor detection parameters
    corridor_angle_tolerance: float = 0.3  # Radians tolerance for left/right wall detection

    # Environment history (for stability)
    env_history_length: int = 5
    env_history_min_samples: int = 3  # Minimum samples before using history


    