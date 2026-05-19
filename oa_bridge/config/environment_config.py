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

    # Environment classification thresholds.
    # density = (obstacles within scan_area_radius) / sphere_volume * 1000.
    # With scan_area_radius=5 m the volume is ~524 m³, so density ≈ 1.9 * count.
    # The lidar bridge voxel-downsamples obstacles (~one point per 0.4 m), so a
    # typical wall in view is a few dozen points. The old 0.03/0.08 thresholds
    # were pixel-space leftovers — they made every non-empty scan read as DENSE.
    #   MODERATE: ~6+ obstacle points near the drone
    #   DENSE:    ~30+ obstacle points near the drone
    density_threshold_dense: float = 55.0      # Density above this is DENSE
    density_threshold_moderate: float = 10.0   # Density above this is MODERATE
    corridor_confidence_threshold: float = 0.6 # Corridor detection confidence threshold

    # Corridor detection parameters
    corridor_angle_tolerance: float = 0.3  # Radians tolerance for left/right wall detection

    # Environment history (for stability)
    env_history_length: int = 5
    env_history_min_samples: int = 3  # Minimum samples before using history


    