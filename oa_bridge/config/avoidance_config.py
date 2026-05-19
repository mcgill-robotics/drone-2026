"""Avoidance behavior configuration."""
from dataclasses import dataclass, field
from typing import Dict


@dataclass
class ScanRadiusConfig:
    """Scan radius configuration for obstacle detection."""
    entry_radius: float  # Scan radius when entering obstacle field
    exit_radius: float   # Scan radius when in wall-following mode


@dataclass
class EnvironmentForceProfile:
    """Force calculation parameters for one environment type."""

    # Normal (repulsion) force parameters
    normal_base: float          # Base repulsion strength
    normal_urgency_mult: float  # Urgency multiplier for normal force

    # Tangent (wall-following) force parameters
    tangent_base: float         # Base tangent force
    tangent_urgency_mult: float # Urgency multiplier for tangent force

    # Velocity-based steering
    steering_gain: float = 0.0         # Velocity-based steering gain
    #low_speed_threshold: float = 10.0  # When to switch to direct tangent push
    low_speed_threshold: float = 0.3   # metres/second
    low_speed_mult: float = 1.0        # Low-speed tangent multiplier

    # Goal force (for corridor mode)
    goal_force_strength: float = 0.0  # Forward bias toward goal


@dataclass
class AvoidanceConfig:
    """Configuration for obstacle avoidance strategies."""

    # Scan radii per environment type
    scan_radius: Dict[str, ScanRadiusConfig] = field(default_factory=lambda: {
        '''
        'SPARSE': ScanRadiusConfig(38.0, 68.0),
        'MODERATE': ScanRadiusConfig(45.0, 82.0),
        'DENSE': ScanRadiusConfig(53.0, 98.0),
        'CORRIDOR': ScanRadiusConfig(53.0, 98.0),
        '''
        # scan_radius entries — divide original values by ~20 (pixel-to-metre ratio)
        'SPARSE': ScanRadiusConfig(1.9, 3.4),
        'MODERATE': ScanRadiusConfig(2.25, 4.1),
        'DENSE': ScanRadiusConfig(2.65, 4.9),
        'CORRIDOR': ScanRadiusConfig(2.65, 4.9),
    })

    # Force profiles per environment type
    sparse: EnvironmentForceProfile = field(default_factory=lambda: EnvironmentForceProfile(
        normal_base=80.0,
        normal_urgency_mult=400.0,
        tangent_base=30.0,
        tangent_urgency_mult=0.0,
        steering_gain=0.0,
        low_speed_threshold=10.0,
        low_speed_mult=1.0
    ))

    moderate: EnvironmentForceProfile = field(default_factory=lambda: EnvironmentForceProfile(
        normal_base=50.0,
        normal_urgency_mult=300.0,
        tangent_base=180.0,
        tangent_urgency_mult=100.0,
        steering_gain=2.5,
        low_speed_threshold=10.0,
        low_speed_mult=1.3
    ))

    dense: EnvironmentForceProfile = field(default_factory=lambda: EnvironmentForceProfile(
        normal_base=40.0,
        normal_urgency_mult=350.0,
        tangent_base=220.0,
        tangent_urgency_mult=140.0,
        steering_gain=3.5,
        low_speed_threshold=10.0,
        low_speed_mult=1.4
    ))

    corridor: EnvironmentForceProfile = field(default_factory=lambda: EnvironmentForceProfile(
        normal_base=25.0,
        normal_urgency_mult=250.0,
        tangent_base=250.0,
        tangent_urgency_mult=180.0,
        steering_gain=0.0,
        low_speed_threshold=10.0,
        low_speed_mult=1.0,
        goal_force_strength=120.0
    ))

    # Memory thresholds (steps before resetting wall-following mode)
    memory_thresholds: Dict[str, int] = field(default_factory=lambda: {
        'SPARSE': 10,
        'MODERATE': 20,
        'DENSE': 25,
        'CORRIDOR': 15,
    })

    # Force smoothing factors per environment
    smoothing_factors: Dict[str, float] = field(default_factory=lambda: {
        'SPARSE': 0.5,      # More responsive
        'MODERATE': 0.4,
        'DENSE': 0.35,      # More stable
        'CORRIDOR': 0.35,
    })

    # Smoothing when no obstacles detected
    free_flight_smoothing: float = 0.6
