"""Physics simulation configuration."""
from dataclasses import dataclass


@dataclass
class PhysicsConfig:
    """Configuration for physics simulation."""

    # Time step
    dt: float = 0.1

    # Mass and force limits
    mass: float = 1.0
    max_force: float = 55.0
    max_speed: float = 30.0

    # Air resistance (velocity damping factor)
    air_resistance: float = 0.98

    # Robot physical properties
    diameter: float = 15.0

    # Velocity smoothing factor
    velocity_smoothing: float = 0.25
