"""Physics simulation configuration."""
from dataclasses import dataclass


@dataclass
class PhysicsConfig:
    """Configuration for physics simulation."""

    # Time step
    dt: float = 0.1

    # Mass and force limits
    mass: float = 1.0
    max_force: float = 5.0     # was 55.0 pixels
    max_speed: float = 1.0     # was 30.0 pixels, now metres/second

    # Air resistance (velocity damping factor)
    air_resistance: float = 0.98

    # Robot physical properties
    diameter: float = 0.3      # was 15.0 pixels, Mavic 2 Pro is ~0.3m

    # Velocity smoothing factor
    velocity_smoothing: float = 0.25  # this one is fine
