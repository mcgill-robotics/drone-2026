"""Main robot configuration aggregator."""
from dataclasses import dataclass, field
from .physics_config import PhysicsConfig
from .navigation_config import NavigationConfig
from .environment_config import EnvironmentConfig
from .avoidance_config import AvoidanceConfig


@dataclass
class VisualizationConfig:
    """Configuration for visualization and rendering."""

    # Trail
    trail_length: int = 100

    # Pulse radar visualization
    pulse_radius_base: float = 60.0
    pulse_amplitude: float = 0.1
    pulse_frequency: float = 5.0

    # Debug visualization
    avoid_dist: float = 45.0  # Distance for drawing avoidance vectors
    vector_scale: float = 40.0  # Scale factor for force vector visualization

    # Environment mode indicator colors
    env_colors: dict = field(default_factory=lambda: {
        'SPARSE': (0, 255, 0),       # Green
        'MODERATE': (255, 255, 0),   # Yellow
        'DENSE': (255, 128, 0),      # Orange
        'CORRIDOR': (255, 0, 255),   # Magenta
    })


@dataclass
class StuckDetectionConfig:
    """Configuration for stuck detection and recovery."""

    # Stuck detection thresholds
    velocity_threshold: float = 0.1        # was 2.0 pixels/step, now metres/second , velocity below this is stuck
    stuck_counter_threshold: int = 50      # Frames to wait before recovery
    stuck_reset_threshold: int = 100       # Frames to wait before resetting wall mode

    # Recovery behavior
    escape_force_mult: float = 0.3  # Multiplier for random escape force


@dataclass
class RobotConfig:
    """Aggregated configuration for the entire robot system."""

    physics: PhysicsConfig = field(default_factory=PhysicsConfig)
    navigation: NavigationConfig = field(default_factory=NavigationConfig)
    environment: EnvironmentConfig = field(default_factory=EnvironmentConfig)
    avoidance: AvoidanceConfig = field(default_factory=AvoidanceConfig)
    visualization: VisualizationConfig = field(default_factory=VisualizationConfig)
    stuck_detection: StuckDetectionConfig = field(default_factory=StuckDetectionConfig)

    @classmethod
    def from_parameters(cls, dt=0.1, m=1.0, f_max=55.0, diam=15.0):
        """
        Create RobotConfig from legacy Robot constructor parameters.

        This provides backward compatibility with the original Robot class signature.
        """
        config = cls()
        config.physics.dt = dt
        config.physics.mass = m
        config.physics.max_force = f_max
        config.physics.diameter = diam
        return config
