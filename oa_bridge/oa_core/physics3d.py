"""
oa_core/physics3d.py
3-D port of PhysicsEngine.  Uses Vector3D instead of pygame.Vector2.
The Z axis is altitude: positive Z = up.
"""

from oa_core.vector3d import Vector3D


class PhysicsEngine3D:
    def __init__(self, position: Vector3D, velocity: Vector3D, config):
        self.position = position.copy()
        self.velocity = velocity.copy()
        self.config   = config

    def apply_force(self, force: Vector3D):
        """Clamp force magnitude, integrate to velocity."""
        mag = force.length()
        if mag > self.config.max_force:
            force = force.normalize() * self.config.max_force

        acceleration  = force * (1.0 / self.config.mass)
        self.velocity += acceleration * self.config.dt
        # Air resistance (damping)
        self.velocity  = self.velocity * self.config.air_resistance

        if self.velocity.length() > self.config.max_speed:
            self.velocity.scale_to_length(self.config.max_speed)

    def update(self):
        """Integrate velocity → position."""
        self.position += self.velocity * self.config.dt
