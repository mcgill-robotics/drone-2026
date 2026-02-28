import pygame

class PhysicsEngine:
    def __init__(self, position, velocity, config):
        self.position = position
        self.velocity = velocity
        self.config = config

    def apply_force(self, force):
        if force.length() > self.config.max_force:
            force.scale_to_length(self.config.max_force)
        acceleration = force / self.config.mass
        self.velocity += acceleration * self.config.dt
        self.velocity *= self.config.air_resistance
        if self.velocity.length() > self.config.max_speed:
            self.velocity.scale_to_length(self.config.max_speed)

    def update(self):
        self.position += self.velocity * self.config.dt
