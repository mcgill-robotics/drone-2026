"""
oa_core/obstacle3d.py
3-D version of obstacle.py – no pygame dependency.
Each LiDAR point received from Webots becomes one Obstacle3D.
"""

from oa_core.vector3d import Vector3D


class Obstacle3D:
    def __init__(self, x, y, z, charge=300.0, radius=0.2):
        self.p      = Vector3D(x, y, z)
        self.charge = charge
        self.radius = radius   # physical size of the obstacle (metres)

    def distance_to(self, pos: Vector3D) -> float:
        return self.p.distance_to(pos)

    def distance_sq_to(self, pos: Vector3D) -> float:
        return self.p.distance_sq_to(pos)
