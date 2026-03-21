"""
oa_core/vector3d.py
A minimal 3-D vector that mirrors the pygame.Vector2 API used in the original
2-D code, so the ported logic stays readable.
"""

import math


class Vector3D:
    __slots__ = ("x", "y", "z")

    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    # ── arithmetic ──────────────────────────────────────────────────────
    def __add__(self, o):  return Vector3D(self.x+o.x, self.y+o.y, self.z+o.z)
    def __iadd__(self, o): self.x+=o.x; self.y+=o.y; self.z+=o.z; return self
    def __sub__(self, o):  return Vector3D(self.x-o.x, self.y-o.y, self.z-o.z)
    def __isub__(self, o): self.x-=o.x; self.y-=o.y; self.z-=o.z; return self
    def __mul__(self, s):  return Vector3D(self.x*s,   self.y*s,   self.z*s)
    def __rmul__(self, s): return self.__mul__(s)
    def __truediv__(self, s): return Vector3D(self.x/s, self.y/s, self.z/s)
    def __neg__(self):     return Vector3D(-self.x, -self.y, -self.z)

    # ── magnitude ───────────────────────────────────────────────────────
    def length(self):
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    def length_squared(self):
        return self.x**2 + self.y**2 + self.z**2

    # ── normalise ───────────────────────────────────────────────────────
    def normalize(self):
        mag = self.length()
        if mag < 1e-9:
            return Vector3D(0, 0, 0)
        return Vector3D(self.x/mag, self.y/mag, self.z/mag)

    def normalize_ip(self):
        mag = self.length()
        if mag < 1e-9:
            self.x = self.y = self.z = 0.0
        else:
            self.x /= mag; self.y /= mag; self.z /= mag

    def scale_to_length(self, length):
        self.normalize_ip()
        self.x *= length; self.y *= length; self.z *= length

    # ── dot / cross ─────────────────────────────────────────────────────
    def dot(self, o):
        return self.x*o.x + self.y*o.y + self.z*o.z

    def cross(self, o):
        return Vector3D(
            self.y*o.z - self.z*o.y,
            self.z*o.x - self.x*o.z,
            self.x*o.y - self.y*o.x,
        )

    # ── distance ────────────────────────────────────────────────────────
    def distance_to(self, o):
        return (self - o).length()

    def distance_sq_to(self, o):
        return (self - o).length_squared()

    # ── lerp ────────────────────────────────────────────────────────────
    def lerp(self, o, t):
        return Vector3D(
            self.x + (o.x - self.x)*t,
            self.y + (o.y - self.y)*t,
            self.z + (o.z - self.z)*t,
        )

    def copy(self):
        return Vector3D(self.x, self.y, self.z)

    def __repr__(self):
        return f"Vector3D({self.x:.3f}, {self.y:.3f}, {self.z:.3f})"
