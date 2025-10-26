"""Geometry primitives for robot links."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path


class GeometryType(Enum):
    """Supported geometry types for URDF."""

    BOX = "box"
    CYLINDER = "cylinder"
    SPHERE = "sphere"
    CAPSULE = "capsule"
    MESH = "mesh"


@dataclass(frozen=True)
class Vector3:
    """3D vector representation."""

    x: float
    y: float
    z: float

    def __iter__(self):
        """Allow unpacking: x, y, z = vector."""
        return iter((self.x, self.y, self.z))

    def to_tuple(self) -> tuple[float, float, float]:
        """Convert to tuple."""
        return (self.x, self.y, self.z)

    def __str__(self) -> str:
        """String representation for URDF."""
        return f"{self.x} {self.y} {self.z}"


@dataclass(frozen=True)
class Quaternion:
    """Quaternion representation for rotations."""

    x: float
    y: float
    z: float
    w: float

    def to_tuple(self) -> tuple[float, float, float, float]:
        """Convert to tuple (x, y, z, w)."""
        return (self.x, self.y, self.z, self.w)

    def __str__(self) -> str:
        """String representation."""
        return f"{self.x} {self.y} {self.z} {self.w}"


@dataclass(frozen=True)
class Transform:
    """Spatial transformation (position + orientation).

    Uses XYZ position and RPY (Roll-Pitch-Yaw) orientation in radians.
    """

    xyz: Vector3 = Vector3(0.0, 0.0, 0.0)
    rpy: Vector3 = Vector3(0.0, 0.0, 0.0)  # Roll, Pitch, Yaw in radians

    @classmethod
    def identity(cls) -> Transform:
        """Create identity transform."""
        return cls()

    def __str__(self) -> str:
        """String representation."""
        return f"xyz: {self.xyz}, rpy: {self.rpy}"


# Geometry primitives


@dataclass(frozen=True)
class Box:
    """Box geometry (rectangular cuboid)."""

    size: Vector3  # width (x), depth (y), height (z)

    @property
    def type(self) -> GeometryType:
        return GeometryType.BOX

    def volume(self) -> float:
        """Calculate volume."""
        return self.size.x * self.size.y * self.size.z


@dataclass(frozen=True)
class Cylinder:
    """Cylinder geometry (axis along Z)."""

    radius: float
    length: float  # height along Z axis

    @property
    def type(self) -> GeometryType:
        return GeometryType.CYLINDER

    def volume(self) -> float:
        """Calculate volume."""
        import math

        return math.pi * self.radius**2 * self.length


@dataclass(frozen=True)
class Sphere:
    """Sphere geometry."""

    radius: float

    @property
    def type(self) -> GeometryType:
        return GeometryType.SPHERE

    def volume(self) -> float:
        """Calculate volume."""
        import math

        return (4.0 / 3.0) * math.pi * self.radius**3


@dataclass(frozen=True)
class Capsule:
    """Capsule geometry (cylinder with hemispherical ends, axis along Z)."""

    radius: float
    length: float  # length of cylindrical portion

    @property
    def type(self) -> GeometryType:
        return GeometryType.CAPSULE

    def volume(self) -> float:
        """Calculate volume."""
        import math

        cylinder_volume = math.pi * self.radius**2 * self.length
        sphere_volume = (4.0 / 3.0) * math.pi * self.radius**3
        return cylinder_volume + sphere_volume


@dataclass(frozen=True)
class Mesh:
    """Mesh geometry from file."""

    filepath: Path
    scale: Vector3 = field(default_factory=lambda: Vector3(1.0, 1.0, 1.0))

    @property
    def type(self) -> GeometryType:
        return GeometryType.MESH


# Type alias for any geometry
Geometry = Box | Cylinder | Sphere | Capsule | Mesh
