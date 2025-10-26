"""Joint model representing connections between links."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from .geometry import Transform, Vector3


class JointType(Enum):
    """URDF joint types."""

    REVOLUTE = "revolute"  # Rotates around axis with limits
    CONTINUOUS = "continuous"  # Rotates around axis without limits
    PRISMATIC = "prismatic"  # Slides along axis with limits
    FIXED = "fixed"  # No motion
    FLOATING = "floating"  # 6 DOF (free in space)
    PLANAR = "planar"  # 2D motion in a plane


@dataclass(frozen=True)
class JointLimits:
    """Joint limits for revolute/prismatic joints."""

    lower: float  # Lower limit (radians for revolute, meters for prismatic)
    upper: float  # Upper limit
    effort: float = 0.0  # Maximum effort (N or Nm)
    velocity: float = 0.0  # Maximum velocity (rad/s or m/s)

    def __post_init__(self) -> None:
        """Validate limits."""
        if self.lower > self.upper:
            raise ValueError(f"Lower limit ({self.lower}) must be <= upper limit ({self.upper})")
        if self.effort < 0:
            raise ValueError(f"Effort must be non-negative, got {self.effort}")
        if self.velocity < 0:
            raise ValueError(f"Velocity must be non-negative, got {self.velocity}")


@dataclass(frozen=True)
class JointDynamics:
    """Joint dynamics properties."""

    damping: float = 0.0  # Damping coefficient
    friction: float = 0.0  # Friction coefficient

    def __post_init__(self) -> None:
        """Validate dynamics."""
        if self.damping < 0:
            raise ValueError(f"Damping must be non-negative, got {self.damping}")
        if self.friction < 0:
            raise ValueError(f"Friction must be non-negative, got {self.friction}")


@dataclass(frozen=True)
class JointCalibration:
    """Joint calibration parameters."""

    rising: float | None = None
    falling: float | None = None


@dataclass(frozen=True)
class JointMimic:
    """Joint mimic configuration (this joint mimics another)."""

    joint: str  # Name of joint to mimic
    multiplier: float = 1.0
    offset: float = 0.0


@dataclass(frozen=True)
class JointSafetyController:
    """Joint safety controller parameters."""

    soft_lower_limit: float
    soft_upper_limit: float
    k_position: float
    k_velocity: float


@dataclass(frozen=True)
class Joint:
    """Robot joint (connection between two links).

    Defines the kinematic relationship between parent and child links.
    """

    name: str
    type: JointType
    parent: str  # Parent link name
    child: str  # Child link name
    origin: Transform = Transform.identity()
    axis: Vector3 = Vector3(1.0, 0.0, 0.0)  # Joint axis (default: X-axis)
    limits: JointLimits | None = None
    dynamics: JointDynamics | None = None
    calibration: JointCalibration | None = None
    mimic: JointMimic | None = None
    safety_controller: JointSafetyController | None = None

    def __post_init__(self) -> None:
        """Validate joint configuration."""
        if not self.name:
            raise ValueError("Joint name cannot be empty")

        # Validate naming convention
        if not all(c.isalnum() or c in ("_", "-") for c in self.name):
            raise ValueError(
                f"Joint name '{self.name}' contains invalid characters. "
                "Use only alphanumeric, underscore, or hyphen."
            )

        if not self.parent:
            raise ValueError("Parent link name cannot be empty")

        if not self.child:
            raise ValueError("Child link name cannot be empty")

        if self.parent == self.child:
            raise ValueError(f"Parent and child cannot be the same: {self.parent}")

        # Revolute and prismatic joints require limits
        if self.type in (JointType.REVOLUTE, JointType.PRISMATIC) and self.limits is None:
            raise ValueError(f"{self.type.value} joints require limits")

        # Fixed joints should not have limits
        if self.type == JointType.FIXED and self.limits is not None:
            raise ValueError("Fixed joints cannot have limits")

        # Validate axis is not zero
        if self.axis.x == 0 and self.axis.y == 0 and self.axis.z == 0:
            raise ValueError("Joint axis cannot be zero vector")

    @property
    def is_actuated(self) -> bool:
        """Check if joint is actuated (revolute, continuous, or prismatic)."""
        return self.type in (JointType.REVOLUTE, JointType.CONTINUOUS, JointType.PRISMATIC)

    @property
    def degrees_of_freedom(self) -> int:
        """Get number of degrees of freedom."""
        dof_map = {
            JointType.FIXED: 0,
            JointType.REVOLUTE: 1,
            JointType.CONTINUOUS: 1,
            JointType.PRISMATIC: 1,
            JointType.PLANAR: 2,
            JointType.FLOATING: 6,
        }
        return dof_map[self.type]
