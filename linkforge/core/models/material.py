"""Material and color definitions for robot visuals."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Color:
    """RGBA color representation."""

    r: float  # Red (0.0 - 1.0)
    g: float  # Green (0.0 - 1.0)
    b: float  # Blue (0.0 - 1.0)
    a: float = 1.0  # Alpha (0.0 - 1.0)

    def __post_init__(self) -> None:
        """Validate color values."""
        for component in (self.r, self.g, self.b, self.a):
            if not 0.0 <= component <= 1.0:
                raise ValueError(f"Color components must be in range [0.0, 1.0], got {component}")

    def to_tuple(self) -> tuple[float, float, float, float]:
        """Convert to RGBA tuple."""
        return (self.r, self.g, self.b, self.a)

    def __str__(self) -> str:
        """String representation for URDF."""
        return f"{self.r} {self.g} {self.b} {self.a}"


@dataclass(frozen=True)
class Material:
    """Material properties for visual elements."""

    name: str
    color: Color | None = None
    texture: str | None = None  # Path to texture file

    def __post_init__(self) -> None:
        """Validate material has at least color or texture."""
        if self.color is None and self.texture is None:
            raise ValueError("Material must have either color or texture")


# Common predefined colors
class Colors:
    """Common colors for robotics visualization."""

    RED = Color(1.0, 0.0, 0.0, 1.0)
    GREEN = Color(0.0, 1.0, 0.0, 1.0)
    BLUE = Color(0.0, 0.0, 1.0, 1.0)
    YELLOW = Color(1.0, 1.0, 0.0, 1.0)
    CYAN = Color(0.0, 1.0, 1.0, 1.0)
    MAGENTA = Color(1.0, 0.0, 1.0, 1.0)
    WHITE = Color(1.0, 1.0, 1.0, 1.0)
    BLACK = Color(0.0, 0.0, 0.0, 1.0)
    GRAY = Color(0.5, 0.5, 0.5, 1.0)
    ORANGE = Color(1.0, 0.5, 0.0, 1.0)

    # Robotics-specific colors
    ALUMINUM = Color(0.8, 0.8, 0.9, 1.0)
    STEEL = Color(0.7, 0.7, 0.7, 1.0)
    PLASTIC_BLACK = Color(0.1, 0.1, 0.1, 1.0)
    RUBBER = Color(0.2, 0.2, 0.2, 1.0)
