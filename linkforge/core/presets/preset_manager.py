"""Preset management system for LinkForge.

Handles saving, loading, and applying presets for joints, materials, and sensors.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any


@dataclass
class JointPreset:
    """Preset for joint configuration."""

    name: str
    description: str = ""
    joint_type: str = "REVOLUTE"
    axis: str = "Z"
    use_limits: bool = True
    limit_lower: float = -3.14159
    limit_upper: float = 3.14159
    limit_effort: float = 10.0
    limit_velocity: float = 1.0
    use_dynamics: bool = False
    dynamics_damping: float = 0.0
    dynamics_friction: float = 0.0

    def to_dict(self) -> dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> JointPreset:
        """Create from dictionary."""
        return cls(**data)


@dataclass
class MaterialPreset:
    """Preset for material/color configuration."""

    name: str
    description: str = ""
    color: list[float] = field(default_factory=lambda: [0.8, 0.8, 0.8, 1.0])  # RGBA
    material_name: str = ""

    def to_dict(self) -> dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> MaterialPreset:
        """Create from dictionary."""
        return cls(**data)


@dataclass
class SensorPreset:
    """Preset for sensor configuration."""

    name: str
    description: str = ""
    sensor_type: str = "CAMERA"
    update_rate: float = 30.0

    # Camera settings
    camera_horizontal_fov: float = 1.047
    camera_width: int = 640
    camera_height: int = 480
    camera_near_clip: float = 0.1
    camera_far_clip: float = 100.0

    # LIDAR settings
    lidar_horizontal_samples: int = 640
    lidar_horizontal_min_angle: float = -1.570796
    lidar_horizontal_max_angle: float = 1.570796
    lidar_vertical_samples: int = 1
    lidar_range_min: float = 0.1
    lidar_range_max: float = 10.0

    # IMU settings
    imu_gravity_magnitude: float = 9.80665

    # Noise settings
    use_noise: bool = False
    noise_type: str = "gaussian"
    noise_mean: float = 0.0
    noise_stddev: float = 0.0

    def to_dict(self) -> dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> SensorPreset:
        """Create from dictionary."""
        return cls(**data)


class PresetManager:
    """Manager for loading, saving, and accessing presets."""

    def __init__(self, presets_dir: Path | None = None):
        """Initialize preset manager.

        Args:
            presets_dir: Directory to store presets. If None, uses default location.
        """
        if presets_dir is None:
            # Default to user's config directory
            import sys

            if sys.platform == "darwin":
                base = Path.home() / "Library" / "Application Support"
            elif sys.platform == "win32":
                base = Path.home() / "AppData" / "Roaming"
            else:  # Linux and others
                base = Path.home() / ".config"

            presets_dir = base / "LinkForge" / "presets"

        self.presets_dir = Path(presets_dir) if not isinstance(presets_dir, Path) else presets_dir
        self.presets_dir.mkdir(parents=True, exist_ok=True)

        # Separate directories for each preset type
        self.joint_presets_dir = self.presets_dir / "joints"
        self.material_presets_dir = self.presets_dir / "materials"
        self.sensor_presets_dir = self.presets_dir / "sensors"

        self.joint_presets_dir.mkdir(exist_ok=True)
        self.material_presets_dir.mkdir(exist_ok=True)
        self.sensor_presets_dir.mkdir(exist_ok=True)

        # Initialize with default presets if directories are empty
        self._init_default_presets()

    def _init_default_presets(self) -> None:
        """Create default presets if none exist."""
        # Joint presets
        if not list(self.joint_presets_dir.glob("*.json")):
            self._create_default_joint_presets()

        # Material presets
        if not list(self.material_presets_dir.glob("*.json")):
            self._create_default_material_presets()

        # Sensor presets
        if not list(self.sensor_presets_dir.glob("*.json")):
            self._create_default_sensor_presets()

    def _create_default_joint_presets(self) -> None:
        """Create default joint presets."""
        defaults = [
            JointPreset(
                name="Revolute Joint (±180°)",
                description="Standard revolute joint with ±180° range",
                joint_type="REVOLUTE",
                limit_lower=-3.14159,
                limit_upper=3.14159,
                limit_effort=10.0,
                limit_velocity=1.0,
            ),
            JointPreset(
                name="Revolute Joint (±90°)",
                description="Limited revolute joint with ±90° range",
                joint_type="REVOLUTE",
                limit_lower=-1.5708,
                limit_upper=1.5708,
                limit_effort=10.0,
                limit_velocity=1.0,
            ),
            JointPreset(
                name="Continuous Joint (Wheel)",
                description="Continuous rotation joint for wheels",
                joint_type="CONTINUOUS",
                use_limits=False,
                limit_effort=20.0,
                limit_velocity=10.0,
            ),
            JointPreset(
                name="Prismatic Joint (0.5m)",
                description="Linear joint with 0.5m travel",
                joint_type="PRISMATIC",
                limit_lower=0.0,
                limit_upper=0.5,
                limit_effort=100.0,
                limit_velocity=0.5,
            ),
            JointPreset(
                name="Fixed Joint",
                description="No movement allowed",
                joint_type="FIXED",
                use_limits=False,
            ),
        ]

        for preset in defaults:
            self.save_joint_preset(preset)

    def _create_default_material_presets(self) -> None:
        """Create default material presets."""
        defaults = [
            MaterialPreset(
                name="Aluminum",
                description="Light gray metallic",
                color=[0.75, 0.75, 0.75, 1.0],
                material_name="aluminum",
            ),
            MaterialPreset(
                name="Steel",
                description="Dark gray metallic",
                color=[0.4, 0.4, 0.4, 1.0],
                material_name="steel",
            ),
            MaterialPreset(
                name="Black Plastic",
                description="Matte black",
                color=[0.1, 0.1, 0.1, 1.0],
                material_name="black_plastic",
            ),
            MaterialPreset(
                name="White Plastic",
                description="Matte white",
                color=[0.9, 0.9, 0.9, 1.0],
                material_name="white_plastic",
            ),
            MaterialPreset(
                name="Red",
                description="Bright red",
                color=[0.8, 0.1, 0.1, 1.0],
                material_name="red",
            ),
            MaterialPreset(
                name="Blue",
                description="Bright blue",
                color=[0.1, 0.3, 0.8, 1.0],
                material_name="blue",
            ),
            MaterialPreset(
                name="Yellow",
                description="Bright yellow",
                color=[0.9, 0.9, 0.1, 1.0],
                material_name="yellow",
            ),
        ]

        for preset in defaults:
            self.save_material_preset(preset)

    def _create_default_sensor_presets(self) -> None:
        """Create default sensor presets."""
        defaults = [
            SensorPreset(
                name="Standard Camera (640x480)",
                description="Standard camera sensor",
                sensor_type="CAMERA",
                update_rate=30.0,
                camera_horizontal_fov=1.047,  # 60 degrees
                camera_width=640,
                camera_height=480,
            ),
            SensorPreset(
                name="HD Camera (1920x1080)",
                description="High definition camera",
                sensor_type="CAMERA",
                update_rate=30.0,
                camera_horizontal_fov=1.047,
                camera_width=1920,
                camera_height=1080,
            ),
            SensorPreset(
                name="2D LIDAR (270°)",
                description="2D LIDAR with 270° scan",
                sensor_type="LIDAR",
                update_rate=10.0,
                lidar_horizontal_samples=640,
                lidar_horizontal_min_angle=-2.356,  # -135°
                lidar_horizontal_max_angle=2.356,  # +135°
                lidar_range_min=0.1,
                lidar_range_max=10.0,
            ),
            SensorPreset(
                name="3D LIDAR",
                description="3D LIDAR sensor",
                sensor_type="LIDAR",
                update_rate=10.0,
                lidar_horizontal_samples=1024,
                lidar_vertical_samples=64,
                lidar_range_min=0.1,
                lidar_range_max=100.0,
            ),
            SensorPreset(
                name="Standard IMU",
                description="IMU with standard gravity",
                sensor_type="IMU",
                update_rate=100.0,
                imu_gravity_magnitude=9.80665,
            ),
            SensorPreset(
                name="GPS",
                description="GPS sensor",
                sensor_type="GPS",
                update_rate=1.0,
            ),
        ]

        for preset in defaults:
            self.save_sensor_preset(preset)

    # Joint preset methods
    def save_joint_preset(self, preset: JointPreset) -> None:
        """Save a joint preset."""
        path = self.joint_presets_dir / f"{preset.name}.json"
        with open(path, "w") as f:
            json.dump(preset.to_dict(), f, indent=2)

    def load_joint_preset(self, name: str) -> JointPreset | None:
        """Load a joint preset by name."""
        path = self.joint_presets_dir / f"{name}.json"
        if not path.exists():
            return None

        with open(path) as f:
            data = json.load(f)
            return JointPreset.from_dict(data)

    def list_joint_presets(self) -> list[str]:
        """List all joint preset names."""
        return [p.stem for p in self.joint_presets_dir.glob("*.json")]

    def delete_joint_preset(self, name: str) -> bool:
        """Delete a joint preset."""
        path = self.joint_presets_dir / f"{name}.json"
        if path.exists():
            path.unlink()
            return True
        return False

    # Material preset methods
    def save_material_preset(self, preset: MaterialPreset) -> None:
        """Save a material preset."""
        path = self.material_presets_dir / f"{preset.name}.json"
        with open(path, "w") as f:
            json.dump(preset.to_dict(), f, indent=2)

    def load_material_preset(self, name: str) -> MaterialPreset | None:
        """Load a material preset by name."""
        path = self.material_presets_dir / f"{name}.json"
        if not path.exists():
            return None

        with open(path) as f:
            data = json.load(f)
            return MaterialPreset.from_dict(data)

    def list_material_presets(self) -> list[str]:
        """List all material preset names."""
        return [p.stem for p in self.material_presets_dir.glob("*.json")]

    def delete_material_preset(self, name: str) -> bool:
        """Delete a material preset."""
        path = self.material_presets_dir / f"{name}.json"
        if path.exists():
            path.unlink()
            return True
        return False

    # Sensor preset methods
    def save_sensor_preset(self, preset: SensorPreset) -> None:
        """Save a sensor preset."""
        path = self.sensor_presets_dir / f"{preset.name}.json"
        with open(path, "w") as f:
            json.dump(preset.to_dict(), f, indent=2)

    def load_sensor_preset(self, name: str) -> SensorPreset | None:
        """Load a sensor preset by name."""
        path = self.sensor_presets_dir / f"{name}.json"
        if not path.exists():
            return None

        with open(path) as f:
            data = json.load(f)
            return SensorPreset.from_dict(data)

    def list_sensor_presets(self) -> list[str]:
        """List all sensor preset names."""
        return [p.stem for p in self.sensor_presets_dir.glob("*.json")]

    def delete_sensor_preset(self, name: str) -> bool:
        """Delete a sensor preset."""
        path = self.sensor_presets_dir / f"{name}.json"
        if path.exists():
            path.unlink()
            return True
        return False
