"""Sensor models for URDF/Gazebo integration."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

from .geometry import Transform


class SensorType(str, Enum):
    """Supported sensor types in Gazebo/SDF."""

    CAMERA = "camera"
    DEPTH_CAMERA = "depth_camera"
    MULTICAMERA = "multicamera"
    LIDAR = "lidar"
    GPU_LIDAR = "gpu_lidar"
    IMU = "imu"
    MAGNETOMETER = "magnetometer"
    GPS = "gps"
    FORCE_TORQUE = "force_torque"
    CONTACT = "contact"
    SONAR = "sonar"
    RFID = "rfid"
    CUSTOM = "custom"


@dataclass(frozen=True)
class SensorNoise:
    """Noise model for sensor measurements."""

    type: str = "gaussian"  # gaussian, gaussian_quantized
    mean: float = 0.0
    stddev: float = 0.0
    bias_mean: float = 0.0
    bias_stddev: float = 0.0
    precision: float = 0.0  # For quantized noise
    dynamic_bias_stddev: float = 0.0
    dynamic_bias_correlation_time: float = 0.0


@dataclass(frozen=True)
class CameraInfo:
    """Camera-specific sensor information."""

    horizontal_fov: float = 1.047  # ~60 degrees in radians
    width: int = 640
    height: int = 480
    format: str = "R8G8B8"  # Pixel format
    near_clip: float = 0.1
    far_clip: float = 100.0

    # Distortion parameters (optional)
    distortion_k1: float = 0.0
    distortion_k2: float = 0.0
    distortion_k3: float = 0.0
    distortion_p1: float = 0.0
    distortion_p2: float = 0.0
    distortion_center_x: float = 0.5
    distortion_center_y: float = 0.5

    def __post_init__(self) -> None:
        """Validate camera parameters."""
        if self.horizontal_fov <= 0 or self.horizontal_fov >= 3.14159:
            raise ValueError(f"Horizontal FOV must be between 0 and π, got {self.horizontal_fov}")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("Image dimensions must be positive")
        if self.near_clip <= 0:
            raise ValueError("Near clip must be positive")
        if self.far_clip <= self.near_clip:
            raise ValueError("Far clip must be greater than near clip")


@dataclass(frozen=True)
class LidarInfo:
    """LIDAR/laser scanner sensor information."""

    # Horizontal scan parameters
    horizontal_samples: int = 640
    horizontal_resolution: float = 1.0
    horizontal_min_angle: float = -1.570796  # -π/2 radians (-90°)
    horizontal_max_angle: float = 1.570796  # π/2 radians (90°)

    # Vertical scan parameters (for 3D LIDAR)
    vertical_samples: int = 1
    vertical_resolution: float = 1.0
    vertical_min_angle: float = 0.0
    vertical_max_angle: float = 0.0

    # Range parameters
    range_min: float = 0.1
    range_max: float = 10.0
    range_resolution: float = 0.01

    # Noise
    noise: SensorNoise | None = None

    def __post_init__(self) -> None:
        """Validate LIDAR parameters."""
        if self.horizontal_samples <= 0:
            raise ValueError("Horizontal samples must be positive")
        if self.range_min <= 0:
            raise ValueError("Range min must be positive")
        if self.range_max <= self.range_min:
            raise ValueError("Range max must be greater than range min")
        if self.horizontal_min_angle >= self.horizontal_max_angle:
            raise ValueError("Horizontal min angle must be less than max angle")


@dataclass(frozen=True)
class IMUInfo:
    """IMU sensor information."""

    # Noise models for different measurements
    angular_velocity_noise: SensorNoise | None = None
    linear_acceleration_noise: SensorNoise | None = None
    orientation_noise: SensorNoise | None = None

    # Reference frames
    angular_velocity_frame: str = "child_frame"  # child_frame or world
    linear_acceleration_frame: str = "child_frame"

    # Gravity
    gravity_magnitude: float = 9.80665  # m/s^2 (standard gravity)


@dataclass(frozen=True)
class GPSInfo:
    """GPS sensor information."""

    # Position noise
    position_sensing_horizontal_noise: SensorNoise | None = None
    position_sensing_vertical_noise: SensorNoise | None = None

    # Velocity noise
    velocity_sensing_horizontal_noise: SensorNoise | None = None
    velocity_sensing_vertical_noise: SensorNoise | None = None


@dataclass(frozen=True)
class GazeboPlugin:
    """Gazebo plugin specification.

    Plugins extend Gazebo functionality for sensors, actuators, and other components.
    """

    name: str
    filename: str
    parameters: dict[str, str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        """Validate plugin."""
        if not self.name:
            raise ValueError("Plugin name cannot be empty")
        if not self.filename:
            raise ValueError("Plugin filename cannot be empty")


@dataclass(frozen=True)
class Sensor:
    """Generic sensor definition for Gazebo simulation.

    Sensors are attached to links and provide measurements in simulation.
    """

    name: str
    type: SensorType
    link_name: str  # Link this sensor is attached to
    update_rate: float = 30.0  # Hz
    always_on: bool = True
    visualize: bool = False

    # Sensor-specific information (only one should be set based on type)
    camera_info: CameraInfo | None = None
    lidar_info: LidarInfo | None = None
    imu_info: IMUInfo | None = None
    gps_info: GPSInfo | None = None

    # Transform relative to parent link
    origin: Transform = field(default_factory=Transform.identity)

    # Optional topic name for ROS
    topic: str | None = None

    # Plugin configuration
    plugin: GazeboPlugin | None = None

    def __post_init__(self) -> None:
        """Validate sensor configuration."""
        if not self.name:
            raise ValueError("Sensor name cannot be empty")
        if not self.link_name:
            raise ValueError("Sensor must be attached to a link")
        if self.update_rate <= 0:
            raise ValueError("Update rate must be positive")

        # Validate that appropriate info is set for sensor type
        if self.type in (SensorType.CAMERA, SensorType.DEPTH_CAMERA):
            if self.camera_info is None:
                raise ValueError(f"Camera sensor '{self.name}' requires camera_info")
        elif self.type in (SensorType.LIDAR, SensorType.GPU_LIDAR):
            if self.lidar_info is None:
                raise ValueError(f"LIDAR sensor '{self.name}' requires lidar_info")
        elif self.type == SensorType.IMU:
            if self.imu_info is None:
                raise ValueError(f"IMU sensor '{self.name}' requires imu_info")
        elif self.type == SensorType.GPS:
            if self.gps_info is None:
                raise ValueError(f"GPS sensor '{self.name}' requires gps_info")

    @property
    def is_camera(self) -> bool:
        """Check if this is a camera sensor."""
        return self.type in (SensorType.CAMERA, SensorType.DEPTH_CAMERA, SensorType.MULTICAMERA)

    @property
    def is_lidar(self) -> bool:
        """Check if this is a LIDAR sensor."""
        return self.type in (SensorType.LIDAR, SensorType.GPU_LIDAR)
