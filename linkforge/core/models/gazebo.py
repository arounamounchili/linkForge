"""Gazebo-specific URDF extensions and elements."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class GazeboElement:
    """Generic Gazebo element that can be applied to robot, link, or joint.

    The <gazebo> tag in URDF allows specification of Gazebo-specific properties
    that are not part of the standard URDF specification.
    """

    reference: str | None = None  # Link or joint name (None for robot-level)
    properties: dict[str, str] = field(default_factory=dict)
    plugins: list[GazeboPlugin] = field(default_factory=list)

    # Common properties for links
    material: str | None = None  # Gazebo material (e.g., "Gazebo/Red")
    self_collide: bool | None = None
    static: bool | None = None
    gravity: bool | None = None

    # Common properties for joints
    stop_cfm: float | None = None  # Constraint force mixing for joint stops
    stop_erp: float | None = None  # Error reduction parameter for joint stops
    provide_feedback: bool | None = None  # Enable force-torque feedback
    implicit_spring_damper: bool | None = None

    # Friction parameters
    mu1: float | None = None  # Friction coefficient in first friction direction
    mu2: float | None = None  # Friction coefficient in second friction direction
    kp: float | None = None  # Contact stiffness
    kd: float | None = None  # Contact damping

    def __post_init__(self) -> None:
        """Validate Gazebo element."""
        # If reference is specified, it must be non-empty
        if self.reference is not None and not self.reference:
            raise ValueError("Gazebo reference cannot be empty string")


@dataclass(frozen=True)
class GazeboPlugin:
    """Gazebo plugin specification.

    Plugins can be applied at robot, link, or joint level to extend Gazebo functionality.
    """

    name: str
    filename: str
    parameters: dict[str, str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        """Validate plugin configuration."""
        if not self.name:
            raise ValueError("Plugin name cannot be empty")
        if not self.filename:
            raise ValueError("Plugin filename cannot be empty")

    def get_parameter(self, key: str, default: str | None = None) -> str | None:
        """Get plugin parameter value."""
        return self.parameters.get(key, default)

    def with_parameter(self, key: str, value: str) -> GazeboPlugin:
        """Create a new plugin with an additional parameter."""
        new_params = dict(self.parameters)
        new_params[key] = value
        return GazeboPlugin(name=self.name, filename=self.filename, parameters=new_params)


# Common Gazebo plugins factory functions


def create_differential_drive_plugin(
    name: str = "differential_drive_controller",
    left_joint: str = "left_wheel_joint",
    right_joint: str = "right_wheel_joint",
    wheel_separation: float = 0.5,
    wheel_diameter: float = 0.2,
    command_topic: str = "cmd_vel",
    odometry_topic: str = "odom",
    odometry_frame: str = "odom",
    robot_base_frame: str = "base_link",
) -> GazeboPlugin:
    """Create a differential drive controller plugin for wheeled robots.

    Args:
        name: Plugin instance name
        left_joint: Name of left wheel joint
        right_joint: Name of right wheel joint
        wheel_separation: Distance between wheels (meters)
        wheel_diameter: Wheel diameter (meters)
        command_topic: Topic for velocity commands
        odometry_topic: Topic for odometry output
        odometry_frame: Frame ID for odometry
        robot_base_frame: Robot base frame ID

    Returns:
        Configured differential drive plugin
    """
    return GazeboPlugin(
        name=name,
        filename="libgazebo_ros_diff_drive.so",
        parameters={
            "left_joint": left_joint,
            "right_joint": right_joint,
            "wheel_separation": str(wheel_separation),
            "wheel_diameter": str(wheel_diameter),
            "max_wheel_torque": "20",
            "max_wheel_acceleration": "1.0",
            "command_topic": command_topic,
            "odometry_topic": odometry_topic,
            "odometry_frame": odometry_frame,
            "robot_base_frame": robot_base_frame,
            "publish_odom": "true",
            "publish_odom_tf": "true",
            "publish_wheel_tf": "false",
        },
    )


def create_joint_state_publisher_plugin(
    name: str = "joint_state_publisher",
    update_rate: float = 50.0,
) -> GazeboPlugin:
    """Create a joint state publisher plugin.

    Args:
        name: Plugin instance name
        update_rate: Publishing rate (Hz)

    Returns:
        Configured joint state publisher plugin
    """
    return GazeboPlugin(
        name=name,
        filename="libgazebo_ros_joint_state_publisher.so",
        parameters={
            "update_rate": str(update_rate),
            "joint_name": "",  # Empty means publish all joints
        },
    )


def create_camera_plugin(
    name: str,
    camera_name: str,
    image_topic: str = "camera/image_raw",
    camera_info_topic: str = "camera/camera_info",
    frame_name: str = "camera_link",
    update_rate: float = 30.0,
) -> GazeboPlugin:
    """Create a camera sensor plugin.

    Args:
        name: Plugin instance name
        camera_name: Camera sensor name
        image_topic: Topic for image output
        camera_info_topic: Topic for camera info
        frame_name: Frame ID for camera
        update_rate: Publishing rate (Hz)

    Returns:
        Configured camera plugin
    """
    return GazeboPlugin(
        name=name,
        filename="libgazebo_ros_camera.so",
        parameters={
            "camera_name": camera_name,
            "image_topic_name": image_topic,
            "camera_info_topic_name": camera_info_topic,
            "frame_name": frame_name,
            "update_rate": str(update_rate),
            "hack_baseline": "0.0",
        },
    )


def create_imu_plugin(
    name: str,
    topic: str = "imu",
    frame_name: str = "imu_link",
    update_rate: float = 100.0,
) -> GazeboPlugin:
    """Create an IMU sensor plugin.

    Args:
        name: Plugin instance name
        topic: Topic for IMU data
        frame_name: Frame ID for IMU
        update_rate: Publishing rate (Hz)

    Returns:
        Configured IMU plugin
    """
    return GazeboPlugin(
        name=name,
        filename="libgazebo_ros_imu_sensor.so",
        parameters={
            "topicName": topic,
            "frameName": frame_name,
            "update_rate": str(update_rate),
            "initial_orientation_as_reference": "false",
        },
    )


def create_lidar_plugin(
    name: str,
    topic: str = "scan",
    frame_name: str = "lidar_link",
) -> GazeboPlugin:
    """Create a LIDAR/laser scanner plugin.

    Args:
        name: Plugin instance name
        topic: Topic for laser scan data
        frame_name: Frame ID for LIDAR

    Returns:
        Configured LIDAR plugin
    """
    return GazeboPlugin(
        name=name,
        filename="libgazebo_ros_ray_sensor.so",
        parameters={
            "topicName": topic,
            "frameName": frame_name,
        },
    )
