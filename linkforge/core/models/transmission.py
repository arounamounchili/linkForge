"""Transmission models for ros2_control integration."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class TransmissionType(str, Enum):
    """Standard transmission types in ros2_control."""

    SIMPLE = "transmission_interface/SimpleTransmission"
    DIFFERENTIAL = "transmission_interface/DifferentialTransmission"
    FOUR_BAR_LINKAGE = "transmission_interface/FourBarLinkageTransmission"
    CUSTOM = "custom"


class HardwareInterface(str, Enum):
    """Standard hardware interface types in ros2_control."""

    POSITION = "hardware_interface/PositionJointInterface"
    VELOCITY = "hardware_interface/VelocityJointInterface"
    EFFORT = "hardware_interface/EffortJointInterface"
    POSITION_VELOCITY = "hardware_interface/PositionVelocityJointInterface"

    # ROS2 control interfaces
    COMMAND_POSITION = "position"
    COMMAND_VELOCITY = "velocity"
    COMMAND_EFFORT = "effort"
    STATE_POSITION = "position"
    STATE_VELOCITY = "velocity"
    STATE_EFFORT = "effort"


@dataclass(frozen=True)
class TransmissionJoint:
    """Joint specification in a transmission.

    Defines how a joint is connected in the transmission with its hardware interface.
    """

    name: str
    hardware_interfaces: list[str] = field(default_factory=lambda: ["position"])
    mechanical_reduction: float = 1.0
    offset: float = 0.0

    def __post_init__(self) -> None:
        """Validate transmission joint."""
        if not self.name:
            raise ValueError("Transmission joint name cannot be empty")
        if not self.hardware_interfaces:
            raise ValueError(f"Joint '{self.name}' must have at least one hardware interface")
        if self.mechanical_reduction == 0:
            raise ValueError(f"Joint '{self.name}' mechanical reduction cannot be zero")


@dataclass(frozen=True)
class TransmissionActuator:
    """Actuator specification in a transmission.

    Defines the actuator properties and its connection to the transmission.
    """

    name: str
    hardware_interfaces: list[str] = field(default_factory=lambda: ["position"])
    mechanical_reduction: float = 1.0
    offset: float = 0.0

    def __post_init__(self) -> None:
        """Validate transmission actuator."""
        if not self.name:
            raise ValueError("Transmission actuator name cannot be empty")
        if not self.hardware_interfaces:
            raise ValueError(f"Actuator '{self.name}' must have at least one hardware interface")
        if self.mechanical_reduction == 0:
            raise ValueError(f"Actuator '{self.name}' mechanical reduction cannot be zero")


@dataclass(frozen=True)
class Transmission:
    """Transmission definition for mapping between joints and actuators.

    Transmissions define the relationship between joints and actuators, handling
    mechanical reduction and other transformations. Used by ros_control/ros2_control.
    """

    name: str
    type: str  # Plugin name (e.g., TransmissionType enum or custom)
    joints: list[TransmissionJoint] = field(default_factory=list)
    actuators: list[TransmissionActuator] = field(default_factory=list)

    # Additional parameters for complex transmissions
    parameters: dict[str, str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        """Validate transmission configuration."""
        if not self.name:
            raise ValueError("Transmission name cannot be empty")
        if not self.type:
            raise ValueError("Transmission type cannot be empty")

        # Validate naming convention
        if not all(c.isalnum() or c in ("_", "-") for c in self.name):
            raise ValueError(
                f"Transmission name '{self.name}' contains invalid characters. "
                "Use only alphanumeric, underscore, or hyphen."
            )

        # Must have at least one joint
        if not self.joints:
            raise ValueError(f"Transmission '{self.name}' must have at least one joint")

        # Check for duplicate joint names
        joint_names = [j.name for j in self.joints]
        if len(joint_names) != len(set(joint_names)):
            duplicates = [name for name in joint_names if joint_names.count(name) > 1]
            raise ValueError(
                f"Transmission '{self.name}' has duplicate joint names: {set(duplicates)}"
            )

        # Check for duplicate actuator names
        if self.actuators:
            actuator_names = [a.name for a in self.actuators]
            if len(actuator_names) != len(set(actuator_names)):
                duplicates = [name for name in actuator_names if actuator_names.count(name) > 1]
                raise ValueError(
                    f"Transmission '{self.name}' has duplicate actuator names: {set(duplicates)}"
                )

    @classmethod
    def create_simple(
        cls,
        name: str,
        joint_name: str,
        actuator_name: str | None = None,
        mechanical_reduction: float = 1.0,
        hardware_interface: str = "position",
    ) -> Transmission:
        """Create a simple 1-to-1 transmission.

        Args:
            name: Transmission name
            joint_name: Name of the joint
            actuator_name: Name of the actuator (defaults to joint_name + "_motor")
            mechanical_reduction: Gear ratio (default 1.0)
            hardware_interface: Interface type (default "position")

        Returns:
            Configured simple transmission
        """
        if actuator_name is None:
            actuator_name = f"{joint_name}_motor"

        return cls(
            name=name,
            type=TransmissionType.SIMPLE.value,
            joints=[
                TransmissionJoint(
                    name=joint_name,
                    hardware_interfaces=[hardware_interface],
                    mechanical_reduction=mechanical_reduction,
                )
            ],
            actuators=[
                TransmissionActuator(
                    name=actuator_name,
                    hardware_interfaces=[hardware_interface],
                    mechanical_reduction=1.0,
                )
            ],
        )

    @classmethod
    def create_differential(
        cls,
        name: str,
        joint1_name: str,
        joint2_name: str,
        actuator1_name: str | None = None,
        actuator2_name: str | None = None,
        mechanical_reduction: float = 1.0,
        hardware_interface: str = "position",
    ) -> Transmission:
        """Create a differential transmission (2 actuators, 2 joints).

        Args:
            name: Transmission name
            joint1_name: First joint name
            joint2_name: Second joint name
            actuator1_name: First actuator name (defaults to joint1_name + "_motor")
            actuator2_name: Second actuator name (defaults to joint2_name + "_motor")
            mechanical_reduction: Gear ratio (default 1.0)
            hardware_interface: Interface type (default "position")

        Returns:
            Configured differential transmission
        """
        if actuator1_name is None:
            actuator1_name = f"{joint1_name}_motor"
        if actuator2_name is None:
            actuator2_name = f"{joint2_name}_motor"

        return cls(
            name=name,
            type=TransmissionType.DIFFERENTIAL.value,
            joints=[
                TransmissionJoint(
                    name=joint1_name,
                    hardware_interfaces=[hardware_interface],
                    mechanical_reduction=mechanical_reduction,
                ),
                TransmissionJoint(
                    name=joint2_name,
                    hardware_interfaces=[hardware_interface],
                    mechanical_reduction=mechanical_reduction,
                ),
            ],
            actuators=[
                TransmissionActuator(
                    name=actuator1_name,
                    hardware_interfaces=[hardware_interface],
                    mechanical_reduction=1.0,
                ),
                TransmissionActuator(
                    name=actuator2_name,
                    hardware_interfaces=[hardware_interface],
                    mechanical_reduction=1.0,
                ),
            ],
        )

    @property
    def is_simple(self) -> bool:
        """Check if this is a simple transmission."""
        return self.type == TransmissionType.SIMPLE.value and len(self.joints) == 1

    @property
    def is_differential(self) -> bool:
        """Check if this is a differential transmission."""
        return self.type == TransmissionType.DIFFERENTIAL.value and len(self.joints) == 2
