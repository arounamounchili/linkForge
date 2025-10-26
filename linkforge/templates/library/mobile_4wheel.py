"""4-Wheel Mobile Base template.

A differential drive mobile robot with 4 wheels (2 driven, 2 casters),
commonly used for mobile robotics research and education.
"""

from __future__ import annotations

from ...core.models import (
    Box,
    Collision,
    Colors,
    Cylinder,
    Inertial,
    InertiaTensor,
    Joint,
    JointType,
    Link,
    Material,
    Robot,
    Sphere,
    Transform,
    Vector3,
    Visual,
)
from ..template import RobotTemplate, TemplateCategory


def create_4wheel_mobile_base() -> Robot:
    """Create a 4-wheel differential drive mobile base.

    Returns:
        Robot instance with chassis and 4 wheels (2 driven + 2 casters)
    """
    # Base chassis link
    chassis_visual = Visual(
        name="chassis_visual",
        geometry=Box(size=Vector3(0.6, 0.4, 0.2)),
        material=Material(name="red", color=Colors.RED),
    )
    chassis_collision = Collision(
        name="chassis_collision",
        geometry=Box(size=Vector3(0.6, 0.4, 0.2)),
    )
    chassis_inertial = Inertial(
        mass=10.0,
        inertia=InertiaTensor(ixx=0.183, ixy=0.0, ixz=0.0, iyy=0.383, iyz=0.0, izz=0.533),
    )
    chassis_link = Link(
        name="base_link",
        visual=chassis_visual,
        collision=chassis_collision,
        inertial=chassis_inertial,
    )

    # Left drive wheel
    left_wheel_visual = Visual(
        name="left_wheel_visual",
        geometry=Cylinder(radius=0.1, length=0.05),
        origin=Transform(rpy=Vector3(1.5708, 0.0, 0.0)),  # Rotate to horizontal
        material=Material(name="dark_gray", color=Colors.RUBBER),
    )
    left_wheel_collision = Collision(
        name="left_wheel_collision",
        geometry=Cylinder(radius=0.1, length=0.05),
        origin=Transform(rpy=Vector3(1.5708, 0.0, 0.0)),
    )
    left_wheel_inertial = Inertial(
        mass=0.5,
        inertia=InertiaTensor(ixx=0.00135, ixy=0.0, ixz=0.0, iyy=0.0025, iyz=0.0, izz=0.00135),
    )
    left_wheel_link = Link(
        name="left_wheel",
        visual=left_wheel_visual,
        collision=left_wheel_collision,
        inertial=left_wheel_inertial,
    )

    # Right drive wheel
    right_wheel_visual = Visual(
        name="right_wheel_visual",
        geometry=Cylinder(radius=0.1, length=0.05),
        origin=Transform(rpy=Vector3(1.5708, 0.0, 0.0)),
        material=Material(name="dark_gray", color=Colors.RUBBER),
    )
    right_wheel_collision = Collision(
        name="right_wheel_collision",
        geometry=Cylinder(radius=0.1, length=0.05),
        origin=Transform(rpy=Vector3(1.5708, 0.0, 0.0)),
    )
    right_wheel_inertial = Inertial(
        mass=0.5,
        inertia=InertiaTensor(ixx=0.00135, ixy=0.0, ixz=0.0, iyy=0.0025, iyz=0.0, izz=0.00135),
    )
    right_wheel_link = Link(
        name="right_wheel",
        visual=right_wheel_visual,
        collision=right_wheel_collision,
        inertial=right_wheel_inertial,
    )

    # Front caster wheel (simplified as sphere)
    front_caster_visual = Visual(
        name="front_caster_visual",
        geometry=Sphere(radius=0.05),
        material=Material(name="gray", color=Colors.GRAY),
    )
    front_caster_collision = Collision(
        name="front_caster_collision",
        geometry=Sphere(radius=0.05),
    )
    front_caster_inertial = Inertial(
        mass=0.2,
        inertia=InertiaTensor(ixx=0.0001, ixy=0.0, ixz=0.0, iyy=0.0001, iyz=0.0, izz=0.0001),
    )
    front_caster_link = Link(
        name="front_caster",
        visual=front_caster_visual,
        collision=front_caster_collision,
        inertial=front_caster_inertial,
    )

    # Rear caster wheel (simplified as sphere)
    rear_caster_visual = Visual(
        name="rear_caster_visual",
        geometry=Sphere(radius=0.05),
        material=Material(name="gray", color=Colors.GRAY),
    )
    rear_caster_collision = Collision(
        name="rear_caster_collision",
        geometry=Sphere(radius=0.05),
    )
    rear_caster_inertial = Inertial(
        mass=0.2,
        inertia=InertiaTensor(ixx=0.0001, ixy=0.0, ixz=0.0, iyy=0.0001, iyz=0.0, izz=0.0001),
    )
    rear_caster_link = Link(
        name="rear_caster",
        visual=rear_caster_visual,
        collision=rear_caster_collision,
        inertial=rear_caster_inertial,
    )

    # Left wheel joint (continuous rotation)
    left_wheel_joint = Joint(
        name="left_wheel_joint",
        type=JointType.CONTINUOUS,
        parent="base_link",
        child="left_wheel",
        origin=Transform(xyz=Vector3(0.0, 0.225, 0.0)),  # Left side
        axis=Vector3(0.0, 1.0, 0.0),  # Rotate around Y axis
    )

    # Right wheel joint (continuous rotation)
    right_wheel_joint = Joint(
        name="right_wheel_joint",
        type=JointType.CONTINUOUS,
        parent="base_link",
        child="right_wheel",
        origin=Transform(xyz=Vector3(0.0, -0.225, 0.0)),  # Right side
        axis=Vector3(0.0, 1.0, 0.0),  # Rotate around Y axis
    )

    # Front caster joint (fixed for simplicity)
    front_caster_joint = Joint(
        name="front_caster_joint",
        type=JointType.FIXED,
        parent="base_link",
        child="front_caster",
        origin=Transform(xyz=Vector3(0.25, 0.0, -0.15)),  # Front center
    )

    # Rear caster joint (fixed for simplicity)
    rear_caster_joint = Joint(
        name="rear_caster_joint",
        type=JointType.FIXED,
        parent="base_link",
        child="rear_caster",
        origin=Transform(xyz=Vector3(-0.25, 0.0, -0.15)),  # Rear center
    )

    # Create robot
    robot = Robot(
        name="mobile_4wheel",
        links=[
            chassis_link,
            left_wheel_link,
            right_wheel_link,
            front_caster_link,
            rear_caster_link,
        ],
        joints=[
            left_wheel_joint,
            right_wheel_joint,
            front_caster_joint,
            rear_caster_joint,
        ],
    )

    return robot


# Create the template
MOBILE_4WHEEL_TEMPLATE = RobotTemplate(
    id="mobile_4wheel",
    name="4-Wheel Mobile Base",
    description="Differential drive mobile robot with 4 wheels (2 driven, 2 casters). "
    "Common configuration for mobile robotics platforms.",
    category=TemplateCategory.MOBILE,
    factory=create_4wheel_mobile_base,
    tags=["mobile", "differential_drive", "wheeled", "4wheel"],
)
