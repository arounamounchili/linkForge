"""2-DOF Robot Arm template.

A simple educational robot arm with 2 revolute joints,
perfect for learning URDF concepts and testing.
"""

from __future__ import annotations

from ...core.models import (
    Box,
    Collision,
    Colors,
    Inertial,
    InertiaTensor,
    Joint,
    JointLimits,
    JointType,
    Link,
    Material,
    Robot,
    Transform,
    Vector3,
    Visual,
)
from ..template import RobotTemplate, TemplateCategory


def create_2dof_arm() -> Robot:
    """Create a 2-DOF robot arm.

    Returns:
        Robot instance with base, 2 arm links, and 2 revolute joints
    """
    # Base link (fixed to ground)
    base_visual = Visual(
        name="base_visual",
        geometry=Box(size=Vector3(0.2, 0.2, 0.1)),
        material=Material(name="gray", color=Colors.GRAY),
    )
    base_collision = Collision(
        name="base_collision",
        geometry=Box(size=Vector3(0.2, 0.2, 0.1)),
    )
    base_inertial = Inertial(
        mass=2.0,
        inertia=InertiaTensor(ixx=0.0067, ixy=0.0, ixz=0.0, iyy=0.0067, iyz=0.0, izz=0.0117),
    )
    base_link = Link(
        name="base_link",
        visual=base_visual,
        collision=base_collision,
        inertial=base_inertial,
    )

    # Link 1 (first arm segment)
    link1_visual = Visual(
        name="link1_visual",
        geometry=Box(size=Vector3(0.08, 0.08, 0.4)),
        origin=Transform(xyz=Vector3(0.0, 0.0, 0.2)),  # Center of mass offset
        material=Material(name="blue", color=Colors.BLUE),
    )
    link1_collision = Collision(
        name="link1_collision",
        geometry=Box(size=Vector3(0.08, 0.08, 0.4)),
        origin=Transform(xyz=Vector3(0.0, 0.0, 0.2)),
    )
    link1_inertial = Inertial(
        mass=0.5,
        origin=Transform(xyz=Vector3(0.0, 0.0, 0.2)),
        inertia=InertiaTensor(ixx=0.0067, ixy=0.0, ixz=0.0, iyy=0.0067, iyz=0.0, izz=0.0003),
    )
    link1 = Link(
        name="link1",
        visual=link1_visual,
        collision=link1_collision,
        inertial=link1_inertial,
    )

    # Link 2 (second arm segment)
    link2_visual = Visual(
        name="link2_visual",
        geometry=Box(size=Vector3(0.06, 0.06, 0.3)),
        origin=Transform(xyz=Vector3(0.0, 0.0, 0.15)),
        material=Material(name="green", color=Colors.GREEN),
    )
    link2_collision = Collision(
        name="link2_collision",
        geometry=Box(size=Vector3(0.06, 0.06, 0.3)),
        origin=Transform(xyz=Vector3(0.0, 0.0, 0.15)),
    )
    link2_inertial = Inertial(
        mass=0.3,
        origin=Transform(xyz=Vector3(0.0, 0.0, 0.15)),
        inertia=InertiaTensor(ixx=0.0023, ixy=0.0, ixz=0.0, iyy=0.0023, iyz=0.0, izz=0.00009),
    )
    link2 = Link(
        name="link2",
        visual=link2_visual,
        collision=link2_collision,
        inertial=link2_inertial,
    )

    # Joint 1 (base to link1)
    joint1 = Joint(
        name="joint1",
        type=JointType.REVOLUTE,
        parent="base_link",
        child="link1",
        origin=Transform(xyz=Vector3(0.0, 0.0, 0.1)),  # Top of base
        axis=Vector3(0.0, 1.0, 0.0),  # Rotate around Y axis
        limits=JointLimits(
            lower=-1.57,  # -90 degrees
            upper=1.57,  # +90 degrees
            effort=10.0,
            velocity=1.0,
        ),
    )

    # Joint 2 (link1 to link2)
    joint2 = Joint(
        name="joint2",
        type=JointType.REVOLUTE,
        parent="link1",
        child="link2",
        origin=Transform(xyz=Vector3(0.0, 0.0, 0.4)),  # Top of link1
        axis=Vector3(0.0, 1.0, 0.0),  # Rotate around Y axis
        limits=JointLimits(
            lower=-2.0,  # ~-115 degrees
            upper=2.0,  # ~+115 degrees
            effort=5.0,
            velocity=1.0,
        ),
    )

    # Create robot
    robot = Robot(
        name="arm_2dof",
        links=[base_link, link1, link2],
        joints=[joint1, joint2],
    )

    return robot


# Create the template
ARM_2DOF_TEMPLATE = RobotTemplate(
    id="arm_2dof",
    name="2-DOF Robot Arm",
    description="Simple educational robot arm with 2 revolute joints. "
    "Perfect for learning URDF concepts and testing kinematics.",
    category=TemplateCategory.ARM,
    factory=create_2dof_arm,
    tags=["educational", "simple", "arm", "2dof"],
)
