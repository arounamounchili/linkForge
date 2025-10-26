"""Parallel Gripper template.

A simple parallel gripper with 2 fingers synchronized using mimic joints.
Demonstrates coupled joint motion commonly used in grippers.
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
    JointMimic,
    JointType,
    Link,
    Material,
    Robot,
    Transform,
    Vector3,
    Visual,
)
from ..template import RobotTemplate, TemplateCategory


def create_parallel_gripper() -> Robot:
    """Create a parallel gripper with mimic joints.

    Returns:
        Robot instance with palm and 2 synchronized fingers
    """
    # Palm/base link
    palm_visual = Visual(
        name="palm_visual",
        geometry=Box(size=Vector3(0.1, 0.1, 0.05)),
        material=Material(name="gray", color=Colors.GRAY),
    )
    palm_collision = Collision(
        name="palm_collision",
        geometry=Box(size=Vector3(0.1, 0.1, 0.05)),
    )
    palm_inertial = Inertial(
        mass=0.3,
        inertia=InertiaTensor(ixx=0.0001, ixy=0.0, ixz=0.0, iyy=0.0001, iyz=0.0, izz=0.0001),
    )
    palm_link = Link(
        name="palm",
        visual=palm_visual,
        collision=palm_collision,
        inertial=palm_inertial,
    )

    # Left finger
    left_finger_visual = Visual(
        name="left_finger_visual",
        geometry=Box(size=Vector3(0.02, 0.02, 0.1)),
        origin=Transform(xyz=Vector3(0.0, 0.0, 0.05)),  # Center of finger
        material=Material(name="blue", color=Colors.BLUE),
    )
    left_finger_collision = Collision(
        name="left_finger_collision",
        geometry=Box(size=Vector3(0.02, 0.02, 0.1)),
        origin=Transform(xyz=Vector3(0.0, 0.0, 0.05)),
    )
    left_finger_inertial = Inertial(
        mass=0.05,
        origin=Transform(xyz=Vector3(0.0, 0.0, 0.05)),
        inertia=InertiaTensor(ixx=0.00004, ixy=0.0, ixz=0.0, iyy=0.00004, iyz=0.0, izz=0.000002),
    )
    left_finger_link = Link(
        name="left_finger",
        visual=left_finger_visual,
        collision=left_finger_collision,
        inertial=left_finger_inertial,
    )

    # Right finger (symmetric to left)
    right_finger_visual = Visual(
        name="right_finger_visual",
        geometry=Box(size=Vector3(0.02, 0.02, 0.1)),
        origin=Transform(xyz=Vector3(0.0, 0.0, 0.05)),
        material=Material(name="blue", color=Colors.BLUE),
    )
    right_finger_collision = Collision(
        name="right_finger_collision",
        geometry=Box(size=Vector3(0.02, 0.02, 0.1)),
        origin=Transform(xyz=Vector3(0.0, 0.0, 0.05)),
    )
    right_finger_inertial = Inertial(
        mass=0.05,
        origin=Transform(xyz=Vector3(0.0, 0.0, 0.05)),
        inertia=InertiaTensor(ixx=0.00004, ixy=0.0, ixz=0.0, iyy=0.00004, iyz=0.0, izz=0.000002),
    )
    right_finger_link = Link(
        name="right_finger",
        visual=right_finger_visual,
        collision=right_finger_collision,
        inertial=right_finger_inertial,
    )

    # Left finger joint (primary - actuated)
    left_finger_joint = Joint(
        name="left_finger_joint",
        type=JointType.PRISMATIC,
        parent="palm",
        child="left_finger",
        origin=Transform(xyz=Vector3(0.0, 0.04, 0.025)),  # Left side of palm
        axis=Vector3(0.0, 1.0, 0.0),  # Slide along Y axis (outward)
        limits=JointLimits(
            lower=0.0,  # Closed
            upper=0.04,  # Open 4cm
            effort=10.0,
            velocity=0.1,
        ),
    )

    # Right finger joint (mimic - follows left finger)
    right_finger_joint = Joint(
        name="right_finger_joint",
        type=JointType.PRISMATIC,
        parent="palm",
        child="right_finger",
        origin=Transform(xyz=Vector3(0.0, -0.04, 0.025)),  # Right side of palm
        axis=Vector3(0.0, -1.0, 0.0),  # Slide along -Y axis (outward, mirror)
        limits=JointLimits(
            lower=0.0,
            upper=0.04,
            effort=10.0,
            velocity=0.1,
        ),
        mimic=JointMimic(
            joint="left_finger_joint",
            multiplier=1.0,  # Same distance
            offset=0.0,
        ),
    )

    # Create robot
    robot = Robot(
        name="gripper_parallel",
        links=[palm_link, left_finger_link, right_finger_link],
        joints=[left_finger_joint, right_finger_joint],
    )

    return robot


# Create the template
GRIPPER_PARALLEL_TEMPLATE = RobotTemplate(
    id="gripper_parallel",
    name="Parallel Gripper",
    description="Simple parallel gripper with 2 fingers synchronized using mimic joints. "
    "Demonstrates coupled joint motion commonly used in grippers.",
    category=TemplateCategory.GRIPPER,
    factory=create_parallel_gripper,
    tags=["gripper", "parallel", "mimic", "end_effector"],
)
