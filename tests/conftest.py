"""Pytest configuration and shared fixtures."""

from __future__ import annotations

import math

import pytest

from linkforge.core.models import (
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
    Vector3,
    Visual,
)


@pytest.fixture
def simple_box() -> Box:
    """Create a simple box geometry."""
    return Box(size=Vector3(1.0, 1.0, 1.0))


@pytest.fixture
def simple_material() -> Material:
    """Create a simple material."""
    return Material(name="gray", color=Colors.GRAY)


@pytest.fixture
def simple_inertia() -> InertiaTensor:
    """Create a simple inertia tensor."""
    return InertiaTensor(
        ixx=1.0,
        ixy=0.0,
        ixz=0.0,
        iyy=1.0,
        iyz=0.0,
        izz=1.0,
    )


@pytest.fixture
def simple_link(simple_box: Box, simple_material: Material, simple_inertia: InertiaTensor) -> Link:
    """Create a simple link."""
    visual = Visual(geometry=simple_box, material=simple_material)
    collision = Collision(geometry=simple_box)
    inertial = Inertial(mass=1.0, inertia=simple_inertia)

    return Link(
        name="test_link",
        visual=visual,
        collision=collision,
        inertial=inertial,
    )


@pytest.fixture
def simple_joint() -> Joint:
    """Create a simple revolute joint."""
    return Joint(
        name="test_joint",
        type=JointType.REVOLUTE,
        parent="link1",
        child="link2",
        limits=JointLimits(lower=-math.pi, upper=math.pi, effort=10.0, velocity=1.0),
    )


@pytest.fixture
def simple_robot(simple_link: Link, simple_joint: Joint) -> Robot:
    """Create a simple robot with two links and one joint."""
    robot = Robot(name="test_robot")

    # Create two links
    link1 = Link(name="link1", visual=simple_link.visual, inertial=simple_link.inertial)
    link2 = Link(name="link2", visual=simple_link.visual, inertial=simple_link.inertial)

    robot.add_link(link1)
    robot.add_link(link2)

    # Create joint
    joint = Joint(
        name="joint1",
        type=JointType.REVOLUTE,
        parent="link1",
        child="link2",
        limits=JointLimits(lower=-math.pi, upper=math.pi, effort=10.0, velocity=1.0),
    )
    robot.add_joint(joint)

    return robot
