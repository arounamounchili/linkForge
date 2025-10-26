"""Tests for Robot model."""

from __future__ import annotations

import math

import pytest

from linkforge.core.models import (
    Box,
    Collision,
    Inertial,
    InertiaTensor,
    Joint,
    JointLimits,
    JointType,
    Link,
    Robot,
    Vector3,
    Visual,
)


class TestRobot:
    """Tests for Robot model."""

    def test_creation(self):
        """Test creating a robot."""
        robot = Robot(name="test_robot")
        assert robot.name == "test_robot"
        assert len(robot.links) == 0
        assert len(robot.joints) == 0

    def test_invalid_name(self):
        """Test that invalid names raise error."""
        with pytest.raises(ValueError, match="invalid characters"):
            Robot(name="robot with spaces!")

    def test_add_link(self):
        """Test adding a link."""
        robot = Robot(name="test_robot")
        link = Link(name="link1")
        robot.add_link(link)

        assert len(robot.links) == 1
        assert robot.get_link("link1") == link

    def test_add_duplicate_link(self):
        """Test that adding duplicate link raises error."""
        robot = Robot(name="test_robot")
        link = Link(name="link1")
        robot.add_link(link)

        with pytest.raises(ValueError, match="already exists"):
            robot.add_link(link)

    def test_add_joint(self):
        """Test adding a joint."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))

        joint = Joint(
            name="joint1",
            type=JointType.FIXED,
            parent="link1",
            child="link2",
        )
        robot.add_joint(joint)

        assert len(robot.joints) == 1
        assert robot.get_joint("joint1") == joint

    def test_add_joint_missing_parent(self):
        """Test that adding joint with missing parent raises error."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link2"))

        joint = Joint(
            name="joint1",
            type=JointType.FIXED,
            parent="link1",  # Does not exist
            child="link2",
        )

        with pytest.raises(ValueError, match="not found"):
            robot.add_joint(joint)

    def test_get_root_link(self):
        """Test getting root link."""
        robot = Robot(name="test_robot")
        link1 = Link(name="link1")
        link2 = Link(name="link2")

        robot.add_link(link1)
        robot.add_link(link2)

        joint = Joint(
            name="joint1",
            type=JointType.FIXED,
            parent="link1",
            child="link2",
        )
        robot.add_joint(joint)

        root = robot.get_root_link()
        assert root == link1

    def test_total_mass(self):
        """Test calculating total mass."""
        robot = Robot(name="test_robot")

        inertia = InertiaTensor(ixx=1.0, ixy=0.0, ixz=0.0, iyy=1.0, iyz=0.0, izz=1.0)
        link1 = Link(name="link1", inertial=Inertial(mass=5.0, inertia=inertia))
        link2 = Link(name="link2", inertial=Inertial(mass=3.0, inertia=inertia))

        robot.add_link(link1)
        robot.add_link(link2)

        assert robot.total_mass == 8.0

    def test_degrees_of_freedom(self):
        """Test calculating degrees of freedom."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))
        robot.add_link(Link(name="link3"))

        # Add fixed joint (0 DOF)
        robot.add_joint(
            Joint(
                name="joint1",
                type=JointType.FIXED,
                parent="link1",
                child="link2",
            )
        )

        # Add revolute joint (1 DOF)
        robot.add_joint(
            Joint(
                name="joint2",
                type=JointType.REVOLUTE,
                parent="link2",
                child="link3",
                limits=JointLimits(lower=-math.pi, upper=math.pi),
            )
        )

        assert robot.degrees_of_freedom == 1


class TestRobotValidation:
    """Tests for robot validation."""

    def test_valid_robot(self):
        """Test that valid robot passes validation."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))

        robot.add_joint(
            Joint(
                name="joint1",
                type=JointType.FIXED,
                parent="link1",
                child="link2",
            )
        )

        errors = robot.validate_tree_structure()
        assert len(errors) == 0

    def test_empty_robot(self):
        """Test that empty robot fails validation."""
        robot = Robot(name="test_robot")
        errors = robot.validate_tree_structure()
        assert len(errors) > 0
        assert any("at least one link" in err for err in errors)

    def test_duplicate_link_names(self):
        """Test that duplicate link names fail validation."""
        robot = Robot(name="test_robot")
        robot.links.append(Link(name="link1"))
        robot.links.append(Link(name="link1"))  # Bypass add_link validation

        errors = robot.validate_tree_structure()
        assert any("Duplicate link names" in err for err in errors)

    def test_duplicate_joint_names(self):
        """Test that duplicate joint names fail validation."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))
        robot.add_link(Link(name="link3"))

        joint1 = Joint(name="joint1", type=JointType.FIXED, parent="link1", child="link2")
        joint2 = Joint(name="joint1", type=JointType.FIXED, parent="link2", child="link3")

        robot.joints.append(joint1)
        robot.joints.append(joint2)

        errors = robot.validate_tree_structure()
        assert any("Duplicate joint names" in err for err in errors)

    def test_missing_parent_link(self):
        """Test that missing parent link fails validation."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link2"))

        # Manually add joint to bypass add_joint validation
        robot.joints.append(
            Joint(
                name="joint1",
                type=JointType.FIXED,
                parent="link1",  # Does not exist
                child="link2",
            )
        )

        errors = robot.validate_tree_structure()
        assert any("parent link 'link1' not found" in err for err in errors)

    def test_cycle_detection(self):
        """Test that cycles are detected."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))

        # Create a cycle
        robot.joints.append(
            Joint(name="joint1", type=JointType.FIXED, parent="link1", child="link2")
        )
        robot.joints.append(
            Joint(name="joint2", type=JointType.FIXED, parent="link2", child="link1")
        )

        errors = robot.validate_tree_structure()
        assert any("cycle" in err.lower() for err in errors)

    def test_multiple_roots(self):
        """Test that multiple roots fail validation."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))
        robot.add_link(Link(name="link3"))

        # Only connect link2 and link3, leaving link1 as orphan root
        robot.add_joint(
            Joint(name="joint1", type=JointType.FIXED, parent="link2", child="link3")
        )

        errors = robot.validate_tree_structure()
        assert any("Multiple root links" in err for err in errors)

    def test_disconnected_link(self):
        """Test that disconnected links fail validation."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))
        robot.add_link(Link(name="link3"))

        # Connect link1 and link2, leaving link3 disconnected
        robot.add_joint(
            Joint(name="joint1", type=JointType.FIXED, parent="link1", child="link2")
        )

        errors = robot.validate_tree_structure()
        # link3 is disconnected and will be detected as a multiple root issue
        assert any("Multiple root links" in err for err in errors)
