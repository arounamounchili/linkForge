"""Tests for URDF generator."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import pytest

from linkforge.core.generators import URDFGenerator
from linkforge.core.models import (
    Box,
    Collision,
    Color,
    Cylinder,
    Inertial,
    InertiaTensor,
    Joint,
    JointLimits,
    JointType,
    Link,
    Material,
    Robot,
    Sphere,
    Transform,
    Vector3,
    Visual,
)


class TestURDFGenerator:
    """Tests for URDF generator."""

    def test_simple_robot(self):
        """Test generating URDF for a simple robot."""
        robot = Robot(name="simple_robot")

        # Create a simple link
        box = Box(size=Vector3(1.0, 1.0, 1.0))
        visual = Visual(geometry=box)
        link = Link(name="base_link", visual=visual)
        robot.add_link(link)

        # Generate URDF
        generator = URDFGenerator()
        urdf = generator.generate(robot)

        # Parse and validate
        root = ET.fromstring(urdf)
        assert root.tag == "robot"
        assert root.get("name") == "simple_robot"

        # Check link exists
        links = root.findall("link")
        assert len(links) == 1
        assert links[0].get("name") == "base_link"

    def test_robot_with_joints(self, simple_robot: Robot):
        """Test generating URDF for robot with joints."""
        generator = URDFGenerator()
        urdf = generator.generate(simple_robot)

        root = ET.fromstring(urdf)

        # Check joints
        joints = root.findall("joint")
        assert len(joints) == 1
        assert joints[0].get("name") == "joint1"
        assert joints[0].get("type") == "revolute"

        # Check parent/child
        parent = joints[0].find("parent")
        child = joints[0].find("child")
        assert parent.get("link") == "link1"
        assert child.get("link") == "link2"

    def test_link_with_visual(self):
        """Test generating URDF for link with visual."""
        robot = Robot(name="test_robot")

        box = Box(size=Vector3(1.0, 2.0, 3.0))
        material = Material(name="red", color=Color(1.0, 0.0, 0.0, 1.0))
        visual = Visual(geometry=box, material=material)
        link = Link(name="link1", visual=visual)

        robot.add_link(link)

        generator = URDFGenerator()
        urdf = generator.generate(robot)
        root = ET.fromstring(urdf)

        # Check visual element
        visual_elem = root.find(".//link[@name='link1']/visual")
        assert visual_elem is not None

        # Check geometry
        geometry = visual_elem.find("geometry/box")
        assert geometry is not None
        assert geometry.get("size") == "1.0 2.0 3.0"

        # Check material
        material_elem = visual_elem.find("material")
        assert material_elem is not None

    def test_link_with_collision(self):
        """Test generating URDF for link with collision."""
        robot = Robot(name="test_robot")

        cylinder = Cylinder(radius=0.5, length=1.0)
        collision = Collision(geometry=cylinder)
        link = Link(name="link1", collision=collision)

        robot.add_link(link)

        generator = URDFGenerator()
        urdf = generator.generate(robot)
        root = ET.fromstring(urdf)

        # Check collision element
        collision_elem = root.find(".//link[@name='link1']/collision")
        assert collision_elem is not None

        # Check geometry
        geometry = collision_elem.find("geometry/cylinder")
        assert geometry is not None
        assert geometry.get("radius") == "0.5"
        assert geometry.get("length") == "1.0"

    def test_link_with_inertial(self):
        """Test generating URDF for link with inertial properties."""
        robot = Robot(name="test_robot")

        inertia = InertiaTensor(ixx=1.0, ixy=0.0, ixz=0.0, iyy=1.0, iyz=0.0, izz=1.0)
        inertial = Inertial(mass=5.0, inertia=inertia)
        link = Link(name="link1", inertial=inertial)

        robot.add_link(link)

        generator = URDFGenerator()
        urdf = generator.generate(robot)
        root = ET.fromstring(urdf)

        # Check inertial element
        inertial_elem = root.find(".//link[@name='link1']/inertial")
        assert inertial_elem is not None

        # Check mass
        mass_elem = inertial_elem.find("mass")
        assert mass_elem is not None
        assert mass_elem.get("value") == "5.0"

        # Check inertia
        inertia_elem = inertial_elem.find("inertia")
        assert inertia_elem is not None
        assert inertia_elem.get("ixx") == "1.0"
        assert inertia_elem.get("iyy") == "1.0"
        assert inertia_elem.get("izz") == "1.0"

    def test_joint_types(self):
        """Test generating URDF for different joint types."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))

        # Test revolute joint
        joint = Joint(
            name="joint1",
            type=JointType.REVOLUTE,
            parent="link1",
            child="link2",
            limits=JointLimits(lower=-math.pi, upper=math.pi, effort=10.0, velocity=1.0),
        )
        robot.add_joint(joint)

        generator = URDFGenerator()
        urdf = generator.generate(robot)
        root = ET.fromstring(urdf)

        joint_elem = root.find(".//joint[@name='joint1']")
        assert joint_elem.get("type") == "revolute"

        # Check limits
        limits = joint_elem.find("limit")
        assert limits is not None
        assert float(limits.get("lower")) == pytest.approx(-math.pi)
        assert float(limits.get("upper")) == pytest.approx(math.pi)

        # Check axis
        axis = joint_elem.find("axis")
        assert axis is not None

    def test_joint_with_origin(self):
        """Test generating URDF for joint with origin transform."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))

        origin = Transform(xyz=Vector3(1.0, 0.0, 0.5), rpy=Vector3(0.0, 0.0, 1.57))
        joint = Joint(
            name="joint1",
            type=JointType.FIXED,
            parent="link1",
            child="link2",
            origin=origin,
        )
        robot.add_joint(joint)

        generator = URDFGenerator()
        urdf = generator.generate(robot)
        root = ET.fromstring(urdf)

        # Check origin
        origin_elem = root.find(".//joint[@name='joint1']/origin")
        assert origin_elem is not None
        assert origin_elem.get("xyz") == "1.0 0.0 0.5"
        assert "1.57" in origin_elem.get("rpy")

    def test_invalid_robot_raises_error(self):
        """Test that invalid robot raises error."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))

        # Add joint with missing parent (bypass validation)
        robot.joints.append(
            Joint(
                name="joint1",
                type=JointType.FIXED,
                parent="nonexistent",
                child="link2",
            )
        )

        generator = URDFGenerator()
        with pytest.raises(ValueError, match="validation failed"):
            generator.generate(robot)

    def test_geometry_types(self):
        """Test all geometry types are correctly generated."""
        robot = Robot(name="test_robot")

        # Box
        robot.add_link(Link(name="box_link", visual=Visual(geometry=Box(Vector3(1, 2, 3)))))

        # Cylinder
        robot.add_link(Link(name="cyl_link", visual=Visual(geometry=Cylinder(0.5, 1.0))))

        # Sphere
        robot.add_link(Link(name="sphere_link", visual=Visual(geometry=Sphere(0.3))))

        # Connect links to form valid tree structure
        robot.add_joint(
            Joint(name="joint1", type=JointType.FIXED, parent="box_link", child="cyl_link")
        )
        robot.add_joint(
            Joint(name="joint2", type=JointType.FIXED, parent="cyl_link", child="sphere_link")
        )

        generator = URDFGenerator()
        urdf = generator.generate(robot)
        root = ET.fromstring(urdf)

        # Check all geometry types exist
        assert root.find(".//link[@name='box_link']/visual/geometry/box") is not None
        assert root.find(".//link[@name='cyl_link']/visual/geometry/cylinder") is not None
        assert root.find(".//link[@name='sphere_link']/visual/geometry/sphere") is not None
