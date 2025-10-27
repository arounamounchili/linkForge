"""Tests for URDF XML parser."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from linkforge.core.models import (
    Box,
    Capsule,
    Collision,
    Color,
    Cylinder,
    Inertial,
    InertiaTensor,
    Joint,
    JointDynamics,
    JointLimits,
    JointMimic,
    JointType,
    Link,
    Material,
    Mesh,
    Robot,
    Sphere,
    Transform,
    Vector3,
    Visual,
)
from linkforge.core.parsers.urdf_parser import (
    parse_geometry,
    parse_joint,
    parse_link,
    parse_material,
    parse_origin,
    parse_urdf,
    parse_vector3,
)


class TestParseVector3:
    """Tests for parse_vector3 function."""

    def test_parse_simple_vector(self):
        """Test parsing simple space-separated vector."""
        vec = parse_vector3("1.0 2.0 3.0")
        assert vec.x == pytest.approx(1.0)
        assert vec.y == pytest.approx(2.0)
        assert vec.z == pytest.approx(3.0)

    def test_parse_vector_with_extra_spaces(self):
        """Test parsing vector with extra spaces."""
        vec = parse_vector3("  1.5   2.5   3.5  ")
        assert vec.x == pytest.approx(1.5)
        assert vec.y == pytest.approx(2.5)
        assert vec.z == pytest.approx(3.5)

    def test_parse_negative_values(self):
        """Test parsing vector with negative values."""
        vec = parse_vector3("-1.0 -2.0 -3.0")
        assert vec.x == pytest.approx(-1.0)
        assert vec.y == pytest.approx(-2.0)
        assert vec.z == pytest.approx(-3.0)

    def test_parse_zeros(self):
        """Test parsing zero vector."""
        vec = parse_vector3("0 0 0")
        assert vec.x == pytest.approx(0.0)
        assert vec.y == pytest.approx(0.0)
        assert vec.z == pytest.approx(0.0)


class TestParseOrigin:
    """Tests for parse_origin function."""

    def test_parse_origin_with_xyz_and_rpy(self):
        """Test parsing origin with both xyz and rpy."""
        elem = ET.fromstring('<origin xyz="1 2 3" rpy="0.1 0.2 0.3"/>')
        transform = parse_origin(elem)

        assert transform.xyz.x == pytest.approx(1.0)
        assert transform.xyz.y == pytest.approx(2.0)
        assert transform.xyz.z == pytest.approx(3.0)
        assert transform.rpy.x == pytest.approx(0.1)
        assert transform.rpy.y == pytest.approx(0.2)
        assert transform.rpy.z == pytest.approx(0.3)

    def test_parse_origin_xyz_only(self):
        """Test parsing origin with only xyz."""
        elem = ET.fromstring('<origin xyz="1 2 3"/>')
        transform = parse_origin(elem)

        assert transform.xyz.x == pytest.approx(1.0)
        assert transform.xyz.y == pytest.approx(2.0)
        assert transform.xyz.z == pytest.approx(3.0)
        assert transform.rpy.x == pytest.approx(0.0)
        assert transform.rpy.y == pytest.approx(0.0)
        assert transform.rpy.z == pytest.approx(0.0)

    def test_parse_origin_rpy_only(self):
        """Test parsing origin with only rpy."""
        elem = ET.fromstring('<origin rpy="1.57 0 0"/>')
        transform = parse_origin(elem)

        assert transform.xyz.x == pytest.approx(0.0)
        assert transform.xyz.y == pytest.approx(0.0)
        assert transform.xyz.z == pytest.approx(0.0)
        assert transform.rpy.x == pytest.approx(1.57)
        assert transform.rpy.y == pytest.approx(0.0)
        assert transform.rpy.z == pytest.approx(0.0)

    def test_parse_origin_none(self):
        """Test parsing None origin returns identity."""
        transform = parse_origin(None)
        assert transform.xyz.x == pytest.approx(0.0)
        assert transform.xyz.y == pytest.approx(0.0)
        assert transform.xyz.z == pytest.approx(0.0)
        assert transform.rpy.x == pytest.approx(0.0)
        assert transform.rpy.y == pytest.approx(0.0)
        assert transform.rpy.z == pytest.approx(0.0)

    def test_parse_origin_empty(self):
        """Test parsing empty origin element."""
        elem = ET.fromstring("<origin/>")
        transform = parse_origin(elem)

        # Should use defaults (0 0 0)
        assert transform.xyz.x == pytest.approx(0.0)
        assert transform.xyz.y == pytest.approx(0.0)
        assert transform.xyz.z == pytest.approx(0.0)


class TestParseGeometry:
    """Tests for parse_geometry function."""

    def test_parse_box(self):
        """Test parsing box geometry."""
        elem = ET.fromstring('<geometry><box size="1 2 3"/></geometry>')
        geom = parse_geometry(elem)

        assert isinstance(geom, Box)
        assert geom.size.x == pytest.approx(1.0)
        assert geom.size.y == pytest.approx(2.0)
        assert geom.size.z == pytest.approx(3.0)

    def test_parse_cylinder(self):
        """Test parsing cylinder geometry."""
        elem = ET.fromstring('<geometry><cylinder radius="0.5" length="2.0"/></geometry>')
        geom = parse_geometry(elem)

        assert isinstance(geom, Cylinder)
        assert geom.radius == pytest.approx(0.5)
        assert geom.length == pytest.approx(2.0)

    def test_parse_sphere(self):
        """Test parsing sphere geometry."""
        elem = ET.fromstring('<geometry><sphere radius="1.5"/></geometry>')
        geom = parse_geometry(elem)

        assert isinstance(geom, Sphere)
        assert geom.radius == pytest.approx(1.5)

    def test_parse_capsule(self):
        """Test parsing capsule geometry."""
        elem = ET.fromstring('<geometry><capsule radius="0.3" length="1.0"/></geometry>')
        geom = parse_geometry(elem)

        assert isinstance(geom, Capsule)
        assert geom.radius == pytest.approx(0.3)
        assert geom.length == pytest.approx(1.0)

    def test_parse_mesh(self):
        """Test parsing mesh geometry."""
        elem = ET.fromstring('<geometry><mesh filename="robot.stl" scale="1 1 1"/></geometry>')
        geom = parse_geometry(elem)

        assert isinstance(geom, Mesh)
        assert str(geom.filepath) == "robot.stl"
        assert geom.scale.x == pytest.approx(1.0)
        assert geom.scale.y == pytest.approx(1.0)
        assert geom.scale.z == pytest.approx(1.0)

    def test_parse_mesh_with_scale(self):
        """Test parsing mesh with custom scale."""
        elem = ET.fromstring('<geometry><mesh filename="model.dae" scale="0.001 0.001 0.001"/></geometry>')
        geom = parse_geometry(elem)

        assert isinstance(geom, Mesh)
        assert geom.scale.x == pytest.approx(0.001)

    def test_parse_empty_geometry(self):
        """Test parsing empty geometry element."""
        elem = ET.fromstring("<geometry></geometry>")
        geom = parse_geometry(elem)

        assert geom is None


class TestParseMaterial:
    """Tests for parse_material function."""

    def test_parse_material_with_color(self):
        """Test parsing material with RGBA color."""
        elem = ET.fromstring('<material name="red"><color rgba="1 0 0 1"/></material>')
        materials = {}
        mat = parse_material(elem, materials)

        assert mat is not None
        assert mat.name == "red"
        assert mat.color.r == pytest.approx(1.0)
        assert mat.color.g == pytest.approx(0.0)
        assert mat.color.b == pytest.approx(0.0)
        assert mat.color.a == pytest.approx(1.0)

    def test_parse_material_rgb_only(self):
        """Test parsing material with RGB (no alpha)."""
        elem = ET.fromstring('<material name="blue"><color rgba="0 0 1"/></material>')
        materials = {}
        mat = parse_material(elem, materials)

        assert mat is not None
        assert mat.color.r == pytest.approx(0.0)
        assert mat.color.g == pytest.approx(0.0)
        assert mat.color.b == pytest.approx(1.0)
        assert mat.color.a == pytest.approx(1.0)  # Default alpha

    def test_parse_material_reference(self):
        """Test parsing material reference to existing material."""
        # Create existing material
        existing = Material(name="gray", color=Color(0.5, 0.5, 0.5, 1.0))
        materials = {"gray": existing}

        # Reference the material
        elem = ET.fromstring('<material name="gray"/>')
        mat = parse_material(elem, materials)

        assert mat is existing  # Should return the same object

    def test_parse_material_none(self):
        """Test parsing None material element."""
        materials = {}
        mat = parse_material(None, materials)

        assert mat is None

    def test_parse_material_no_color(self):
        """Test parsing material without color element."""
        elem = ET.fromstring('<material name="no_color"/>')
        materials = {}
        mat = parse_material(elem, materials)

        assert mat is None


class TestParseLink:
    """Tests for parse_link function."""

    def test_parse_simple_link(self):
        """Test parsing simple link with visual."""
        xml = """
        <link name="test_link">
            <visual>
                <geometry>
                    <box size="1 1 1"/>
                </geometry>
            </visual>
        </link>
        """
        elem = ET.fromstring(xml)
        materials = {}
        link = parse_link(elem, materials)

        assert link.name == "test_link"
        assert link.visual is not None
        assert isinstance(link.visual.geometry, Box)

    def test_parse_link_with_collision(self):
        """Test parsing link with collision."""
        xml = """
        <link name="link1">
            <collision>
                <geometry>
                    <cylinder radius="0.5" length="1.0"/>
                </geometry>
            </collision>
        </link>
        """
        elem = ET.fromstring(xml)
        materials = {}
        link = parse_link(elem, materials)

        assert link.collision is not None
        assert isinstance(link.collision.geometry, Cylinder)

    def test_parse_link_with_inertial(self):
        """Test parsing link with inertial properties."""
        xml = """
        <link name="link_with_mass">
            <inertial>
                <mass value="5.0"/>
                <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
            </inertial>
        </link>
        """
        elem = ET.fromstring(xml)
        materials = {}
        link = parse_link(elem, materials)

        assert link.inertial is not None
        assert link.inertial.mass == pytest.approx(5.0)
        assert link.inertial.inertia.ixx == pytest.approx(1.0)

    def test_parse_link_with_material(self):
        """Test parsing link with material."""
        xml = """
        <link name="colored_link">
            <visual>
                <geometry>
                    <sphere radius="0.5"/>
                </geometry>
                <material name="green">
                    <color rgba="0 1 0 1"/>
                </material>
            </visual>
        </link>
        """
        elem = ET.fromstring(xml)
        materials = {}
        link = parse_link(elem, materials)

        assert link.visual is not None
        assert link.visual.material is not None
        assert link.visual.material.name == "green"

    def test_parse_link_with_origin(self):
        """Test parsing link with origin offset."""
        xml = """
        <link name="offset_link">
            <visual>
                <origin xyz="1 0 0" rpy="0 0 1.57"/>
                <geometry>
                    <box size="1 1 1"/>
                </geometry>
            </visual>
        </link>
        """
        elem = ET.fromstring(xml)
        materials = {}
        link = parse_link(elem, materials)

        assert link.visual is not None
        assert link.visual.origin.xyz.x == pytest.approx(1.0)
        assert link.visual.origin.rpy.z == pytest.approx(1.57)

    def test_parse_empty_link(self):
        """Test parsing link without visual, collision, or inertial."""
        xml = '<link name="empty_link"/>'
        elem = ET.fromstring(xml)
        materials = {}
        link = parse_link(elem, materials)

        assert link.name == "empty_link"
        assert link.visual is None
        assert link.collision is None
        assert link.inertial is None


class TestParseJoint:
    """Tests for parse_joint function."""

    def test_parse_revolute_joint(self):
        """Test parsing revolute joint."""
        xml = """
        <joint name="joint1" type="revolute">
            <parent link="link1"/>
            <child link="link2"/>
            <axis xyz="0 0 1"/>
            <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
        </joint>
        """
        elem = ET.fromstring(xml)
        joint = parse_joint(elem)

        assert joint.name == "joint1"
        assert joint.type == JointType.REVOLUTE
        assert joint.parent == "link1"
        assert joint.child == "link2"
        assert joint.axis.z == pytest.approx(1.0)
        assert joint.limits is not None
        assert joint.limits.lower == pytest.approx(-1.57)
        assert joint.limits.upper == pytest.approx(1.57)

    def test_parse_continuous_joint(self):
        """Test parsing continuous joint."""
        xml = """
        <joint name="wheel_joint" type="continuous">
            <parent link="base"/>
            <child link="wheel"/>
            <axis xyz="0 1 0"/>
        </joint>
        """
        elem = ET.fromstring(xml)
        joint = parse_joint(elem)

        assert joint.type == JointType.CONTINUOUS
        assert joint.axis.y == pytest.approx(1.0)

    def test_parse_fixed_joint(self):
        """Test parsing fixed joint."""
        xml = """
        <joint name="fixed_joint" type="fixed">
            <parent link="base"/>
            <child link="sensor"/>
        </joint>
        """
        elem = ET.fromstring(xml)
        joint = parse_joint(elem)

        assert joint.type == JointType.FIXED

    def test_parse_prismatic_joint(self):
        """Test parsing prismatic joint."""
        xml = """
        <joint name="slider" type="prismatic">
            <parent link="base"/>
            <child link="slide"/>
            <axis xyz="1 0 0"/>
            <limit lower="0" upper="1.0" effort="100" velocity="0.5"/>
        </joint>
        """
        elem = ET.fromstring(xml)
        joint = parse_joint(elem)

        assert joint.type == JointType.PRISMATIC
        assert joint.axis.x == pytest.approx(1.0)

    def test_parse_joint_with_origin(self):
        """Test parsing joint with origin."""
        xml = """
        <joint name="joint_with_origin" type="revolute">
            <parent link="link1"/>
            <child link="link2"/>
            <origin xyz="0.5 0 0" rpy="0 1.57 0"/>
            <limit lower="-3.14" upper="3.14" effort="50" velocity="2"/>
        </joint>
        """
        elem = ET.fromstring(xml)
        joint = parse_joint(elem)

        assert joint.origin.xyz.x == pytest.approx(0.5)
        assert joint.origin.rpy.y == pytest.approx(1.57)

    def test_parse_joint_with_dynamics(self):
        """Test parsing joint with dynamics."""
        xml = """
        <joint name="damped_joint" type="revolute">
            <parent link="link1"/>
            <child link="link2"/>
            <dynamics damping="0.5" friction="0.1"/>
            <limit lower="-1" upper="1" effort="10" velocity="1"/>
        </joint>
        """
        elem = ET.fromstring(xml)
        joint = parse_joint(elem)

        assert joint.dynamics is not None
        assert joint.dynamics.damping == pytest.approx(0.5)
        assert joint.dynamics.friction == pytest.approx(0.1)

    def test_parse_joint_with_mimic(self):
        """Test parsing joint with mimic."""
        xml = """
        <joint name="mimic_joint" type="revolute">
            <parent link="link1"/>
            <child link="link2"/>
            <mimic joint="leader_joint" multiplier="2.0" offset="0.1"/>
            <limit lower="-1" upper="1" effort="10" velocity="1"/>
        </joint>
        """
        elem = ET.fromstring(xml)
        joint = parse_joint(elem)

        assert joint.mimic is not None
        assert joint.mimic.joint == "leader_joint"
        assert joint.mimic.multiplier == pytest.approx(2.0)
        assert joint.mimic.offset == pytest.approx(0.1)

    def test_parse_floating_joint(self):
        """Test parsing floating joint type."""
        xml = """
        <joint name="float_joint" type="floating">
            <parent link="world"/>
            <child link="object"/>
        </joint>
        """
        elem = ET.fromstring(xml)
        joint = parse_joint(elem)

        assert joint.type == JointType.FLOATING

    def test_parse_planar_joint(self):
        """Test parsing planar joint type."""
        xml = """
        <joint name="planar_joint" type="planar">
            <parent link="base"/>
            <child link="platform"/>
        </joint>
        """
        elem = ET.fromstring(xml)
        joint = parse_joint(elem)

        assert joint.type == JointType.PLANAR


class TestParseURDF:
    """Tests for parse_urdf function (integration tests)."""

    def test_parse_simple_robot(self, tmp_path: Path):
        """Test parsing simple robot with one link."""
        urdf_content = """<?xml version="1.0"?>
<robot name="simple_robot">
    <link name="base_link">
        <visual>
            <geometry>
                <box size="1 1 1"/>
            </geometry>
        </visual>
    </link>
</robot>
"""
        urdf_file = tmp_path / "simple.urdf"
        urdf_file.write_text(urdf_content)

        robot = parse_urdf(urdf_file)

        assert robot.name == "simple_robot"
        assert len(robot.links) == 1
        assert robot.links[0].name == "base_link"

    def test_parse_robot_with_joint(self, tmp_path: Path):
        """Test parsing robot with links and joints."""
        urdf_content = """<?xml version="1.0"?>
<robot name="two_link_robot">
    <link name="link1">
        <visual>
            <geometry>
                <cylinder radius="0.1" length="0.5"/>
            </geometry>
        </visual>
    </link>
    <link name="link2">
        <visual>
            <geometry>
                <box size="0.2 0.2 0.2"/>
            </geometry>
        </visual>
    </link>
    <joint name="joint1" type="revolute">
        <parent link="link1"/>
        <child link="link2"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
    </joint>
</robot>
"""
        urdf_file = tmp_path / "two_link.urdf"
        urdf_file.write_text(urdf_content)

        robot = parse_urdf(urdf_file)

        assert robot.name == "two_link_robot"
        assert len(robot.links) == 2
        assert len(robot.joints) == 1
        assert robot.joints[0].name == "joint1"
        assert robot.joints[0].type == JointType.REVOLUTE

    def test_parse_robot_with_global_materials(self, tmp_path: Path):
        """Test parsing robot with global material definitions."""
        urdf_content = """<?xml version="1.0"?>
<robot name="colored_robot">
    <material name="red">
        <color rgba="1 0 0 1"/>
    </material>
    <material name="blue">
        <color rgba="0 0 1 1"/>
    </material>
    <link name="link1">
        <visual>
            <geometry>
                <box size="1 1 1"/>
            </geometry>
            <material name="red"/>
        </visual>
    </link>
    <link name="link2">
        <visual>
            <geometry>
                <sphere radius="0.5"/>
            </geometry>
            <material name="blue"/>
        </visual>
    </link>
    <joint name="j1" type="fixed">
        <parent link="link1"/>
        <child link="link2"/>
    </joint>
</robot>
"""
        urdf_file = tmp_path / "colored.urdf"
        urdf_file.write_text(urdf_content)

        robot = parse_urdf(urdf_file)

        assert len(robot.links) == 2
        # Both links should reference the global materials
        assert robot.links[0].visual.material is not None
        assert robot.links[1].visual.material is not None

    def test_parse_file_not_found(self, tmp_path: Path):
        """Test that parsing nonexistent file raises FileNotFoundError."""
        nonexistent = tmp_path / "nonexistent.urdf"

        with pytest.raises(FileNotFoundError):
            parse_urdf(nonexistent)

    def test_parse_invalid_xml(self, tmp_path: Path):
        """Test that invalid XML raises ParseError."""
        urdf_file = tmp_path / "invalid.urdf"
        urdf_file.write_text("<robot><link></robot>")  # Malformed XML

        with pytest.raises(ET.ParseError):
            parse_urdf(urdf_file)

    def test_parse_non_robot_root(self, tmp_path: Path):
        """Test that non-robot root element raises ValueError."""
        urdf_content = """<?xml version="1.0"?>
<notarobot name="invalid">
    <link name="link1"/>
</notarobot>
"""
        urdf_file = tmp_path / "invalid_root.urdf"
        urdf_file.write_text(urdf_content)

        with pytest.raises(ValueError, match="Root element must be <robot>"):
            parse_urdf(urdf_file)

    def test_parse_complex_robot(self, tmp_path: Path):
        """Test parsing complex robot with multiple features."""
        urdf_content = """<?xml version="1.0"?>
<robot name="complex_robot">
    <material name="gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
    </material>

    <link name="base_link">
        <visual>
            <origin xyz="0 0 0.05" rpy="0 0 0"/>
            <geometry>
                <box size="0.5 0.3 0.1"/>
            </geometry>
            <material name="gray"/>
        </visual>
        <collision>
            <geometry>
                <box size="0.5 0.3 0.1"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="1.0"/>
            <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
        </inertial>
    </link>

    <link name="wheel_left">
        <visual>
            <geometry>
                <cylinder radius="0.05" length="0.02"/>
            </geometry>
            <material name="gray"/>
        </visual>
    </link>

    <link name="wheel_right">
        <visual>
            <geometry>
                <cylinder radius="0.05" length="0.02"/>
            </geometry>
            <material name="gray"/>
        </visual>
    </link>

    <joint name="wheel_left_joint" type="continuous">
        <parent link="base_link"/>
        <child link="wheel_left"/>
        <origin xyz="0.15 0.2 0" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
    </joint>

    <joint name="wheel_right_joint" type="continuous">
        <parent link="base_link"/>
        <child link="wheel_right"/>
        <origin xyz="0.15 -0.2 0" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
    </joint>
</robot>
"""
        urdf_file = tmp_path / "complex.urdf"
        urdf_file.write_text(urdf_content)

        robot = parse_urdf(urdf_file)

        assert robot.name == "complex_robot"
        assert len(robot.links) == 3
        assert len(robot.joints) == 2

        # Check base link has all properties
        base_link = robot.links[0]
        assert base_link.visual is not None
        assert base_link.collision is not None
        assert base_link.inertial is not None
        assert base_link.inertial.mass == pytest.approx(1.0)

        # Check joints are continuous type
        assert all(j.type == JointType.CONTINUOUS for j in robot.joints)
