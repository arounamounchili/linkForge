"""Tests for XACRO generator."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from linkforge.core.generators import XACROGenerator
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


class TestXACROGenerator:
    """Tests for XACRO generator."""

    def test_simple_robot_with_xacro_namespace(self):
        """Test generating XACRO with xacro namespace."""
        robot = Robot(name="simple_robot")

        # Create a simple link
        box = Box(size=Vector3(1.0, 1.0, 1.0))
        visual = Visual(geometry=box)
        link = Link(name="base_link", visual=visual)
        robot.add_link(link)

        # Generate XACRO
        generator = XACROGenerator(advanced_mode=False)
        xacro = generator.generate(robot)

        # Parse and validate
        root = ET.fromstring(xacro)
        assert root.tag == "robot"

        # Check that xacro namespace declaration is present in the XML string
        assert 'xmlns:xacro="http://www.ros.org/wiki/xacro"' in xacro

        # Check that xacro:property elements can be found
        # (verifies the namespace is properly registered)
        properties = root.findall(".//{http://www.ros.org/wiki/xacro}property")
        assert len(properties) > 0

    def test_robot_name_property(self):
        """Test that robot name is extracted as XACRO property."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="base_link"))

        generator = XACROGenerator()
        xacro = generator.generate(robot)
        root = ET.fromstring(xacro)

        # Check robot_name property exists
        properties = root.findall("xacro:property", {"xacro": "http://www.ros.org/wiki/xacro"})
        robot_name_prop = None
        for prop in properties:
            if prop.get("name") == "robot_name":
                robot_name_prop = prop
                break

        assert robot_name_prop is not None
        assert robot_name_prop.get("value") == "test_robot"

        # Check robot name uses property reference
        assert root.get("name") == "${robot_name}"

    def test_material_property_extraction(self):
        """Test extraction of material colors as XACRO properties."""
        robot = Robot(name="test_robot")

        # Create links with different materials
        red_material = Material(name="red", color=Color(1.0, 0.0, 0.0, 1.0))
        blue_material = Material(name="blue", color=Color(0.0, 0.0, 1.0, 1.0))

        link1 = Link(
            name="link1",
            visual=Visual(geometry=Box(Vector3(1, 1, 1)), material=red_material),
        )
        link2 = Link(
            name="link2",
            visual=Visual(geometry=Box(Vector3(1, 1, 1)), material=blue_material),
        )

        robot.add_link(link1)
        robot.add_link(link2)
        robot.add_joint(Joint(name="j1", type=JointType.FIXED, parent="link1", child="link2"))

        # Generate XACRO with material extraction
        generator = XACROGenerator(
            advanced_mode=True,
            extract_materials=True,
            extract_dimensions=False,
            generate_macros=False,
        )
        xacro = generator.generate(robot)
        root = ET.fromstring(xacro)

        # Check color properties were created
        properties = root.findall("xacro:property", {"xacro": "http://www.ros.org/wiki/xacro"})
        property_names = [p.get("name") for p in properties]

        assert "color_red" in property_names
        assert "color_blue" in property_names

        # Check that material colors use property references
        materials = root.findall(".//material")
        for material in materials:
            mat_name = material.get("name")
            if mat_name in ["red", "blue"]:
                color_elem = material.find("color")
                if color_elem is not None:
                    rgba_value = color_elem.get("rgba")
                    # Should contain property reference like ${color_red}
                    assert rgba_value.startswith("${")
                    assert rgba_value.endswith("}")

        # Check tracked material properties
        assert "red" in generator.material_properties
        assert "blue" in generator.material_properties
        assert generator.material_properties["red"] == "color_red"
        assert generator.material_properties["blue"] == "color_blue"

    def test_material_property_extraction_same_material(self):
        """Test that same material used multiple times creates single property."""
        robot = Robot(name="test_robot")

        # Create multiple links with the same material
        gray_material = Material(name="gray", color=Color(0.5, 0.5, 0.5, 1.0))

        for i in range(3):
            link = Link(
                name=f"link{i}",
                visual=Visual(geometry=Box(Vector3(1, 1, 1)), material=gray_material),
            )
            robot.add_link(link)
            if i > 0:
                robot.add_joint(
                    Joint(name=f"j{i}", type=JointType.FIXED, parent=f"link{i-1}", child=f"link{i}")
                )

        generator = XACROGenerator(
            advanced_mode=True,
            extract_materials=True,
            extract_dimensions=False,
            generate_macros=False,
        )
        xacro = generator.generate(robot)
        root = ET.fromstring(xacro)

        # Check that only ONE color_gray property was created
        properties = root.findall("xacro:property", {"xacro": "http://www.ros.org/wiki/xacro"})
        color_gray_props = [p for p in properties if p.get("name") == "color_gray"]

        assert len(color_gray_props) == 1
        assert color_gray_props[0].get("value") == "0.5 0.5 0.5 1.0"

    def test_dimension_property_extraction(self):
        """Test extraction of common dimensions as XACRO properties."""
        robot = Robot(name="test_robot")

        # Create multiple links with same radius cylinder
        for i in range(3):
            link = Link(
                name=f"link{i}",
                visual=Visual(geometry=Cylinder(radius=0.1, length=0.5)),
            )
            robot.add_link(link)
            if i > 0:
                robot.add_joint(
                    Joint(name=f"j{i}", type=JointType.FIXED, parent=f"link{i-1}", child=f"link{i}")
                )

        generator = XACROGenerator(
            advanced_mode=True,
            extract_materials=False,
            extract_dimensions=True,
            generate_macros=False,
        )
        xacro = generator.generate(robot)

        # Check that dimension properties were tracked
        # (dimension_properties stores dimensions used 2+ times)
        assert len(generator.dimension_properties) > 0

    def test_pattern_detection_identical_cylinders(self):
        """Test pattern detection for identical cylinder links."""
        robot = Robot(name="test_robot")

        # Base link
        base = Link(name="base_link", visual=Visual(geometry=Box(Vector3(1, 1, 1))))
        robot.add_link(base)

        # Create 4 identical cylinder wheels
        for i in range(4):
            wheel = Link(
                name=f"wheel_{i}",
                visual=Visual(geometry=Cylinder(radius=0.1, length=0.05)),
            )
            robot.add_link(wheel)
            robot.add_joint(
                Joint(
                    name=f"wheel_{i}_joint",
                    type=JointType.CONTINUOUS,
                    parent="base_link",
                    child=f"wheel_{i}",
                )
            )

        generator = XACROGenerator(
            advanced_mode=True,
            extract_materials=False,
            extract_dimensions=False,
            generate_macros=True,
        )
        xacro = generator.generate(robot)

        # Check that macro was generated
        assert len(generator.generated_macros) >= 1

        # Find the wheel macro
        wheel_macro = None
        for macro_info in generator.generated_macros:
            if "cyl" in macro_info["name"]:
                wheel_macro = macro_info
                break

        assert wheel_macro is not None
        # Should detect all 4 wheels as instances
        assert len(wheel_macro["instances"]) == 4
        assert all(f"wheel_{i}" in wheel_macro["instances"] for i in range(4))

    def test_pattern_detection_different_geometries(self):
        """Test that different geometries don't create macros."""
        robot = Robot(name="test_robot")

        # Create links with different geometries
        robot.add_link(Link(name="box_link", visual=Visual(geometry=Box(Vector3(1, 1, 1)))))
        robot.add_link(
            Link(name="cylinder_link", visual=Visual(geometry=Cylinder(radius=0.5, length=1.0)))
        )
        robot.add_link(Link(name="sphere_link", visual=Visual(geometry=Sphere(radius=0.5))))

        robot.add_joint(
            Joint(name="j1", type=JointType.FIXED, parent="box_link", child="cylinder_link")
        )
        robot.add_joint(
            Joint(name="j2", type=JointType.FIXED, parent="cylinder_link", child="sphere_link")
        )

        generator = XACROGenerator(
            advanced_mode=True,
            extract_materials=False,
            extract_dimensions=False,
            generate_macros=True,
        )
        xacro = generator.generate(robot)

        # Should not generate macros (all geometries are different)
        assert len(generator.generated_macros) == 0

    def test_pattern_detection_signature_generation(self):
        """Test link signature generation for pattern detection."""
        generator = XACROGenerator()

        # Test cylinder signature
        cyl_link = Link(
            name="cyl",
            visual=Visual(geometry=Cylinder(radius=0.1, length=0.5)),
        )
        cyl_sig = generator._get_link_signature(cyl_link)
        assert cyl_sig is not None
        assert "cyl" in cyl_sig
        assert "0.100" in cyl_sig
        assert "0.500" in cyl_sig

        # Test box signature
        box_link = Link(
            name="box",
            visual=Visual(geometry=Box(size=Vector3(1.0, 2.0, 3.0))),
        )
        box_sig = generator._get_link_signature(box_link)
        assert box_sig is not None
        assert "box" in box_sig
        assert "1.000" in box_sig
        assert "2.000" in box_sig
        assert "3.000" in box_sig

        # Test sphere signature
        sphere_link = Link(
            name="sphere",
            visual=Visual(geometry=Sphere(radius=0.5)),
        )
        sphere_sig = generator._get_link_signature(sphere_link)
        assert sphere_sig is not None
        assert "sph" in sphere_sig
        assert "0.500" in sphere_sig

        # Test link without visual
        empty_link = Link(name="empty")
        assert generator._get_link_signature(empty_link) is None

    def test_macro_generation_creates_xml_element(self):
        """Test that macro generation creates proper XML macro element."""
        robot = Robot(name="test_robot")

        # Base
        robot.add_link(Link(name="base_link"))

        # Create 2 identical links (minimum for macro generation)
        for i in range(2):
            link = Link(
                name=f"link_{i}",
                visual=Visual(geometry=Cylinder(radius=0.1, length=0.05)),
            )
            robot.add_link(link)
            robot.add_joint(
                Joint(name=f"j{i}", type=JointType.FIXED, parent="base_link", child=f"link_{i}")
            )

        generator = XACROGenerator(
            advanced_mode=True,
            extract_materials=False,
            extract_dimensions=False,
            generate_macros=True,
        )
        xacro = generator.generate(robot)
        root = ET.fromstring(xacro)

        # Check that macro element exists in XML
        macros = root.findall("xacro:macro", {"xacro": "http://www.ros.org/wiki/xacro"})
        assert len(macros) >= 1

        # Check macro has required attributes
        macro = macros[0]
        assert macro.get("name") is not None
        assert macro.get("params") is not None
        assert "name" in macro.get("params")
        assert "parent" in macro.get("params")

    def test_file_splitting_disabled(self, tmp_path: Path):
        """Test that normal write works when file splitting is disabled."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="base_link"))

        generator = XACROGenerator(split_files=False)
        output_file = tmp_path / "robot.xacro"
        generator.write(robot, output_file)

        # Check only one file was created
        xacro_files = list(tmp_path.glob("*.xacro"))
        assert len(xacro_files) == 1
        assert output_file.exists()

        # Verify content is valid
        content = output_file.read_text()
        root = ET.fromstring(content)
        assert root.tag == "robot"

    def test_file_splitting_creates_multiple_files(self, tmp_path: Path):
        """Test that file splitting creates separate files."""
        robot = Robot(name="test_robot")

        # Create robot with materials and potential macros
        red_material = Material(name="red", color=Color(1.0, 0.0, 0.0, 1.0))
        blue_material = Material(name="blue", color=Color(0.0, 0.0, 1.0, 1.0))

        base = Link(
            name="base_link",
            visual=Visual(geometry=Box(Vector3(1, 1, 1)), material=red_material),
        )
        robot.add_link(base)

        # Add wheels with same material
        for i in range(2):
            wheel = Link(
                name=f"wheel_{i}",
                visual=Visual(geometry=Cylinder(radius=0.1, length=0.05), material=blue_material),
            )
            robot.add_link(wheel)
            robot.add_joint(
                Joint(
                    name=f"wheel_{i}_joint",
                    type=JointType.CONTINUOUS,
                    parent="base_link",
                    child=f"wheel_{i}",
                )
            )

        generator = XACROGenerator(
            advanced_mode=True,
            extract_materials=True,
            generate_macros=True,
            split_files=True,
        )

        output_file = tmp_path / "robot.xacro"
        generator.write(robot, output_file)

        # Check that multiple files were created
        xacro_files = sorted(tmp_path.glob("*.xacro"))
        assert len(xacro_files) >= 2  # At least robot.xacro and robot_materials.xacro

        # Check main file exists
        assert output_file.exists()

        # Check materials file exists
        materials_file = tmp_path / "robot_materials.xacro"
        assert materials_file.exists()

    def test_file_splitting_main_file_includes(self, tmp_path: Path):
        """Test that main file contains xacro:include statements."""
        robot = Robot(name="test_robot")

        # Create robot with materials
        material = Material(name="gray", color=Color(0.5, 0.5, 0.5, 1.0))
        link = Link(
            name="base_link",
            visual=Visual(geometry=Box(Vector3(1, 1, 1)), material=material),
        )
        robot.add_link(link)

        generator = XACROGenerator(
            advanced_mode=True,
            extract_materials=True,
            generate_macros=False,
            split_files=True,
        )

        output_file = tmp_path / "robot.xacro"
        generator.write(robot, output_file)

        # Read main file and check for includes
        content = output_file.read_text()
        root = ET.fromstring(content)

        # Find xacro:include elements
        includes = root.findall("xacro:include", {"xacro": "http://www.ros.org/wiki/xacro"})
        assert len(includes) >= 1

        # Check that includes reference correct files
        include_filenames = [inc.get("filename") for inc in includes]
        assert "robot_materials.xacro" in include_filenames

    def test_file_splitting_materials_file_content(self, tmp_path: Path):
        """Test that materials file contains material definitions."""
        robot = Robot(name="test_robot")

        # Create robot with materials
        red_material = Material(name="red", color=Color(1.0, 0.0, 0.0, 1.0))
        link = Link(
            name="base_link",
            visual=Visual(geometry=Box(Vector3(1, 1, 1)), material=red_material),
        )
        robot.add_link(link)

        generator = XACROGenerator(
            advanced_mode=True,
            extract_materials=True,
            generate_macros=False,
            split_files=True,
        )

        output_file = tmp_path / "robot.xacro"
        generator.write(robot, output_file)

        # Read materials file
        materials_file = tmp_path / "robot_materials.xacro"
        assert materials_file.exists()

        content = materials_file.read_text()
        root = ET.fromstring(content)

        # Check for material elements
        materials = root.findall("material")
        assert len(materials) >= 1

        # Check material has color with property reference
        mat = materials[0]
        assert mat.get("name") == "red"
        color = mat.find("color")
        assert color is not None
        assert "${" in color.get("rgba")

    def test_file_splitting_macros_file_content(self, tmp_path: Path):
        """Test that macros file contains macro definitions."""
        robot = Robot(name="test_robot")

        # Create robot with repeated pattern
        robot.add_link(Link(name="base_link"))

        for i in range(2):
            link = Link(
                name=f"link_{i}",
                visual=Visual(geometry=Cylinder(radius=0.1, length=0.05)),
            )
            robot.add_link(link)
            robot.add_joint(
                Joint(name=f"j{i}", type=JointType.FIXED, parent="base_link", child=f"link_{i}")
            )

        generator = XACROGenerator(
            advanced_mode=True,
            extract_materials=False,
            generate_macros=True,
            split_files=True,
        )

        output_file = tmp_path / "robot.xacro"
        generator.write(robot, output_file)

        # Check if macros file was created
        macros_file = tmp_path / "robot_macros.xacro"
        if macros_file.exists():
            content = macros_file.read_text()
            root = ET.fromstring(content)

            # Check for macro elements
            macros = root.findall("xacro:macro", {"xacro": "http://www.ros.org/wiki/xacro"})
            assert len(macros) >= 1

    def test_advanced_mode_disabled(self):
        """Test that advanced features are disabled when advanced_mode=False."""
        robot = Robot(name="test_robot")

        # Create robot with materials and repeated patterns
        material = Material(name="red", color=Color(1.0, 0.0, 0.0, 1.0))

        for i in range(3):
            link = Link(
                name=f"link{i}",
                visual=Visual(geometry=Cylinder(radius=0.1, length=0.05), material=material),
            )
            robot.add_link(link)
            if i > 0:
                robot.add_joint(
                    Joint(name=f"j{i}", type=JointType.FIXED, parent=f"link{i-1}", child=f"link{i}")
                )

        # Generate with advanced_mode=False
        generator = XACROGenerator(advanced_mode=False)
        xacro = generator.generate(robot)
        root = ET.fromstring(xacro)

        # Check that only robot_name property exists
        properties = root.findall("xacro:property", {"xacro": "http://www.ros.org/wiki/xacro"})
        assert len(properties) == 1
        assert properties[0].get("name") == "robot_name"

        # Check that no macros were generated
        macros = root.findall("xacro:macro", {"xacro": "http://www.ros.org/wiki/xacro"})
        assert len(macros) == 0

        # Check tracking attributes are empty
        assert len(generator.material_properties) == 0
        assert len(generator.dimension_properties) == 0
        assert len(generator.generated_macros) == 0

    def test_robot_with_joints(self, simple_robot: Robot):
        """Test generating XACRO for robot with joints."""
        generator = XACROGenerator()
        xacro = generator.generate(simple_robot)

        root = ET.fromstring(xacro)

        # Check joints
        joints = root.findall("joint")
        assert len(joints) == 1
        assert joints[0].get("name") == "joint1"
        assert joints[0].get("type") == "revolute"

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

        generator = XACROGenerator()
        with pytest.raises(ValueError, match="validation failed"):
            generator.generate(robot)

    def test_pretty_print_disabled(self):
        """Test that pretty_print=False generates compact XML."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="base_link"))

        generator = XACROGenerator(pretty_print=False)
        xacro = generator.generate(robot)

        # Compact XML should not have excessive newlines
        # (exact format depends on ET.tostring, but should be more compact)
        assert xacro.count("\n") < 10  # Less structured than pretty printed

    def test_combination_all_advanced_features(self):
        """Test enabling all advanced features together."""
        robot = Robot(name="car_robot")

        # Create complex robot
        gray = Material(name="gray", color=Color(0.5, 0.5, 0.5, 1.0))
        black = Material(name="black", color=Color(0.1, 0.1, 0.1, 1.0))

        # Base
        base = Link(
            name="base_link",
            visual=Visual(geometry=Box(Vector3(1, 0.5, 0.3)), material=gray),
        )
        robot.add_link(base)

        # 4 identical wheels
        for i in range(4):
            wheel = Link(
                name=f"wheel_{i}",
                visual=Visual(geometry=Cylinder(radius=0.1, length=0.05), material=black),
            )
            robot.add_link(wheel)
            robot.add_joint(
                Joint(
                    name=f"wheel_{i}_joint",
                    type=JointType.CONTINUOUS,
                    parent="base_link",
                    child=f"wheel_{i}",
                )
            )

        # Generate with all features
        generator = XACROGenerator(
            advanced_mode=True,
            extract_materials=True,
            extract_dimensions=True,
            generate_macros=True,
            split_files=False,
        )
        xacro = generator.generate(robot)
        root = ET.fromstring(xacro)

        # Verify all features are present
        properties = root.findall("xacro:property", {"xacro": "http://www.ros.org/wiki/xacro"})
        macros = root.findall("xacro:macro", {"xacro": "http://www.ros.org/wiki/xacro"})

        # Should have: robot_name, color_gray, color_black, possibly dimension props
        assert len(properties) >= 3
        assert len(macros) >= 1  # Wheel macro

        # Check tracking
        assert "gray" in generator.material_properties
        assert "black" in generator.material_properties
        assert len(generator.generated_macros) >= 1
