"""XACRO generation from robot models.

XACRO (XML Macros) is an extension of URDF that supports:
- Variables/properties
- Macros for repeated elements
- Math expressions
- File includes

This module provides both basic and advanced XACRO generation:
- Basic: Simple property substitution
- Advanced: Material properties, dimension extraction, macro generation
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path
from typing import Any
from xml.dom import minidom

from ..models.robot import Robot
from ..models.material import Material
from ..models.geometry import Box, Cylinder, Sphere, GeometryType
from .urdf import URDFGenerator


class XACROGenerator:
    """Generate XACRO from Robot model.

    Supports both basic and advanced XACRO generation:
    - Basic mode: Robot name property and xacro namespace
    - Advanced mode: Material properties, dimension extraction, macro generation

    Advanced features:
    - Extract unique materials as color properties
    - Extract common dimensions as properties
    - Auto-generate macros for repeated patterns
    - Optional file splitting (materials.xacro, macros.xacro, robot.xacro)
    """

    def __init__(
        self,
        pretty_print: bool = True,
        advanced_mode: bool = True,
        extract_materials: bool = True,
        extract_dimensions: bool = True,
        generate_macros: bool = False,
        split_files: bool = False,
    ) -> None:
        """Initialize XACRO generator.

        Args:
            pretty_print: If True, format XML with indentation
            advanced_mode: Enable advanced XACRO features
            extract_materials: Extract material colors as properties
            extract_dimensions: Extract common dimensions as properties
            generate_macros: Auto-generate macros for repeated patterns
            split_files: Split into multiple files (materials, macros, robot)
        """
        self.pretty_print = pretty_print
        self.advanced_mode = advanced_mode
        self.extract_materials = extract_materials if advanced_mode else False
        self.extract_dimensions = extract_dimensions if advanced_mode else False
        self.generate_macros = generate_macros if advanced_mode else False
        self.split_files = split_files if advanced_mode else False

        # Track extracted properties
        self.material_properties: dict[str, str] = {}
        self.dimension_properties: dict[str, float] = {}

    def generate(self, robot: Robot) -> str:
        """Generate XACRO XML string from robot.

        Args:
            robot: Robot model

        Returns:
            XACRO XML as string

        Raises:
            ValueError: If robot validation fails
        """
        # Validate robot structure
        errors = robot.validate_tree_structure()
        if errors:
            raise ValueError("Robot validation failed:\n" + "\n".join(errors))

        # Generate base URDF
        urdf_gen = URDFGenerator(pretty_print=False)
        urdf_string = urdf_gen.generate(robot)

        # Parse URDF
        root = ET.fromstring(urdf_string)

        # Add xacro namespace
        root.set("xmlns:xacro", "http://www.ros.org/wiki/xacro")

        # Reset property tracking
        self.material_properties = {}
        self.dimension_properties = {}

        # Build properties list
        properties: list[tuple[str, str]] = []

        # Always add robot name property
        properties.append(("robot_name", robot.name))
        root.set("name", "${robot_name}")

        if self.advanced_mode:
            # Extract materials
            if self.extract_materials:
                self._extract_material_properties(robot, root, properties)

            # Extract dimensions
            if self.extract_dimensions:
                self._extract_dimension_properties(robot, root, properties)

        # Insert all properties at the beginning
        for i, (prop_name, prop_value) in enumerate(properties):
            prop_elem = ET.Element("xacro:property", name=prop_name, value=str(prop_value))
            root.insert(i, prop_elem)

        # Convert to string
        return self._element_to_string(root)

    def write(self, robot: Robot, filepath: Path) -> None:
        """Write XACRO to file.

        Args:
            robot: Robot model
            filepath: Output file path
        """
        xacro_string = self.generate(robot)
        filepath.write_text(xacro_string, encoding="utf-8")

    def _extract_material_properties(
        self, robot: Robot, root: ET.Element, properties: list[tuple[str, str]]
    ) -> None:
        """Extract unique materials as XACRO properties.

        Args:
            robot: Robot model
            root: XML root element
            properties: List to append properties to
        """
        # Find all unique materials
        materials: dict[str, Material] = {}
        for link in robot.links:
            if link.visual and link.visual.material:
                mat = link.visual.material
                if mat.name not in materials:
                    materials[mat.name] = mat

        # Create properties for each unique material color
        for mat_name, mat in materials.items():
            prop_name = f"color_{mat_name.lower().replace(' ', '_')}"
            prop_value = f"{mat.color.r} {mat.color.g} {mat.color.b} {mat.color.a}"
            properties.append((prop_name, prop_value))
            self.material_properties[mat_name] = prop_name

        # Replace color values in XML with property references
        for material_elem in root.findall(".//material"):
            mat_name = material_elem.get("name")
            if mat_name and mat_name in self.material_properties:
                color_elem = material_elem.find("color")
                if color_elem is not None:
                    prop_ref = f"${{{self.material_properties[mat_name]}}}"
                    color_elem.set("rgba", prop_ref)

    def _extract_dimension_properties(
        self, robot: Robot, root: ET.Element, properties: list[tuple[str, str]]
    ) -> None:
        """Extract common dimensions as XACRO properties.

        Args:
            robot: Robot model
            root: XML root element
            properties: List to append properties to
        """
        # Track dimensions by type
        radii: dict[float, list[str]] = defaultdict(list)
        lengths: dict[float, list[str]] = defaultdict(list)
        sizes: dict[tuple[float, float, float], list[str]] = defaultdict(list)

        # Collect dimensions from all links
        for link in robot.links:
            for visual_or_collision in [link.visual, link.collision]:
                if not visual_or_collision:
                    continue

                geom = visual_or_collision.geometry
                link_id = link.name

                if geom.type == GeometryType.CYLINDER:
                    radii[geom.radius].append(f"{link_id}_radius")  # type: ignore
                    lengths[geom.length].append(f"{link_id}_length")  # type: ignore
                elif geom.type == GeometryType.SPHERE:
                    radii[geom.radius].append(f"{link_id}_radius")  # type: ignore
                elif geom.type == GeometryType.BOX:
                    size_tuple = (geom.size.x, geom.size.y, geom.size.z)  # type: ignore
                    sizes[size_tuple].append(f"{link_id}_size")

        # Create properties for dimensions used multiple times
        dim_counter = 0
        for radius, usage_list in radii.items():
            if len(usage_list) >= 2:  # Used by 2+ links
                prop_name = f"radius_{dim_counter}"
                properties.append((prop_name, str(radius)))
                self.dimension_properties[f"radius_{radius}"] = radius
                dim_counter += 1

        for length, usage_list in lengths.items():
            if len(usage_list) >= 2:
                prop_name = f"length_{dim_counter}"
                properties.append((prop_name, str(length)))
                self.dimension_properties[f"length_{length}"] = length
                dim_counter += 1

        # Note: Replacing dimension values in XML is complex
        # For now, we just create the properties
        # Full replacement would require tracking geometry elements

    def _element_to_string(self, element: ET.Element) -> str:
        """Convert XML element to string with pretty printing."""
        if self.pretty_print:
            rough_string = ET.tostring(element, encoding="unicode")
            reparsed = minidom.parseString(rough_string)
            pretty = reparsed.toprettyxml(indent="  ")

            # Clean up extra blank lines
            lines = [line for line in pretty.split("\n") if line.strip()]
            return "\n".join(lines) + "\n"
        else:
            return ET.tostring(element, encoding="unicode")


# Future enhancement: XACROMacroGenerator
# This would analyze the robot and auto-generate macros for repeated patterns
# Example: if multiple wheels have the same geometry, create a wheel macro
