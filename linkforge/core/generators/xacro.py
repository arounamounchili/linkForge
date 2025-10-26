"""XACRO generation from robot models.

XACRO (XML Macros) is an extension of URDF that supports:
- Variables/properties
- Macros for repeated elements
- Math expressions
- File includes

For Phase 2, we generate basic XACRO with simple macros.
Advanced features can be added in later phases.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path
from xml.dom import minidom

from ..models.robot import Robot
from .urdf import URDFGenerator


class XACROGenerator:
    """Generate XACRO from Robot model.

    For initial version, we generate XACRO with:
    - Robot name as property
    - Basic structure similar to URDF
    - Ready for manual macro expansion by users

    Future enhancements:
    - Auto-generate macros for repeated link/joint patterns
    - Material property variables
    - Conditional blocks
    - File splitting
    """

    def __init__(self, pretty_print: bool = True) -> None:
        """Initialize XACRO generator.

        Args:
            pretty_print: If True, format XML with indentation
        """
        self.pretty_print = pretty_print

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

        # For basic XACRO, we use URDF generator and add xacro namespace
        urdf_gen = URDFGenerator(pretty_print=False)
        urdf_string = urdf_gen.generate(robot)

        # Parse URDF and add xacro namespace
        root = ET.fromstring(urdf_string)

        # Add xacro namespace
        root.set("xmlns:xacro", "http://www.ros.org/wiki/xacro")

        # Add robot name as property
        # Insert at the beginning (after namespace)
        prop = ET.Element("xacro:property", name="robot_name", value=robot.name)
        root.insert(0, prop)

        # Update robot name to use property
        root.set("name", "${robot_name}")

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
