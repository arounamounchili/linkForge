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
from ..models.link import Link
from ..models.joint import Joint
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
        self.generated_macros: list[dict[str, Any]] = []

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
        self.generated_macros = []

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

            # Generate macros for repeated patterns
            if self.generate_macros:
                self._generate_macros(robot, root)

        # Insert all properties at the beginning
        insert_index = 0
        for prop_name, prop_value in properties:
            prop_elem = ET.Element("xacro:property", name=prop_name, value=str(prop_value))
            root.insert(insert_index, prop_elem)
            insert_index += 1

        # Insert generated macros after properties
        for macro_info in self.generated_macros:
            root.insert(insert_index, macro_info["element"])
            insert_index += 1

        # Convert to string
        return self._element_to_string(root)

    def write(self, robot: Robot, filepath: Path) -> None:
        """Write XACRO to file.

        If split_files is enabled, creates multiple files:
        - robot.xacro (main file with includes)
        - materials.xacro (material definitions)
        - macros.xacro (macro definitions)

        Args:
            robot: Robot model
            filepath: Output file path (for main robot.xacro)
        """
        if self.split_files:
            self._write_split_files(robot, filepath)
        else:
            xacro_string = self.generate(robot)
            filepath.write_text(xacro_string, encoding="utf-8")

    def _write_split_files(self, robot: Robot, main_filepath: Path) -> None:
        """Write robot to multiple XACRO files.

        Args:
            robot: Robot model
            main_filepath: Path for main robot.xacro file
        """
        # Generate full XACRO first
        full_xacro = self.generate(robot)
        root = ET.fromstring(full_xacro)

        # Create separate files
        base_dir = main_filepath.parent
        robot_name = main_filepath.stem

        # Extract top-level materials (not nested in links)
        materials_root = ET.Element("robot")
        materials_root.set("xmlns:xacro", "http://www.ros.org/wiki/xacro")
        for mat_elem in list(root.findall("material")):
            materials_root.append(mat_elem)
            root.remove(mat_elem)

        # Extract macros (top-level)
        macros_root = ET.Element("robot")
        macros_root.set("xmlns:xacro", "http://www.ros.org/wiki/xacro")
        for macro_elem in list(root.findall("xacro:macro", {"xacro": "http://www.ros.org/wiki/xacro"})):
            macros_root.append(macro_elem)
            root.remove(macro_elem)

        # Create main file with includes
        main_root = ET.Element("robot")
        main_root.set("xmlns:xacro", "http://www.ros.org/wiki/xacro")
        main_root.set("name", root.get("name", robot.name))

        # Add includes at the top
        if len(materials_root) > 0:
            include_mat = ET.Element("xacro:include")
            include_mat.set("filename", f"{robot_name}_materials.xacro")
            main_root.insert(0, include_mat)

        if len(macros_root) > 0:
            include_mac = ET.Element("xacro:include")
            include_mac.set("filename", f"{robot_name}_macros.xacro")
            main_root.insert(1 if len(materials_root) > 0 else 0, include_mac)

        # Copy properties and remaining content
        for child in root:
            main_root.append(child)

        # Write files
        if len(materials_root) > 0:
            mat_path = base_dir / f"{robot_name}_materials.xacro"
            mat_path.write_text(self._element_to_string(materials_root), encoding="utf-8")

        if len(macros_root) > 0:
            mac_path = base_dir / f"{robot_name}_macros.xacro"
            mac_path.write_text(self._element_to_string(macros_root), encoding="utf-8")

        main_filepath.write_text(self._element_to_string(main_root), encoding="utf-8")

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

    def _generate_macros(self, robot: Robot, root: ET.Element) -> None:
        """Generate macros for repeated link patterns.

        Args:
            robot: Robot model
            root: XML root element
        """
        # Group links by geometry signature
        link_groups: dict[str, list[tuple[Link, Joint | None]]] = defaultdict(list)

        for link in robot.links:
            # Create signature based on geometry
            signature = self._get_link_signature(link)
            if signature:
                # Find associated joint
                joint = None
                for j in robot.joints:
                    if j.child == link.name:
                        joint = j
                        break
                link_groups[signature].append((link, joint))

        # Generate macros for groups with 2+ members
        for signature, group in link_groups.items():
            if len(group) >= 2:
                self._create_macro_for_group(signature, group, root)

    def _get_link_signature(self, link: Link) -> str | None:
        """Create a signature string for a link based on its geometry.

        Args:
            link: Link to analyze

        Returns:
            Signature string or None if link has no visual geometry
        """
        if not link.visual or not link.visual.geometry:
            return None

        geom = link.visual.geometry
        parts = [geom.type.value]

        # Add geometry dimensions to signature
        if geom.type == GeometryType.BOX:
            parts.extend([f"box", f"{geom.size.x:.3f}", f"{geom.size.y:.3f}", f"{geom.size.z:.3f}"])  # type: ignore
        elif geom.type == GeometryType.CYLINDER:
            parts.extend([f"cyl", f"{geom.radius:.3f}", f"{geom.length:.3f}"])  # type: ignore
        elif geom.type == GeometryType.SPHERE:
            parts.extend([f"sph", f"{geom.radius:.3f}"])  # type: ignore

        return "_".join(parts)

    def _create_macro_for_group(
        self, signature: str, group: list[tuple[Link, Joint | None]], root: ET.Element
    ) -> None:
        """Create a macro for a group of similar links.

        Args:
            signature: Group signature
            group: List of (link, joint) tuples
            root: XML root element
        """
        # Take first link as template
        template_link, template_joint = group[0]

        # Create macro name from signature
        macro_name = signature.split("_")[1] if "_" in signature else "link"
        macro_name = f"{macro_name}_macro"

        # Create macro element
        macro_elem = ET.Element("xacro:macro")
        macro_elem.set("name", macro_name)
        macro_elem.set("params", "name parent xyz rpy")

        # Add comment
        comment = ET.Comment(f" Macro for {len(group)} similar {macro_name}s ")
        macro_elem.append(comment)

        # Store macro info
        self.generated_macros.append({
            "element": macro_elem,
            "name": macro_name,
            "instances": [link.name for link, _ in group],
        })

        # Note: Full macro content generation would require converting link/joint XML
        # For now, we just create the macro structure as a placeholder
        # In a full implementation, we'd:
        # 1. Generate link element with parameterized name
        # 2. Generate joint element with parameterized parent, xyz, rpy
        # 3. Replace instances in root with macro calls

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
