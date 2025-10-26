"""URDF XML generation from robot models."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path
from xml.dom import minidom

from ..models.geometry import Box, Capsule, Cylinder, Mesh, Sphere
from ..models.joint import Joint, JointType
from ..models.link import Collision, Inertial, Link, Visual
from ..models.material import Material
from ..models.robot import Robot


class URDFGenerator:
    """Generate URDF XML from Robot model."""

    def __init__(self, pretty_print: bool = True, urdf_path: Path | None = None) -> None:
        """Initialize URDF generator.

        Args:
            pretty_print: If True, format XML with indentation
            urdf_path: Path where URDF will be saved (for relative mesh paths)
        """
        self.pretty_print = pretty_print
        self.urdf_path = urdf_path

    def generate(self, robot: Robot) -> str:
        """Generate URDF XML string from robot.

        Args:
            robot: Robot model

        Returns:
            URDF XML as string

        Raises:
            ValueError: If robot validation fails
        """
        # Validate robot structure
        errors = robot.validate_tree_structure()
        if errors:
            raise ValueError("Robot validation failed:\n" + "\n".join(errors))

        # Create root element
        root = ET.Element("robot", name=robot.name)

        # Add materials (collect unique materials first)
        materials = self._collect_materials(robot)
        for material in materials.values():
            self._add_material_element(root, material)

        # Add links
        for link in robot.links:
            self._add_link_element(root, link)

        # Add joints
        for joint in robot.joints:
            self._add_joint_element(root, joint)

        # Convert to string
        return self._element_to_string(root)

    def write(self, robot: Robot, filepath: Path) -> None:
        """Write URDF to file.

        Args:
            robot: Robot model
            filepath: Output file path
        """
        urdf_string = self.generate(robot)
        filepath.write_text(urdf_string, encoding="utf-8")

    def _collect_materials(self, robot: Robot) -> dict[str, Material]:
        """Collect unique materials from all links."""
        materials: dict[str, Material] = {}

        for link in robot.links:
            if link.visual and link.visual.material:
                mat = link.visual.material
                if mat.name not in materials:
                    materials[mat.name] = mat

        return materials

    def _add_material_element(self, parent: ET.Element, material: Material) -> None:
        """Add material element to parent."""
        mat_elem = ET.SubElement(parent, "material", name=material.name)

        if material.color:
            ET.SubElement(mat_elem, "color", rgba=str(material.color))

        if material.texture:
            ET.SubElement(mat_elem, "texture", filename=material.texture)

    def _add_link_element(self, parent: ET.Element, link: Link) -> None:
        """Add link element to parent."""
        link_elem = ET.SubElement(parent, "link", name=link.name)

        # Visual
        if link.visual:
            self._add_visual_element(link_elem, link.visual)

        # Collision
        if link.collision:
            self._add_collision_element(link_elem, link.collision)

        # Inertial
        if link.inertial:
            self._add_inertial_element(link_elem, link.inertial)

    def _add_visual_element(self, parent: ET.Element, visual: Visual) -> None:
        """Add visual element to parent."""
        visual_elem = ET.SubElement(parent, "visual")

        if visual.name:
            visual_elem.set("name", visual.name)

        # Origin
        self._add_origin_element(visual_elem, visual.origin)

        # Geometry
        self._add_geometry_element(visual_elem, visual.geometry)

        # Material (reference if it has a name, inline if not)
        if visual.material:
            if visual.material.name:
                ET.SubElement(visual_elem, "material", name=visual.material.name)
            else:
                # Inline material (unnamed)
                mat_elem = ET.SubElement(visual_elem, "material", name="")
                if visual.material.color:
                    ET.SubElement(mat_elem, "color", rgba=str(visual.material.color))

    def _add_collision_element(self, parent: ET.Element, collision: Collision) -> None:
        """Add collision element to parent."""
        collision_elem = ET.SubElement(parent, "collision")

        if collision.name:
            collision_elem.set("name", collision.name)

        # Origin
        self._add_origin_element(collision_elem, collision.origin)

        # Geometry
        self._add_geometry_element(collision_elem, collision.geometry)

    def _add_inertial_element(self, parent: ET.Element, inertial: Inertial) -> None:
        """Add inertial element to parent."""
        inertial_elem = ET.SubElement(parent, "inertial")

        # Origin (COM position)
        self._add_origin_element(inertial_elem, inertial.origin)

        # Mass
        ET.SubElement(inertial_elem, "mass", value=str(inertial.mass))

        # Inertia tensor
        inertia = inertial.inertia
        ET.SubElement(
            inertial_elem,
            "inertia",
            ixx=str(inertia.ixx),
            ixy=str(inertia.ixy),
            ixz=str(inertia.ixz),
            iyy=str(inertia.iyy),
            iyz=str(inertia.iyz),
            izz=str(inertia.izz),
        )

    def _add_origin_element(self, parent: ET.Element, transform) -> None:
        """Add origin element to parent."""
        from ..models.geometry import Transform

        if transform != Transform.identity():
            ET.SubElement(parent, "origin", xyz=str(transform.xyz), rpy=str(transform.rpy))

    def _add_geometry_element(self, parent: ET.Element, geometry) -> None:
        """Add geometry element to parent."""
        geom_elem = ET.SubElement(parent, "geometry")

        if isinstance(geometry, Box):
            ET.SubElement(geom_elem, "box", size=str(geometry.size))

        elif isinstance(geometry, Cylinder):
            ET.SubElement(
                geom_elem, "cylinder", radius=str(geometry.radius), length=str(geometry.length)
            )

        elif isinstance(geometry, Sphere):
            ET.SubElement(geom_elem, "sphere", radius=str(geometry.radius))

        elif isinstance(geometry, Capsule):
            ET.SubElement(
                geom_elem, "capsule", radius=str(geometry.radius), length=str(geometry.length)
            )

        elif isinstance(geometry, Mesh):
            # Make mesh path relative to URDF location if possible
            mesh_path = geometry.filepath
            if self.urdf_path and mesh_path.is_absolute():
                try:
                    mesh_path = mesh_path.relative_to(self.urdf_path.parent)
                except ValueError:
                    # Can't make relative, use absolute
                    pass

            attrib = {"filename": str(mesh_path)}
            # Check if scale is not default (1.0, 1.0, 1.0)
            if geometry.scale.x != 1.0 or geometry.scale.y != 1.0 or geometry.scale.z != 1.0:
                attrib["scale"] = str(geometry.scale)
            ET.SubElement(geom_elem, "mesh", **attrib)

    def _add_joint_element(self, parent: ET.Element, joint: Joint) -> None:
        """Add joint element to parent."""
        joint_elem = ET.SubElement(parent, "joint", name=joint.name, type=joint.type.value)

        # Origin
        self._add_origin_element(joint_elem, joint.origin)

        # Parent and child
        ET.SubElement(joint_elem, "parent", link=joint.parent)
        ET.SubElement(joint_elem, "child", link=joint.child)

        # Axis (only for revolute, continuous, prismatic)
        if joint.type in (JointType.REVOLUTE, JointType.CONTINUOUS, JointType.PRISMATIC):
            ET.SubElement(joint_elem, "axis", xyz=str(joint.axis))

        # Limits
        if joint.limits:
            ET.SubElement(
                joint_elem,
                "limit",
                lower=str(joint.limits.lower),
                upper=str(joint.limits.upper),
                effort=str(joint.limits.effort),
                velocity=str(joint.limits.velocity),
            )

        # Dynamics
        if joint.dynamics:
            ET.SubElement(
                joint_elem,
                "dynamics",
                damping=str(joint.dynamics.damping),
                friction=str(joint.dynamics.friction),
            )

        # Calibration
        if joint.calibration:
            attrib = {}
            if joint.calibration.rising is not None:
                attrib["rising"] = str(joint.calibration.rising)
            if joint.calibration.falling is not None:
                attrib["falling"] = str(joint.calibration.falling)
            if attrib:
                ET.SubElement(joint_elem, "calibration", **attrib)

        # Mimic
        if joint.mimic:
            attrib = {"joint": joint.mimic.joint}
            if joint.mimic.multiplier != 1.0:
                attrib["multiplier"] = str(joint.mimic.multiplier)
            if joint.mimic.offset != 0.0:
                attrib["offset"] = str(joint.mimic.offset)
            ET.SubElement(joint_elem, "mimic", **attrib)

        # Safety controller
        if joint.safety_controller:
            ET.SubElement(
                joint_elem,
                "safety_controller",
                soft_lower_limit=str(joint.safety_controller.soft_lower_limit),
                soft_upper_limit=str(joint.safety_controller.soft_upper_limit),
                k_position=str(joint.safety_controller.k_position),
                k_velocity=str(joint.safety_controller.k_velocity),
            )

    def _element_to_string(self, element: ET.Element) -> str:
        """Convert XML element to string.

        Args:
            element: XML element

        Returns:
            XML string (pretty-printed if enabled)
        """
        if self.pretty_print:
            # Use minidom for pretty printing
            rough_string = ET.tostring(element, encoding="unicode")
            reparsed = minidom.parseString(rough_string)
            return reparsed.toprettyxml(indent="  ")
        else:
            return ET.tostring(element, encoding="unicode")
