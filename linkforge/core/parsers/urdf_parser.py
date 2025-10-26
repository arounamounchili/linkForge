"""URDF XML parser to import robot models."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

from ..models import (
    Box,
    Capsule,
    Color,
    Collision,
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


def parse_vector3(text: str) -> Vector3:
    """Parse space-separated vector3 string."""
    parts = text.strip().split()
    return Vector3(float(parts[0]), float(parts[1]), float(parts[2]))


def parse_origin(elem: ET.Element | None) -> Transform:
    """Parse origin element to Transform."""
    if elem is None:
        return Transform.identity()

    xyz_text = elem.get("xyz", "0 0 0")
    rpy_text = elem.get("rpy", "0 0 0")

    xyz = parse_vector3(xyz_text)
    rpy = parse_vector3(rpy_text)

    return Transform(xyz=xyz, rpy=rpy)


def parse_geometry(geom_elem: ET.Element) -> Box | Cylinder | Sphere | Capsule | Mesh | None:
    """Parse geometry element."""
    # Check for box
    box = geom_elem.find("box")
    if box is not None:
        size = parse_vector3(box.get("size", "1 1 1"))
        return Box(size=size)

    # Check for cylinder
    cylinder = geom_elem.find("cylinder")
    if cylinder is not None:
        radius = float(cylinder.get("radius", "0.5"))
        length = float(cylinder.get("length", "1.0"))
        return Cylinder(radius=radius, length=length)

    # Check for sphere
    sphere = geom_elem.find("sphere")
    if sphere is not None:
        radius = float(sphere.get("radius", "0.5"))
        return Sphere(radius=radius)

    # Check for capsule
    capsule = geom_elem.find("capsule")
    if capsule is not None:
        radius = float(capsule.get("radius", "0.5"))
        length = float(capsule.get("length", "1.0"))
        return Capsule(radius=radius, length=length)

    # Check for mesh
    mesh = geom_elem.find("mesh")
    if mesh is not None:
        filename = mesh.get("filename", "")
        scale_text = mesh.get("scale", "1 1 1")
        scale = parse_vector3(scale_text)
        return Mesh(filepath=Path(filename), scale=scale)

    return None


def parse_material(mat_elem: ET.Element | None, materials: dict[str, Material]) -> Material | None:
    """Parse material element or reference."""
    if mat_elem is None:
        return None

    # Check if it's a reference
    mat_name = mat_elem.get("name", "")
    if mat_name and mat_name in materials:
        return materials[mat_name]

    # Parse color
    color_elem = mat_elem.find("color")
    if color_elem is not None:
        rgba_text = color_elem.get("rgba", "0.8 0.8 0.8 1.0")
        parts = rgba_text.strip().split()
        color = Color(
            r=float(parts[0]),
            g=float(parts[1]),
            b=float(parts[2]),
            a=float(parts[3]) if len(parts) > 3 else 1.0,
        )
        return Material(name=mat_name if mat_name else "default", color=color)

    return None


def parse_link(link_elem: ET.Element, materials: dict[str, Material]) -> Link:
    """Parse link element."""
    name = link_elem.get("name", "unnamed_link")

    # Parse visual
    visual = None
    visual_elem = link_elem.find("visual")
    if visual_elem is not None:
        origin = parse_origin(visual_elem.find("origin"))
        geom_elem = visual_elem.find("geometry")
        geometry = parse_geometry(geom_elem) if geom_elem is not None else None
        material = parse_material(visual_elem.find("material"), materials)

        if geometry:
            visual = Visual(geometry=geometry, origin=origin, material=material)

    # Parse collision
    collision = None
    collision_elem = link_elem.find("collision")
    if collision_elem is not None:
        origin = parse_origin(collision_elem.find("origin"))
        geom_elem = collision_elem.find("geometry")
        geometry = parse_geometry(geom_elem) if geom_elem is not None else None

        if geometry:
            collision = Collision(geometry=geometry, origin=origin)

    # Parse inertial
    inertial = None
    inertial_elem = link_elem.find("inertial")
    if inertial_elem is not None:
        origin = parse_origin(inertial_elem.find("origin"))

        mass_elem = inertial_elem.find("mass")
        mass = float(mass_elem.get("value", "1.0")) if mass_elem is not None else 1.0

        inertia_elem = inertial_elem.find("inertia")
        if inertia_elem is not None:
            inertia = InertiaTensor(
                ixx=float(inertia_elem.get("ixx", "1.0")),
                ixy=float(inertia_elem.get("ixy", "0.0")),
                ixz=float(inertia_elem.get("ixz", "0.0")),
                iyy=float(inertia_elem.get("iyy", "1.0")),
                iyz=float(inertia_elem.get("iyz", "0.0")),
                izz=float(inertia_elem.get("izz", "1.0")),
            )
            inertial = Inertial(mass=mass, origin=origin, inertia=inertia)

    return Link(name=name, visual=visual, collision=collision, inertial=inertial)


def parse_joint(joint_elem: ET.Element) -> Joint:
    """Parse joint element."""
    name = joint_elem.get("name", "unnamed_joint")
    joint_type_str = joint_elem.get("type", "fixed")

    # Map URDF type string to JointType enum
    type_map = {
        "revolute": JointType.REVOLUTE,
        "continuous": JointType.CONTINUOUS,
        "prismatic": JointType.PRISMATIC,
        "fixed": JointType.FIXED,
        "floating": JointType.FLOATING,
        "planar": JointType.PLANAR,
    }
    joint_type = type_map.get(joint_type_str.lower(), JointType.FIXED)

    # Parse parent and child
    parent_elem = joint_elem.find("parent")
    child_elem = joint_elem.find("child")
    parent = parent_elem.get("link", "") if parent_elem is not None else ""
    child = child_elem.get("link", "") if child_elem is not None else ""

    # Parse origin
    origin = parse_origin(joint_elem.find("origin"))

    # Parse axis
    axis_elem = joint_elem.find("axis")
    axis = parse_vector3(axis_elem.get("xyz", "0 0 1")) if axis_elem is not None else Vector3(0, 0, 1)

    # Parse limits
    limits = None
    limits_elem = joint_elem.find("limit")
    if limits_elem is not None:
        limits = JointLimits(
            lower=float(limits_elem.get("lower", "0")),
            upper=float(limits_elem.get("upper", "0")),
            effort=float(limits_elem.get("effort", "0")),
            velocity=float(limits_elem.get("velocity", "0")),
        )

    # Parse dynamics
    dynamics = None
    dynamics_elem = joint_elem.find("dynamics")
    if dynamics_elem is not None:
        dynamics = JointDynamics(
            damping=float(dynamics_elem.get("damping", "0")),
            friction=float(dynamics_elem.get("friction", "0")),
        )

    # Parse mimic
    mimic = None
    mimic_elem = joint_elem.find("mimic")
    if mimic_elem is not None:
        mimic = JointMimic(
            joint=mimic_elem.get("joint", ""),
            multiplier=float(mimic_elem.get("multiplier", "1.0")),
            offset=float(mimic_elem.get("offset", "0.0")),
        )

    return Joint(
        name=name,
        type=joint_type,
        parent=parent,
        child=child,
        origin=origin,
        axis=axis,
        limits=limits,
        dynamics=dynamics,
        mimic=mimic,
    )


def parse_urdf(filepath: Path) -> Robot:
    """Parse URDF file and return Robot model.

    Args:
        filepath: Path to URDF file

    Returns:
        Robot model

    Raises:
        FileNotFoundError: If URDF file doesn't exist
        ET.ParseError: If XML is malformed
    """
    if not filepath.exists():
        raise FileNotFoundError(f"URDF file not found: {filepath}")

    tree = ET.parse(filepath)
    root = tree.getroot()

    if root.tag != "robot":
        raise ValueError("Root element must be <robot>")

    robot_name = root.get("name", "imported_robot")
    robot = Robot(name=robot_name)

    # Parse global materials first
    materials: dict[str, Material] = {}
    for mat_elem in root.findall("material"):
        mat = parse_material(mat_elem, materials)
        if mat:
            materials[mat.name] = mat

    # Parse all links
    for link_elem in root.findall("link"):
        link = parse_link(link_elem, materials)
        robot.add_link(link)

    # Parse all joints
    for joint_elem in root.findall("joint"):
        joint = parse_joint(joint_elem)
        robot.add_joint(joint)

    return robot
