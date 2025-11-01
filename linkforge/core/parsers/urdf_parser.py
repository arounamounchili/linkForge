"""URDF XML parser to import robot models."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

from ..models import (
    Box,
    Capsule,
    Collision,
    Color,
    Cylinder,
    GazeboElement,
    GazeboPlugin,
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
    Transmission,
    TransmissionActuator,
    TransmissionJoint,
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
    axis = (
        parse_vector3(axis_elem.get("xyz", "0 0 1")) if axis_elem is not None else Vector3(0, 0, 1)
    )

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


def parse_transmission(trans_elem: ET.Element) -> Transmission:
    """Parse transmission element.

    Args:
        trans_elem: <transmission> XML element

    Returns:
        Transmission model
    """
    name = trans_elem.get("name", "")
    trans_type = trans_elem.findtext("type", "")

    # Parse joints
    joints: list[TransmissionJoint] = []
    for joint_elem in trans_elem.findall("joint"):
        joint_name = joint_elem.get("name", "")

        # Parse hardware interfaces
        interfaces = []
        for iface_elem in joint_elem.findall("hardwareInterface"):
            interfaces.append(iface_elem.text or "position")

        if not interfaces:
            interfaces = ["position"]  # Default

        # Parse mechanical reduction (optional)
        reduction = 1.0
        reduction_elem = joint_elem.find("mechanicalReduction")
        if reduction_elem is not None and reduction_elem.text:
            reduction = float(reduction_elem.text)

        # Parse offset (optional)
        offset = 0.0
        offset_elem = joint_elem.find("offset")
        if offset_elem is not None and offset_elem.text:
            offset = float(offset_elem.text)

        joints.append(
            TransmissionJoint(
                name=joint_name,
                hardware_interfaces=interfaces,
                mechanical_reduction=reduction,
                offset=offset,
            )
        )

    # Parse actuators
    actuators: list[TransmissionActuator] = []
    for actuator_elem in trans_elem.findall("actuator"):
        actuator_name = actuator_elem.get("name", "")

        # Parse hardware interfaces
        interfaces = []
        for iface_elem in actuator_elem.findall("hardwareInterface"):
            interfaces.append(iface_elem.text or "position")

        if not interfaces:
            interfaces = ["position"]  # Default

        # Parse mechanical reduction (optional)
        reduction = 1.0
        reduction_elem = actuator_elem.find("mechanicalReduction")
        if reduction_elem is not None and reduction_elem.text:
            reduction = float(reduction_elem.text)

        actuators.append(
            TransmissionActuator(
                name=actuator_name,
                hardware_interfaces=interfaces,
                mechanical_reduction=reduction,
            )
        )

    return Transmission(
        name=name,
        type=trans_type,
        joints=joints,
        actuators=actuators,
    )


def parse_gazebo_plugin(plugin_elem: ET.Element) -> GazeboPlugin:
    """Parse Gazebo plugin element.

    Args:
        plugin_elem: <plugin> XML element

    Returns:
        GazeboPlugin model
    """
    name = plugin_elem.get("name", "")
    filename = plugin_elem.get("filename", "")

    # Parse all sub-elements as parameters
    parameters: dict[str, str] = {}
    for child in plugin_elem:
        if child.text:
            parameters[child.tag] = child.text.strip()

    return GazeboPlugin(name=name, filename=filename, parameters=parameters)


def parse_gazebo_element(gazebo_elem: ET.Element) -> GazeboElement:
    """Parse Gazebo extension element.

    Args:
        gazebo_elem: <gazebo> XML element

    Returns:
        GazeboElement model
    """
    reference = gazebo_elem.get("reference", None)

    # Parse properties as dict
    properties: dict[str, str] = {}

    # Parse plugins
    plugins: list[GazeboPlugin] = []
    for plugin_elem in gazebo_elem.findall("plugin"):
        plugins.append(parse_gazebo_plugin(plugin_elem))

    # Helper functions for parsing optional elements
    def _parse_optional_bool(elem: ET.Element, tag: str, default: str = "false") -> bool | None:
        """Parse optional boolean element."""
        if elem.find(tag) is not None:
            return elem.findtext(tag, default).lower() == "true"
        return None

    def _parse_optional_float(elem: ET.Element, tag: str, default: str = "0") -> float | None:
        """Parse optional float element."""
        if elem.find(tag) is not None:
            return float(elem.findtext(tag, default))
        return None

    # Parse common properties
    material = gazebo_elem.findtext("material")

    # Parse boolean properties
    self_collide = _parse_optional_bool(gazebo_elem, "selfCollide")
    static = _parse_optional_bool(gazebo_elem, "static")
    gravity = _parse_optional_bool(gazebo_elem, "gravity", "true")
    provide_feedback = _parse_optional_bool(gazebo_elem, "provideFeedback")
    implicit_spring_damper = _parse_optional_bool(gazebo_elem, "implicitSpringDamper")

    # Parse numeric properties
    mu1 = _parse_optional_float(gazebo_elem, "mu1")
    mu2 = _parse_optional_float(gazebo_elem, "mu2")
    kp = _parse_optional_float(gazebo_elem, "kp")
    kd = _parse_optional_float(gazebo_elem, "kd")
    stop_cfm = _parse_optional_float(gazebo_elem, "stopCfm")
    stop_erp = _parse_optional_float(gazebo_elem, "stopErp")

    # Store any other elements as properties
    for child in gazebo_elem:
        if child.tag not in [
            "plugin",
            "material",
            "selfCollide",
            "static",
            "gravity",
            "provideFeedback",
            "implicitSpringDamper",
            "mu1",
            "mu2",
            "kp",
            "kd",
            "stopCfm",
            "stopErp",
        ]:
            if child.text:
                properties[child.tag] = child.text.strip()

    return GazeboElement(
        reference=reference,
        properties=properties,
        plugins=plugins,
        material=material,
        self_collide=self_collide,
        static=static,
        gravity=gravity,
        stop_cfm=stop_cfm,
        stop_erp=stop_erp,
        provide_feedback=provide_feedback,
        implicit_spring_damper=implicit_spring_damper,
        mu1=mu1,
        mu2=mu2,
        kp=kp,
        kd=kd,
    )


def _detect_xacro_file(root: ET.Element, filepath: Path) -> None:
    """Detect if file is XACRO and raise helpful error.

    This function is called by parse_urdf() to prevent attempting to parse
    raw XACRO files. XACRO files should be handled by the import operator
    which converts them to URDF first using xacrodoc.

    Args:
        root: XML root element
        filepath: Path to file being parsed

    Raises:
        ValueError: If XACRO features are detected in this parser
            (XACRO files should use the "Import Robot" operator instead)
    """
    # Check for .xacro extension
    is_xacro_extension = filepath.suffix.lower() in [".xacro", ".urdf.xacro"]

    # Check file content for xmlns:xacro (ElementTree may not preserve it)
    has_xacro_namespace = False
    try:
        content = filepath.read_text(encoding="utf-8")
        has_xacro_namespace = "xmlns:xacro" in content
    except Exception:
        pass

    # Check for xacro elements in root
    xacro_elements = []
    for child in root:
        tag = child.tag
        # Handle both namespaced and non-namespaced tags
        if "xacro:" in tag or (isinstance(tag, str) and tag.startswith("{") and "xacro" in tag):
            element_name = tag.split("}")[-1] if "}" in tag else tag.split("xacro:")[-1]
            xacro_elements.append(element_name)

    # Check for xacro substitutions in attributes
    has_substitutions = False
    for elem in root.iter():
        for attr_value in elem.attrib.values():
            if isinstance(attr_value, str) and ("${" in attr_value or "$(" in attr_value):
                has_substitutions = True
                break
        if has_substitutions:
            break

    # Raise error if XACRO features detected
    if is_xacro_extension or has_xacro_namespace or xacro_elements or has_substitutions:
        error_msg = (
            f"XACRO file detected: {filepath.name}\n\n"
            "This parser handles URDF files only. For XACRO files, use the 'Import Robot' "
            "operator in Blender which automatically converts XACRO to URDF.\n\n"
            "If using this parser programmatically, convert XACRO to URDF first:\n"
            "  1. Use parse_urdf_string() with xacrodoc:\n"
            "     from xacrodoc import XacroDoc\n"
            "     doc = XacroDoc.from_file('{filename}')\n"
            "     robot = parse_urdf_string(doc.to_urdf_string())\n\n"
            "  2. Use the standalone converter:\n"
            "     python tools/convert_xacro.py {filename}\n"
        ).format(filename=filepath.name)

        if xacro_elements:
            error_msg += f"\n\nDetected XACRO features: {', '.join(set(xacro_elements))}"

        raise ValueError(error_msg)


def parse_urdf(filepath: Path) -> Robot:
    """Parse URDF file and return Robot model.

    Note: This function parses URDF files only. For XACRO files,
    use parse_urdf_string() with xacrodoc, or use the Blender
    "Import Robot" operator which handles XACRO automatically.

    Args:
        filepath: Path to URDF file

    Returns:
        Robot model

    Raises:
        FileNotFoundError: If URDF file doesn't exist
        ET.ParseError: If XML is malformed
        ValueError: If XACRO file is detected (use "Import Robot" operator instead)
    """
    if not filepath.exists():
        raise FileNotFoundError(f"URDF file not found: {filepath}")

    tree = ET.parse(filepath)
    root = tree.getroot()

    if root.tag != "robot":
        raise ValueError("Root element must be <robot>")

    # Detect XACRO files and provide helpful error
    _detect_xacro_file(root, filepath)

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

    # Parse all transmissions
    for trans_elem in root.findall("transmission"):
        transmission = parse_transmission(trans_elem)
        robot.transmissions.append(transmission)  # Direct append, validation done by Robot model

    # Parse all Gazebo elements
    for gazebo_elem in root.findall("gazebo"):
        gazebo_element = parse_gazebo_element(gazebo_elem)
        robot.gazebo_elements.append(gazebo_element)

    return robot


def parse_urdf_string(urdf_string: str) -> Robot:
    """Parse URDF from XML string instead of file.

    This function is used for parsing URDF content that has been generated
    or converted from other formats (e.g., XACRO) in memory.

    Args:
        urdf_string: URDF XML content as string

    Returns:
        Robot model

    Raises:
        ET.ParseError: If XML is malformed
        ValueError: If root element is not <robot> or URDF is invalid

    Example:
        >>> urdf_xml = '<robot name="test">...</robot>'
        >>> robot = parse_urdf_string(urdf_xml)
        >>> print(robot.name)
        test
    """
    # Parse XML string
    root = ET.fromstring(urdf_string)

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

    # Parse all transmissions
    for trans_elem in root.findall("transmission"):
        transmission = parse_transmission(trans_elem)
        robot.transmissions.append(transmission)

    # Parse all Gazebo elements
    for gazebo_elem in root.findall("gazebo"):
        gazebo_element = parse_gazebo_element(gazebo_elem)
        robot.gazebo_elements.append(gazebo_element)

    return robot
