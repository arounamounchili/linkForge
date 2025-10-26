"""Converters between Blender properties and Core models.

These functions bridge the gap between Blender's property system
and LinkForge's core data models.
"""

from __future__ import annotations

from pathlib import Path

try:
    import bpy
    from mathutils import Matrix, Vector
except ImportError:
    # Allow importing without Blender
    bpy = None  # type: ignore
    Matrix = Vector = None  # type: ignore

from ...core.models import (
    Box,
    Collision,
    Color,
    Cylinder,
    Geometry,
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
from ...core.physics import calculate_inertia


def blender_to_vector3(vec) -> Vector3:
    """Convert Blender Vector to Vector3.

    Args:
        vec: Blender mathutils.Vector

    Returns:
        Core Vector3
    """
    return Vector3(vec.x, vec.y, vec.z)


def matrix_to_transform(matrix) -> Transform:
    """Convert Blender 4x4 matrix to Transform.

    Args:
        matrix: Blender mathutils.Matrix (4x4)

    Returns:
        Core Transform with XYZ position and RPY rotation
    """
    if matrix is None or Matrix is None:
        return Transform.identity()

    # Extract translation
    translation = matrix.to_translation()
    xyz = Vector3(translation.x, translation.y, translation.z)

    # Extract rotation (Euler angles in radians)
    rotation = matrix.to_euler("XYZ")
    rpy = Vector3(rotation.x, rotation.y, rotation.z)

    return Transform(xyz=xyz, rpy=rpy)


def get_object_geometry(
    obj,
    geometry_type: str = "MESH",
    link_name: str | None = None,
    geom_purpose: str = "visual",
    meshes_dir: Path | None = None,
    mesh_format: str = "STL",
    simplify: bool = False,
    decimation_ratio: float = 0.5,
) -> Geometry | None:
    """Extract geometry from Blender object.

    Args:
        obj: Blender Object
        geometry_type: Type of geometry to extract (MESH, BOX, CYLINDER, SPHERE, CAPSULE)
        link_name: Name of the link (for mesh filename)
        geom_purpose: "visual" or "collision" (for mesh filename)
        meshes_dir: Directory to export mesh files to
        mesh_format: "STL" or "DAE"
        simplify: Whether to simplify mesh (for collision)
        decimation_ratio: Simplification ratio if simplify=True

    Returns:
        Core Geometry or None
    """
    if bpy is None or obj is None:
        return None

    if geometry_type == "MESH":
        # Export actual mesh file if meshes_dir is provided
        if meshes_dir and link_name and obj.type == 'MESH':
            from .mesh_export import export_link_mesh

            mesh_path = export_link_mesh(
                obj=obj,
                link_name=link_name,
                geometry_type=geom_purpose,
                mesh_format=mesh_format,
                meshes_dir=meshes_dir,
                simplify=simplify,
                decimation_ratio=decimation_ratio,
            )

            if mesh_path:
                # Return Mesh geometry with file path
                return Mesh(filepath=mesh_path, scale=Vector3(1.0, 1.0, 1.0))

        # Fallback: approximate with bounding box if export failed or not requested
        geometry_type = "BOX"

    if geometry_type == "BOX":
        # Use bounding box
        bbox = [Vector(corner) for corner in obj.bound_box]
        dimensions = obj.dimensions
        return Box(size=Vector3(dimensions.x, dimensions.y, dimensions.z))

    elif geometry_type == "CYLINDER":
        # Approximate with bounding cylinder
        dimensions = obj.dimensions
        radius = max(dimensions.x, dimensions.y) / 2.0
        length = dimensions.z
        return Cylinder(radius=radius, length=length)

    elif geometry_type == "SPHERE":
        # Approximate with bounding sphere
        radius = max(obj.dimensions) / 2.0
        return Sphere(radius=radius)

    elif geometry_type == "CAPSULE":
        # Approximate with bounding capsule
        dimensions = obj.dimensions
        radius = max(dimensions.x, dimensions.y) / 2.0
        length = dimensions.z - 2 * radius  # Cylinder portion length
        return Capsule(radius=radius, length=max(0.0, length))

    return None


def get_object_material(obj, props) -> Material | None:
    """Extract material from Blender object.

    Args:
        obj: Blender Object
        props: LinkPropertyGroup with material settings

    Returns:
        Core Material or None
    """
    if not props.use_material:
        return None

    # Determine material name
    mat_name = props.material_name if props.material_name else f"{obj.name}_material"

    # Get color based on source
    if props.material_source == "CUSTOM":
        # Use custom color picker
        color = Color(
            r=props.material_color[0],
            g=props.material_color[1],
            b=props.material_color[2],
            a=props.material_color[3],
        )
    else:
        # Extract from Blender material
        if obj.material_slots and obj.material_slots[0].material:
            blender_mat = obj.material_slots[0].material

            # Try to get color from Principled BSDF node (modern Blender)
            color = None
            if blender_mat.use_nodes and blender_mat.node_tree:
                # Find Principled BSDF node
                for node in blender_mat.node_tree.nodes:
                    if node.type == 'BSDF_PRINCIPLED':
                        # Get Base Color input (index 0)
                        base_color_input = node.inputs['Base Color']
                        base_color = base_color_input.default_value
                        color = Color(
                            r=base_color[0],
                            g=base_color[1],
                            b=base_color[2],
                            a=base_color[3] if len(base_color) > 3 else 1.0
                        )
                        break

            # Fallback to viewport display color if no node found
            if color is None:
                diffuse = blender_mat.diffuse_color
                color = Color(r=diffuse[0], g=diffuse[1], b=diffuse[2], a=diffuse[3])
        else:
            # No Blender material, use default gray
            color = Color(0.8, 0.8, 0.8, 1.0)

    return Material(name=mat_name, color=color)


def blender_link_to_core(
    obj,
    meshes_dir: Path | None = None,
    robot_props=None,
) -> Link | None:
    """Convert Blender object with LinkPropertyGroup to Core Link.

    Args:
        obj: Blender Object with linkforge property group
        meshes_dir: Optional directory for exporting mesh files
        robot_props: Robot property group with export settings

    Returns:
        Core Link model or None
    """
    if bpy is None or obj is None:
        return None

    props = obj.linkforge
    if not props.is_robot_link:
        return None

    link_name = props.link_name if props.link_name else obj.name

    # Extract material
    material = get_object_material(obj, props)

    # Get mesh format from robot props
    mesh_format = "STL"
    if robot_props and hasattr(robot_props, 'mesh_format'):
        mesh_format = robot_props.mesh_format

    # Visual geometry
    visual = None
    if props.use_visual_geometry:
        visual_geom = get_object_geometry(
            obj=obj,
            geometry_type=props.visual_geometry_type,
            link_name=link_name,
            geom_purpose="visual",
            meshes_dir=meshes_dir,
            mesh_format=mesh_format,
            simplify=False,  # Don't simplify visual meshes
            decimation_ratio=1.0,
        )
        if visual_geom:
            visual = Visual(
                geometry=visual_geom,
                origin=Transform.identity(),
                material=material,
            )

    # Collision geometry
    collision = None
    if props.export_collision:
        collision_geom = get_object_geometry(
            obj=obj,
            geometry_type=props.collision_geometry_type,
            link_name=link_name,
            geom_purpose="collision",
            meshes_dir=meshes_dir,
            mesh_format="STL",  # Always use STL for collision
            simplify=props.simplify_collision,
            decimation_ratio=props.collision_decimation_ratio,
        )
        if collision_geom:
            collision = Collision(geometry=collision_geom, origin=Transform.identity())

    # Inertial properties
    inertial = None
    if props.mass > 0:
        if props.use_auto_inertia and collision_geom:
            # Auto-calculate inertia from geometry
            # Note: Can't calculate inertia from Mesh, use bounding box instead
            if isinstance(collision_geom, Mesh):
                # Use bounding box for mesh inertia calculation
                dimensions = obj.dimensions
                bbox_geom = Box(size=Vector3(dimensions.x, dimensions.y, dimensions.z))
                inertia_tensor = calculate_inertia(bbox_geom, props.mass)
            else:
                # Calculate from primitive geometry
                inertia_tensor = calculate_inertia(collision_geom, props.mass)
        else:
            # Use manual inertia
            inertia_tensor = InertiaTensor(
                ixx=props.inertia_ixx,
                ixy=props.inertia_ixy,
                ixz=props.inertia_ixz,
                iyy=props.inertia_iyy,
                iyz=props.inertia_iyz,
                izz=props.inertia_izz,
            )

        inertial = Inertial(
            mass=props.mass,
            origin=Transform.identity(),
            inertia=inertia_tensor,
        )

    return Link(
        name=link_name,
        visual=visual,
        collision=collision,
        inertial=inertial,
    )


def blender_joint_to_core(obj) -> Joint | None:
    """Convert Blender Empty with JointPropertyGroup to Core Joint.

    Args:
        obj: Blender Empty object with linkforge_joint property group

    Returns:
        Core Joint model or None
    """
    if bpy is None or obj is None:
        return None

    props = obj.linkforge_joint
    if not props.is_robot_joint:
        return None

    joint_name = props.joint_name if props.joint_name else obj.name

    # Joint type
    joint_type = JointType(props.joint_type.lower())

    # Joint axis
    if props.axis == "X":
        axis = Vector3(1.0, 0.0, 0.0)
    elif props.axis == "Y":
        axis = Vector3(0.0, 1.0, 0.0)
    elif props.axis == "Z":
        axis = Vector3(0.0, 0.0, 1.0)
    else:  # CUSTOM
        axis = Vector3(props.custom_axis_x, props.custom_axis_y, props.custom_axis_z)

    # Joint origin (from Empty's transform)
    origin = matrix_to_transform(obj.matrix_world)

    # Joint limits
    limits = None
    if props.use_limits and joint_type in (JointType.REVOLUTE, JointType.PRISMATIC):
        limits = JointLimits(
            lower=props.limit_lower,
            upper=props.limit_upper,
            effort=props.limit_effort,
            velocity=props.limit_velocity,
        )

    # Dynamics
    dynamics = None
    if props.use_dynamics:
        dynamics = JointDynamics(
            damping=props.dynamics_damping,
            friction=props.dynamics_friction,
        )

    # Mimic
    mimic = None
    if props.use_mimic and props.mimic_joint:
        mimic = JointMimic(
            joint=props.mimic_joint,
            multiplier=props.mimic_multiplier,
            offset=props.mimic_offset,
        )

    return Joint(
        name=joint_name,
        type=joint_type,
        parent=props.parent_link,
        child=props.child_link,
        origin=origin,
        axis=axis,
        limits=limits,
        dynamics=dynamics,
        mimic=mimic,
    )


def scene_to_robot(context, meshes_dir: Path | None = None) -> Robot:
    """Convert entire Blender scene to Core Robot.

    Args:
        context: Blender context
        meshes_dir: Optional directory for exporting mesh files

    Returns:
        Core Robot model
    """
    if bpy is None or context is None:
        return Robot(name="empty_robot")

    scene = context.scene
    robot_props = scene.linkforge
    robot_name = robot_props.robot_name if robot_props.robot_name else "robot"

    robot = Robot(name=robot_name)

    # Collect all links
    for obj in scene.objects:
        if obj.linkforge.is_robot_link:
            link = blender_link_to_core(obj, meshes_dir, robot_props)
            if link:
                try:
                    robot.add_link(link)
                except ValueError as e:
                    print(f"Warning: Could not add link {link.name}: {e}")

    # Collect all joints
    for obj in scene.objects:
        if obj.type == "EMPTY" and obj.linkforge_joint.is_robot_joint:
            joint = blender_joint_to_core(obj)
            if joint:
                try:
                    robot.add_joint(joint)
                except ValueError as e:
                    print(f"Warning: Could not add joint {joint.name}: {e}")

    return robot
