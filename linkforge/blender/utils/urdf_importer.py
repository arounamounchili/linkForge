"""URDF importer utilities for creating Blender objects from URDF models."""

from __future__ import annotations

from pathlib import Path

try:
    import bpy
    import mathutils
except ImportError:
    bpy = None
    mathutils = None

from ...core.models import (
    Box,
    Capsule,
    Color,
    Cylinder,
    Joint,
    Link,
    Mesh,
    Robot,
    Sphere,
)


def create_material_from_color(color: Color, name: str):
    """Create Blender material from Color model.

    Args:
        color: Color model
        name: Material name

    Returns:
        Blender Material or None
    """
    if bpy is None:
        return None

    # Check if material already exists
    if name in bpy.data.materials:
        return bpy.data.materials[name]

    # Create new material
    mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True

    # Get Principled BSDF node
    nodes = mat.node_tree.nodes
    bsdf = None
    for node in nodes:
        if node.type == "BSDF_PRINCIPLED":
            bsdf = node
            break

    if bsdf:
        bsdf.inputs["Base Color"].default_value = (color.r, color.g, color.b, color.a)

    return mat


def create_primitive_mesh(geometry, name: str):
    """Create Blender mesh object from primitive geometry.

    Args:
        geometry: Box, Cylinder, Sphere, or Capsule
        name: Object name

    Returns:
        Blender Object or None
    """
    if bpy is None:
        return None

    # Deselect all first
    bpy.ops.object.select_all(action="DESELECT")

    obj = None

    try:
        if isinstance(geometry, Box):
            bpy.ops.mesh.primitive_cube_add(location=(0, 0, 0))
            obj = bpy.context.active_object
            if obj:
                obj.name = name
                obj.dimensions = (geometry.size.x, geometry.size.y, geometry.size.z)

        elif isinstance(geometry, Cylinder):
            bpy.ops.mesh.primitive_cylinder_add(location=(0, 0, 0))
            obj = bpy.context.active_object
            if obj:
                obj.name = name
                obj.dimensions = (geometry.radius * 2, geometry.radius * 2, geometry.length)

        elif isinstance(geometry, Sphere):
            bpy.ops.mesh.primitive_uv_sphere_add(location=(0, 0, 0))
            obj = bpy.context.active_object
            if obj:
                obj.name = name
                obj.dimensions = (geometry.radius * 2, geometry.radius * 2, geometry.radius * 2)

        elif isinstance(geometry, Capsule):
            # Capsule: cylinder with hemispherical caps
            # Approximate with UV sphere for now
            bpy.ops.mesh.primitive_uv_sphere_add(location=(0, 0, 0))
            obj = bpy.context.active_object
            if obj:
                obj.name = name
                # Scale to approximate capsule
                obj.dimensions = (
                    geometry.radius * 2,
                    geometry.radius * 2,
                    geometry.length + geometry.radius * 2,
                )

        else:
            return None

    except Exception as e:
        print(f"ERROR creating primitive geometry: {e}")
        return None

    return obj


def import_mesh_file(mesh_path: Path, name: str):
    """Import mesh file into Blender.

    Args:
        mesh_path: Path to mesh file (OBJ, STL, etc.)
        name: Object name

    Returns:
        Blender Object or None
    """
    if bpy is None or not mesh_path.exists():
        return None

    # Deselect all
    bpy.ops.object.select_all(action="DESELECT")

    # Import based on file extension
    ext = mesh_path.suffix.lower()

    try:
        if ext == ".obj":
            bpy.ops.wm.obj_import(filepath=str(mesh_path))
        elif ext == ".stl":
            bpy.ops.wm.stl_import(filepath=str(mesh_path))
        elif ext == ".dae":
            # DAE might not be available in all Blender versions
            bpy.ops.wm.collada_import(filepath=str(mesh_path))
        else:
            return None

        # Get imported object (should be selected)
        imported_objects = [obj for obj in bpy.context.selected_objects]
        if imported_objects:
            obj = imported_objects[0]
            obj.name = name
            return obj

    except Exception as e:
        print(f"ERROR importing mesh {mesh_path.name}: {e}")
        return None

    return None


def create_link_object(link: Link, urdf_dir: Path, collection=None) -> object | None:
    """Create Blender object from Link model.

    Args:
        link: Link model
        urdf_dir: Directory containing URDF file (for resolving mesh paths)
        collection: Blender Collection to add object to

    Returns:
        Blender Object or None (returns the visual mesh object with link properties)
    """
    if bpy is None:
        return None

    # Will store the main link object (visual mesh or empty if no visual)
    link_obj = None

    # If link has visual geometry, create it
    if link.visual:
        visual = link.visual
        visual_obj = None

        # Create geometry
        if isinstance(visual.geometry, Mesh):
            # Resolve mesh path relative to URDF directory
            mesh_path = urdf_dir / visual.geometry.filepath
            if not mesh_path.exists():
                # Try as absolute path
                mesh_path = visual.geometry.filepath

            visual_obj = import_mesh_file(mesh_path, link.name)

            # Apply scale from URDF
            if visual_obj and visual.geometry.scale:
                scale = visual.geometry.scale
                visual_obj.scale = (scale.x, scale.y, scale.z)
        else:
            # Create primitive geometry
            visual_obj = create_primitive_mesh(visual.geometry, link.name)

        if visual_obj:
            # Apply visual origin transform (offset from link frame)
            if visual.origin:
                origin = visual.origin
                visual_obj.location = (origin.xyz.x, origin.xyz.y, origin.xyz.z)
                visual_obj.rotation_euler = (origin.rpy.x, origin.rpy.y, origin.rpy.z)
            else:
                visual_obj.location = (0, 0, 0)
                visual_obj.rotation_euler = (0, 0, 0)

            # Add visual mesh to collection
            if collection:
                for coll in list(visual_obj.users_collection):
                    if coll != collection:
                        coll.objects.unlink(visual_obj)
                if visual_obj not in collection.objects[:]:
                    collection.objects.link(visual_obj)

            # Apply material to visual mesh
            if link.visual and link.visual.material and link.visual.material.color:
                mat = create_material_from_color(
                    link.visual.material.color, link.visual.material.name
                )
                if mat and visual_obj.data:
                    visual_obj.data.materials.clear()
                    visual_obj.data.materials.append(mat)

            # This visual object IS the link object
            link_obj = visual_obj

    else:
        # No visual geometry - create an Empty to represent the link
        bpy.ops.object.empty_add(type="SPHERE", location=(0, 0, 0))
        link_obj = bpy.context.active_object
        link_obj.name = link.name
        link_obj.empty_display_size = 0.05

        # Add to collection
        if collection:
            for coll in list(link_obj.users_collection):
                if coll != collection:
                    coll.objects.unlink(link_obj)
            if link_obj not in collection.objects[:]:
                collection.objects.link(link_obj)

    # Set link properties on the main link object (visual mesh or empty)
    if link_obj and hasattr(link_obj, "linkforge"):
        props = link_obj.linkforge
        props.is_robot_link = True
        props.link_name = link.name

        # Set mass and inertia
        if link.inertial:
            props.mass = link.inertial.mass
            if link.inertial.inertia:
                inertia = link.inertial.inertia
                props.inertia_ixx = inertia.ixx
                props.inertia_ixy = inertia.ixy
                props.inertia_ixz = inertia.ixz
                props.inertia_iyy = inertia.iyy
                props.inertia_iyz = inertia.iyz
                props.inertia_izz = inertia.izz
                props.use_auto_inertia = False  # Don't recalculate

        # Set geometry type
        if link.visual:
            visual = link.visual
            if isinstance(visual.geometry, Box):
                props.visual_geometry_type = "BOX"
            elif isinstance(visual.geometry, Cylinder):
                props.visual_geometry_type = "CYLINDER"
            elif isinstance(visual.geometry, Sphere):
                props.visual_geometry_type = "SPHERE"
            elif isinstance(visual.geometry, Capsule):
                props.visual_geometry_type = "CAPSULE"
            elif isinstance(visual.geometry, Mesh):
                props.visual_geometry_type = "MESH"

        # Set collision geometry (if different from visual)
        if link.collision and link.collision.geometry:
            collision_geom = link.collision.geometry
            if isinstance(collision_geom, Box):
                props.collision_geometry_type = "BOX"
            elif isinstance(collision_geom, Cylinder):
                props.collision_geometry_type = "CYLINDER"
            elif isinstance(collision_geom, Sphere):
                props.collision_geometry_type = "SPHERE"
            elif isinstance(collision_geom, Capsule):
                props.collision_geometry_type = "CAPSULE"
            elif isinstance(collision_geom, Mesh):
                props.collision_geometry_type = "MESH"
        else:
            # Use visual geometry for collision
            if link.visual:
                props.collision_geometry_type = props.visual_geometry_type

    return link_obj


def create_joint_object(joint: Joint, link_objects: dict, collection=None) -> object | None:
    """Create Empty object from Joint model.

    Args:
        joint: Joint model
        link_objects: Dictionary mapping link names to Blender objects
        collection: Blender Collection to add object to

    Returns:
        Blender Empty object or None
    """
    if bpy is None:
        return None

    # Create Empty object with RViz-style 3-axis visualization
    bpy.ops.object.empty_add(type="ARROWS", location=(0, 0, 0))
    empty = bpy.context.active_object
    empty.name = joint.name  # Clean name without "joint_" prefix
    empty.empty_display_size = 0.15  # Larger and more visible

    # Set joint properties
    if hasattr(empty, "linkforge_joint"):
        props = empty.linkforge_joint
        props.is_robot_joint = True
        props.joint_name = joint.name

        # Set joint type
        type_map = {
            "REVOLUTE": "REVOLUTE",
            "CONTINUOUS": "CONTINUOUS",
            "PRISMATIC": "PRISMATIC",
            "FIXED": "FIXED",
            "FLOATING": "FLOATING",
            "PLANAR": "PLANAR",
        }
        props.joint_type = type_map.get(joint.type.name, "FIXED")

        # Set parent and child links
        props.parent_link = joint.parent
        props.child_link = joint.child

        # Set axis
        if joint.axis:
            # Check if it's a standard axis (X, Y, or Z)
            if joint.axis.x == 1.0 and joint.axis.y == 0.0 and joint.axis.z == 0.0:
                props.axis = "X"
            elif joint.axis.x == 0.0 and joint.axis.y == 1.0 and joint.axis.z == 0.0:
                props.axis = "Y"
            elif joint.axis.x == 0.0 and joint.axis.y == 0.0 and joint.axis.z == 1.0:
                props.axis = "Z"
            else:
                # Custom axis
                props.axis = "CUSTOM"
                props.custom_axis_x = joint.axis.x
                props.custom_axis_y = joint.axis.y
                props.custom_axis_z = joint.axis.z

        # Set limits
        if joint.limits:
            props.use_limits = True
            props.limit_lower = joint.limits.lower
            props.limit_upper = joint.limits.upper
            props.limit_effort = joint.limits.effort
            props.limit_velocity = joint.limits.velocity

        # Set dynamics
        if joint.dynamics:
            props.use_dynamics = True
            props.dynamics_damping = joint.dynamics.damping
            props.dynamics_friction = joint.dynamics.friction

        # Set mimic
        if joint.mimic:
            props.use_mimic = True
            props.mimic_joint = joint.mimic.joint
            props.mimic_multiplier = joint.mimic.multiplier
            props.mimic_offset = joint.mimic.offset

    # Set up parent-child relationship in Blender
    if joint.parent in link_objects and joint.child in link_objects:
        parent_obj = link_objects[joint.parent]
        child_obj = link_objects[joint.child]

        # Parent the joint Empty to the parent link
        empty.parent = parent_obj
        empty.matrix_parent_inverse = parent_obj.matrix_world.inverted()

        # Apply joint origin transform (offset from parent link frame)
        if joint.origin:
            origin = joint.origin
            empty.location = (origin.xyz.x, origin.xyz.y, origin.xyz.z)
            empty.rotation_euler = (origin.rpy.x, origin.rpy.y, origin.rpy.z)

        # Parent the child link to the joint Empty
        # Child link should maintain its visual origin transform
        # Store the current transform before parenting
        current_loc = child_obj.location.copy()
        current_rot = child_obj.rotation_euler.copy()

        child_obj.parent = empty
        child_obj.matrix_parent_inverse = empty.matrix_world.inverted()

        # Restore the visual origin transform (don't reset to 0!)
        child_obj.location = current_loc
        child_obj.rotation_euler = current_rot

    # Add to collection
    if collection:
        # Link to target collection if not already there
        if empty not in collection.objects[:]:
            collection.objects.link(empty)
        # Remove from all other collections
        for coll in empty.users_collection[:]:
            if coll != collection:
                coll.objects.unlink(empty)

    # Set visibility based on "Show Joint Axes" property
    if bpy and hasattr(bpy.context.scene, "linkforge"):
        show_joint_axes = bpy.context.scene.linkforge.show_joint_axes
        empty.hide_viewport = not show_joint_axes
        empty.hide_render = True  # Always hide from render

    return empty


def import_robot_to_scene(robot: Robot, urdf_path: Path, context) -> bool:
    """Import Robot model to Blender scene.

    Args:
        robot: Robot model
        urdf_path: Path to URDF file
        context: Blender context

    Returns:
        True if import succeeded, False otherwise
    """
    if bpy is None:
        return False

    # Set robot name in scene properties
    scene = context.scene
    if hasattr(scene, "linkforge"):
        scene.linkforge.robot_name = robot.name

    # Create collection for this robot
    collection = bpy.data.collections.new(robot.name)
    context.scene.collection.children.link(collection)

    # Get URDF directory for resolving mesh paths
    urdf_dir = urdf_path.parent

    # Create link objects
    link_objects = {}
    print(f"Importing robot '{robot.name}': {len(robot.links)} links, {len(robot.joints)} joints")

    for link in robot.links:
        obj = create_link_object(link, urdf_dir, collection)
        if obj:
            link_objects[link.name] = obj

    # Create joint objects
    for joint in robot.joints:
        create_joint_object(joint, link_objects, collection)

    print(f"Import complete: {len(link_objects)} links created")
    return True
