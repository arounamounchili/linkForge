"""3D gizmos for visualizing joint axes in the viewport.

This module provides RViz-style RGB axis visualization for robot joints:
- Red = X axis (with arrow head)
- Green = Y axis (with arrow head)
- Blue = Z axis (with arrow head)

Each axis is drawn as a solid colored line with an arrow cone at the tip,
matching the professional appearance of RViz.
"""

from __future__ import annotations

import math

try:
    import bpy
    import gpu
    from gpu_extras.batch import batch_for_shader
    from mathutils import Vector
except ImportError:
    # Allow importing without Blender
    bpy = None
    gpu = None
    batch_for_shader = None
    Vector = None


# Global drawing handle
_draw_handle = None


def generate_arrow_cone_vertices(
    origin: Vector, direction: Vector, length: float, cone_ratio: float = 0.2
) -> tuple[list, list]:
    """Generate vertices for an arrow cone at the tip of an axis.

    Args:
        origin: Start point of the arrow
        direction: Direction vector (normalized)
        length: Total length of axis
        cone_ratio: Ratio of cone length to total length (default 0.2 = 20%)

    Returns:
        Tuple of (positions, indices) for triangle drawing
    """
    cone_length = length * cone_ratio
    cone_radius = cone_length * 0.3  # Cone base radius
    shaft_end = length * (1.0 - cone_ratio)

    # Calculate cone tip and base center
    tip = origin + (direction * length)
    base_center = origin + (direction * shaft_end)

    # Create perpendicular vectors for cone base circle
    if abs(direction.x) < 0.9:
        perp1 = direction.cross(Vector((1, 0, 0))).normalized()
    else:
        perp1 = direction.cross(Vector((0, 1, 0))).normalized()
    perp2 = direction.cross(perp1).normalized()

    # Generate cone base circle vertices (8 segments for smooth appearance)
    num_segments = 8
    base_vertices = []
    for i in range(num_segments):
        angle = (2 * math.pi * i) / num_segments
        vertex = base_center + (perp1 * math.cos(angle) + perp2 * math.sin(angle)) * cone_radius
        base_vertices.append(vertex[:])

    positions = [tip[:]]  # Tip is vertex 0
    positions.extend(base_vertices)  # Base vertices are 1 to num_segments
    positions.append(base_center[:])  # Base center is last vertex

    # Generate triangle indices for cone
    indices = []
    tip_idx = 0
    center_idx = len(positions) - 1

    # Side triangles (from tip to base edge)
    for i in range(num_segments):
        next_i = (i + 1) % num_segments
        indices.extend([tip_idx, i + 1, next_i + 1])

    # Base triangles (filling the base)
    for i in range(num_segments):
        next_i = (i + 1) % num_segments
        indices.extend([center_idx, next_i + 1, i + 1])

    return positions, indices


def generate_axis_geometry(obj, axis_length: float = 0.2) -> dict:
    """Generate geometry data for RGB axes with arrow heads (RViz style).

    Args:
        obj: Blender Empty object with joint properties
        axis_length: Length of axis in Blender units

    Returns:
        Dictionary with line and triangle data for drawing
    """
    if not obj or obj.type != "EMPTY":
        return {"lines": [], "line_colors": [], "tris": [], "tri_colors": []}

    # Get joint origin in world space
    origin = obj.matrix_world.translation

    # Get joint rotation matrix
    rotation_matrix = obj.matrix_world.to_3x3()

    # Define axis directions in local space
    local_axes = {
        "x": Vector((1.0, 0.0, 0.0)),
        "y": Vector((0.0, 1.0, 0.0)),
        "z": Vector((0.0, 0.0, 1.0)),
    }

    # Define RGB colors for each axis (RViz convention)
    axis_colors = {
        "x": (1.0, 0.0, 0.0, 1.0),  # Red
        "y": (0.0, 1.0, 0.0, 1.0),  # Green
        "z": (0.0, 0.0, 1.0, 1.0),  # Blue
    }

    line_positions = []
    line_colors = []
    tri_positions = []
    tri_colors = []

    # Generate geometry for each axis
    for axis_name, local_dir in local_axes.items():
        # Transform axis direction to world space
        world_dir = rotation_matrix @ local_dir
        world_dir.normalize()

        color = axis_colors[axis_name]

        # Generate shaft line (from origin to 80% of length)
        shaft_length = axis_length * 0.8
        shaft_end = origin + (world_dir * shaft_length)
        line_positions.extend([origin[:], shaft_end[:]])
        line_colors.extend([color, color])

        # Generate arrow cone at tip
        cone_positions, cone_indices = generate_arrow_cone_vertices(
            origin, world_dir, axis_length, cone_ratio=0.2
        )

        # Add cone triangles
        for idx in cone_indices:
            tri_positions.append(cone_positions[idx])
            tri_colors.append(color)

    return {
        "lines": line_positions,
        "line_colors": line_colors,
        "tris": tri_positions,
        "tri_colors": tri_colors,
    }


def draw_joint_axes():
    """Draw RGB axes for all joint objects in the scene.

    This is called as a SpaceView3D draw handler.
    Draws RViz-style arrows with colored shafts and arrow heads.
    """
    if not bpy or not gpu:
        return

    context = bpy.context
    scene = context.scene

    # Get preferences
    show_axes = True
    axis_length = 0.2

    try:
        addon_prefs = context.preferences.addons.get("bl_ext.user_default.linkforge")
        if addon_prefs and hasattr(addon_prefs, "preferences"):
            prefs = addon_prefs.preferences
            if hasattr(prefs, "show_joint_axes"):
                show_axes = prefs.show_joint_axes
            if hasattr(prefs, "joint_axis_length"):
                axis_length = prefs.joint_axis_length
    except (AttributeError, KeyError):
        pass

    if not show_axes:
        return

    # Collect all joint geometry
    all_line_positions = []
    all_line_colors = []
    all_tri_positions = []
    all_tri_colors = []

    for obj in scene.objects:
        # Check if this is a joint Empty
        if (
            obj.type == "EMPTY"
            and hasattr(obj, "linkforge_joint")
            and obj.linkforge_joint.is_robot_joint
        ):
            # Generate axis geometry for this joint
            axis_data = generate_axis_geometry(obj, axis_length)
            all_line_positions.extend(axis_data["lines"])
            all_line_colors.extend(axis_data["line_colors"])
            all_tri_positions.extend(axis_data["tris"])
            all_tri_colors.extend(axis_data["tri_colors"])

    # Set up GPU state
    gpu.state.depth_test_set("LESS_EQUAL")
    gpu.state.blend_set("ALPHA")

    # Draw lines (shafts)
    if all_line_positions:
        shader = gpu.shader.from_builtin("FLAT_COLOR")
        batch = batch_for_shader(
            shader,
            "LINES",
            {"pos": all_line_positions, "color": all_line_colors},
        )
        gpu.state.line_width_set(4.0)
        shader.bind()
        batch.draw(shader)

    # Draw triangles (arrow cones)
    if all_tri_positions:
        shader = gpu.shader.from_builtin("FLAT_COLOR")
        batch = batch_for_shader(
            shader,
            "TRIS",
            {"pos": all_tri_positions, "color": all_tri_colors},
        )
        shader.bind()
        batch.draw(shader)

    # Reset GPU state
    gpu.state.line_width_set(1.0)
    gpu.state.blend_set("NONE")
    gpu.state.depth_test_set("NONE")


def fix_existing_joints(scene=None):
    """Fix display type for existing joints."""
    if bpy is None:
        return

    # Use current scene if available, otherwise skip (will be called on load)
    if scene is None:
        try:
            scene = bpy.context.scene
        except (AttributeError, RuntimeError):
            return

    for obj in scene.objects:
        if (
            obj.type == "EMPTY"
            and hasattr(obj, "linkforge_joint")
            and obj.linkforge_joint.is_robot_joint
        ):
            # Ensure PLAIN_AXES type
            if obj.empty_display_type != "PLAIN_AXES":
                obj.empty_display_type = "PLAIN_AXES"
            # Set very small display size for easier selection (but invisible axes)
            obj.empty_display_size = 0.01


def fix_current_scene():
    """Timer callback to fix joints in the current scene after registration.

    This runs once after the addon registers to fix any existing joints
    in the currently open scene. Returns None to prevent the timer from repeating.
    """
    fix_existing_joints()
    return None  # Don't repeat


def register():
    """Register the draw handler for joint axes visualization."""
    global _draw_handle

    if bpy is None:
        return

    # Add draw handler
    if _draw_handle is None:
        _draw_handle = bpy.types.SpaceView3D.draw_handler_add(
            draw_joint_axes, (), "WINDOW", "POST_VIEW"
        )

    # Fix joints when file is loaded
    if fix_existing_joints not in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.append(fix_existing_joints)

    # Fix joints in the current scene using a timer
    # This ensures Blender context is fully available after registration
    bpy.app.timers.register(fix_current_scene, first_interval=0.1)


def unregister():
    """Unregister the draw handler."""
    global _draw_handle

    if bpy is None:
        return

    # Remove timer (if still registered)
    if bpy.app.timers.is_registered(fix_current_scene):
        bpy.app.timers.unregister(fix_current_scene)

    # Remove load handler
    if fix_existing_joints in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.remove(fix_existing_joints)

    # Remove draw handler
    if _draw_handle is not None:
        bpy.types.SpaceView3D.draw_handler_remove(_draw_handle, "WINDOW")
        _draw_handle = None


if __name__ == "__main__":
    register()
