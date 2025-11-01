"""3D gizmos for visualizing joint axes in the viewport.

This module provides RViz-style RGB axis visualization for robot joints:
- Red = X axis
- Green = Y axis
- Blue = Z axis
"""

from __future__ import annotations

try:
    import bpy
    import gpu
    from gpu_extras.batch import batch_for_shader
    from mathutils import Matrix, Vector
except ImportError:
    # Allow importing without Blender
    bpy = None
    gpu = None
    batch_for_shader = None
    Matrix = None
    Vector = None


# Global drawing handle
_draw_handle = None


def get_joint_axis_vector(joint_props) -> tuple[float, float, float]:
    """Get the axis vector for a joint based on its properties.

    Args:
        joint_props: JointPropertyGroup instance

    Returns:
        Tuple of (x, y, z) representing the axis direction
    """
    if joint_props.axis == "X":
        return (1.0, 0.0, 0.0)
    elif joint_props.axis == "Y":
        return (0.0, 1.0, 0.0)
    elif joint_props.axis == "Z":
        return (0.0, 0.0, 1.0)
    elif joint_props.axis == "CUSTOM":
        return (
            joint_props.custom_axis_x,
            joint_props.custom_axis_y,
            joint_props.custom_axis_z,
        )
    else:
        return (0.0, 0.0, 1.0)  # Default to Z


def generate_axis_lines(obj, axis_length: float = 0.2) -> dict:
    """Generate line data for RGB axes visualization.

    Args:
        obj: Blender Empty object with joint properties
        axis_length: Length of axis lines in Blender units

    Returns:
        Dictionary with 'positions' and 'colors' lists for drawing
    """
    if not obj or obj.type != "EMPTY":
        return {"positions": [], "colors": []}

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

    positions = []
    colors = []

    # Generate lines for each axis
    for axis_name, local_dir in local_axes.items():
        # Transform axis direction to world space
        world_dir = rotation_matrix @ local_dir
        world_dir.normalize()

        # Calculate endpoint
        endpoint = origin + (world_dir * axis_length)

        # Add line (from origin to endpoint)
        positions.extend([origin[:], endpoint[:]])

        # Add color for both vertices (same color for the line)
        color = axis_colors[axis_name]
        colors.extend([color, color])

    return {"positions": positions, "colors": colors}


def draw_joint_axes():
    """Draw RGB axes for all joint objects in the scene.

    This is called as a SpaceView3D draw handler.
    """
    if not bpy or not gpu:
        return

    context = bpy.context
    scene = context.scene

    # Get preferences
    try:
        addon_prefs = context.preferences.addons.get("linkforge")
        if addon_prefs and hasattr(addon_prefs.preferences, "show_joint_axes"):
            if not addon_prefs.preferences.show_joint_axes:
                return
        # If preference doesn't exist yet, default to showing axes
    except (AttributeError, KeyError):
        pass

    # Check if axis length preference exists
    axis_length = 0.2  # Default
    try:
        addon_prefs = context.preferences.addons.get("linkforge")
        if addon_prefs and hasattr(addon_prefs.preferences, "joint_axis_length"):
            axis_length = addon_prefs.preferences.joint_axis_length
    except (AttributeError, KeyError):
        pass

    # Collect all joint objects
    all_positions = []
    all_colors = []

    for obj in scene.objects:
        # Check if this is a joint Empty
        if (
            obj.type == "EMPTY"
            and hasattr(obj, "linkforge_joint")
            and obj.linkforge_joint.is_robot_joint
        ):
            # Generate axis lines for this joint
            axis_data = generate_axis_lines(obj, axis_length)
            all_positions.extend(axis_data["positions"])
            all_colors.extend(axis_data["colors"])

    # Don't draw if no joints
    if not all_positions:
        return

    # Create shader
    shader = gpu.shader.from_builtin("SMOOTH_COLOR")

    # Create batch
    batch = batch_for_shader(
        shader,
        "LINES",
        {"pos": all_positions, "color": all_colors},
    )

    # Enable line smoothing and set line width
    gpu.state.blend_set("ALPHA")
    gpu.state.line_width_set(3.0)

    # Draw
    shader.bind()
    batch.draw(shader)

    # Reset state
    gpu.state.line_width_set(1.0)
    gpu.state.blend_set("NONE")


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


def unregister():
    """Unregister the draw handler."""
    global _draw_handle

    if bpy is None:
        return

    # Remove draw handler
    if _draw_handle is not None:
        bpy.types.SpaceView3D.draw_handler_remove(_draw_handle, "WINDOW")
        _draw_handle = None


if __name__ == "__main__":
    register()
