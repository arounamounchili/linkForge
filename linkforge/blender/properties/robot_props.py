"""Blender Property Groups for robot-level properties.

These properties are stored on the Scene and define global robot settings.
"""

from __future__ import annotations

try:
    import bpy
    from bpy.props import BoolProperty, EnumProperty, StringProperty
    from bpy.types import PropertyGroup
except ImportError:
    # Allow importing without Blender for testing
    PropertyGroup = object
    BoolProperty = EnumProperty = StringProperty = None


def update_joint_axes_visibility(self, context):
    """Update visibility of all joint empties in the scene."""
    if bpy is None:
        return

    show = self.show_joint_axes

    # Find all joint empties and update their visibility
    for obj in context.scene.objects:
        if obj.type == 'EMPTY' and hasattr(obj, 'linkforge_joint'):
            if obj.linkforge_joint.is_robot_joint:
                obj.hide_viewport = not show


class RobotPropertyGroup(PropertyGroup):
    """Global robot properties stored on the Scene."""

    # Robot identification
    robot_name: StringProperty(  # type: ignore
        name="Robot Name",
        description="Name of the robot in URDF",
        default="my_robot",
        maxlen=64,
    )

    # Export settings
    export_format: EnumProperty(  # type: ignore
        name="Format",
        description="Export format",
        items=[
            ("URDF", "URDF", "Unified Robot Description Format (XML)"),
            ("XACRO", "XACRO", "XACRO (XML Macros for URDF)"),
        ],
        default="URDF",
    )

    export_meshes: BoolProperty(  # type: ignore
        name="Export Meshes",
        description="Export mesh files (STL/OBJ) alongside URDF",
        default=True,
    )

    mesh_format: EnumProperty(  # type: ignore
        name="Mesh Format",
        description="Format for exported visual mesh files (collision always uses STL)",
        items=[
            ("OBJ", "OBJ", "Wavefront OBJ with materials (recommended)"),
            ("STL", "STL", "STereoLithography without materials"),
        ],
        default="OBJ",
    )

    mesh_directory_name: StringProperty(  # type: ignore
        name="Mesh Directory",
        description="Subdirectory name for mesh files",
        default="meshes",
    )

    # Validation
    validate_before_export: BoolProperty(  # type: ignore
        name="Validate Before Export",
        description="Validate robot structure before exporting",
        default=True,
    )

    # Visual helpers
    show_joint_axes: BoolProperty(  # type: ignore
        name="Show Joint Axes",
        description="Display all joint axes in viewport",
        default=True,
        update=update_joint_axes_visibility,
    )

    show_kinematic_tree: BoolProperty(  # type: ignore
        name="Show Structure",
        description="Display robot tree structure in panel",
        default=False,
    )


# Registration
def register():
    """Register property group."""
    if bpy is not None:
        bpy.utils.register_class(RobotPropertyGroup)
        bpy.types.Scene.linkforge = bpy.props.PointerProperty(type=RobotPropertyGroup)


def unregister():
    """Unregister property group."""
    if bpy is not None:
        del bpy.types.Scene.linkforge
        bpy.utils.unregister_class(RobotPropertyGroup)


if __name__ == "__main__":
    register()
