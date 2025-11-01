"""Blender addon preferences for LinkForge.

User preferences for controlling visualization and behavior.
"""

from __future__ import annotations

try:
    import bpy
    from bpy.props import BoolProperty, FloatProperty
    from bpy.types import AddonPreferences
except ImportError:
    # Allow importing without Blender
    bpy = None
    BoolProperty = FloatProperty = None
    AddonPreferences = object


class LinkForgePreferences(AddonPreferences):
    """User preferences for LinkForge extension."""

    bl_idname = "linkforge"

    # Joint axis visualization
    show_joint_axes: BoolProperty(  # type: ignore
        name="Show Joint Axes",
        description="Display RGB colored axes for all joints (Red=X, Green=Y, Blue=Z)",
        default=True,
    )

    joint_axis_length: FloatProperty(  # type: ignore
        name="Axis Length",
        description="Length of joint axis lines in Blender units",
        default=0.2,
        min=0.01,
        max=2.0,
        soft_min=0.05,
        soft_max=1.0,
        step=5,
        precision=2,
        unit="LENGTH",
    )

    def draw(self, context):
        """Draw the preferences UI."""
        layout = self.layout

        # Joint Visualization Section
        box = layout.box()
        box.label(text="Joint Visualization", icon="EMPTY_ARROWS")

        row = box.row()
        row.prop(self, "show_joint_axes")

        row = box.row()
        row.prop(self, "joint_axis_length")
        row.enabled = self.show_joint_axes

        # Help text
        box.separator()
        help_box = box.box()
        help_box.label(text="RGB Axis Colors (RViz Convention):", icon="INFO")
        row = help_box.row()
        row.label(text="Red = X axis", icon="DECORATE")
        row = help_box.row()
        row.label(text="Green = Y axis", icon="DECORATE")
        row = help_box.row()
        row.label(text="Blue = Z axis", icon="DECORATE")


def register():
    """Register preferences."""
    if bpy is not None:
        bpy.utils.register_class(LinkForgePreferences)


def unregister():
    """Unregister preferences."""
    if bpy is not None:
        bpy.utils.unregister_class(LinkForgePreferences)


if __name__ == "__main__":
    register()
