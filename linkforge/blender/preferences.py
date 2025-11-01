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


def update_joint_axes_visibility(self, context):
    """Callback when show_joint_axes changes - force viewport redraw."""
    if bpy:
        # Force all 3D viewports to redraw
        for window in context.window_manager.windows:
            for area in window.screen.areas:
                if area.type == "VIEW_3D":
                    area.tag_redraw()


def update_joint_axis_length(self, context):
    """Callback when joint_axis_length changes - force viewport redraw."""
    if bpy:
        # Force all 3D viewports to redraw
        for window in context.window_manager.windows:
            for area in window.screen.areas:
                if area.type == "VIEW_3D":
                    area.tag_redraw()


class LinkForgePreferences(AddonPreferences):
    """User preferences for LinkForge extension."""

    bl_idname = "bl_ext.user_default.linkforge"

    # Joint axis visualization
    show_joint_axes: BoolProperty(  # type: ignore
        name="Show Joint Axes",
        description="Display RGB colored axes for all joints (Red=X, Green=Y, Blue=Z)",
        default=True,
        update=update_joint_axes_visibility,
    )

    joint_axis_length: FloatProperty(  # type: ignore
        name="Axis Length",
        description="Length of joint axis arrows in Blender units (adjust to fit your robot scale)",
        default=0.2,
        min=0.01,
        max=100.0,
        soft_min=0.05,
        soft_max=5.0,
        step=5,
        precision=2,
        unit="LENGTH",
        update=update_joint_axis_length,
    )

    def draw(self, context):
        """Draw the preferences UI."""
        layout = self.layout

        # Joint Visualization Section
        box = layout.box()
        box.label(text="Joint Visualization", icon="EMPTY_ARROWS")

        # Toggle checkbox
        row = box.row()
        row.prop(self, "show_joint_axes", text="Show Joint Axes (RGB Arrows)")

        # Length slider
        row = box.row()
        row.prop(self, "joint_axis_length", slider=True)
        row.enabled = self.show_joint_axes

        # Quick access note
        box.separator()
        info_box = box.box()
        info_box.label(text="Quick Access:", icon="INFO")
        info_box.label(text="Toggle in 3D View → N Panel → LinkForge → Joints")

        # Help text
        box.separator()
        help_box = box.box()
        help_box.label(text="RGB Axis Colors (RViz Convention):", icon="COLOR")
        col = help_box.column(align=True)
        col.label(text="  Red Arrow = X axis", icon="EVENT_X")
        col.label(text="  Green Arrow = Y axis", icon="EVENT_Y")
        col.label(text="  Blue Arrow = Z axis", icon="EVENT_Z")

        # Usage tip
        box.separator()
        tip_box = box.box()
        tip_box.label(text="Tip: Adjust 'Axis Length' to match your robot's scale", icon="LIGHT")


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
