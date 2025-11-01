"""UI Panel for managing robot joints."""

from __future__ import annotations

try:
    import bpy
    from bpy.types import Panel
except ImportError:
    # Allow importing without Blender
    bpy = None  # type: ignore
    Panel = object


class LINKFORGE_PT_joint_panel(Panel):
    """Panel for robot joint properties in 3D Viewport sidebar."""

    bl_label = "Joints"
    bl_idname = "LINKFORGE_PT_joint_panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "LinkForge"
    bl_order = 2

    def draw(self, context):
        """Draw the panel."""
        layout = self.layout
        obj = context.active_object

        # Visualization settings
        vis_box = layout.box()
        vis_box.label(text="Visualization", icon="HIDE_OFF")

        addon_prefs = context.preferences.addons.get("bl_ext.user_default.linkforge")
        if addon_prefs and hasattr(addon_prefs, "preferences"):
            prefs = addon_prefs.preferences
            row = vis_box.row()
            row.prop(prefs, "show_joint_axes", text="Show Axes (RGB)")
            row = vis_box.row()
            row.prop(prefs, "joint_axis_length", text="Length", slider=True)
            row.enabled = prefs.show_joint_axes

        # Create Joint buttons (always visible)
        box = layout.box()
        box.operator("linkforge.create_joint", icon="ADD")

        # Show "Create at Link" button if a link is selected
        if obj and obj.select_get() and obj.linkforge.is_robot_link:
            box.operator("linkforge.create_joint_at_selection", icon="PIVOT_BOUNDBOX")

        if obj is None or not obj.select_get() or obj.type != "EMPTY":
            box.label(text="Select an Empty to edit joint", icon="INFO")
            return

        props = obj.linkforge_joint

        if not props.is_robot_joint:
            box.label(text="This Empty is not a joint", icon="INFO")
            return

        # Joint properties
        box.separator()
        box.label(text=f"Joint: {obj.name}", icon="EMPTY_ARROWS")
        box.prop(props, "joint_name")

        # Joint type
        box.separator()
        box.prop(props, "joint_type")

        # Parent and child
        box.separator()
        box.label(text="Connection:", icon="LINKED")
        box.prop(props, "parent_link", icon="TRIA_UP")
        box.prop(props, "child_link", icon="TRIA_DOWN")

        # Joint axis
        if props.joint_type in {"REVOLUTE", "CONTINUOUS", "PRISMATIC"}:
            box.separator()
            box.label(text="Axis:", icon="ORIENTATION_GIMBAL")
            box.prop(props, "axis", expand=True)

            if props.axis == "CUSTOM":
                col = box.column(align=True)
                col.prop(props, "custom_axis_x", text="X")
                col.prop(props, "custom_axis_y", text="Y")
                col.prop(props, "custom_axis_z", text="Z")

        # Joint limits
        if props.joint_type in {"REVOLUTE", "PRISMATIC"}:
            box.separator()
            box.prop(props, "use_limits")

            if props.use_limits:
                col = box.column(align=True)
                col.label(text="Limits:", icon="DRIVER_DISTANCE")
                col.prop(props, "limit_lower")
                col.prop(props, "limit_upper")
                col.prop(props, "limit_effort")
                col.prop(props, "limit_velocity")

        # Dynamics (optional)
        box.separator()
        box.prop(props, "use_dynamics")
        if props.use_dynamics:
            col = box.column(align=True)
            col.prop(props, "dynamics_damping")
            col.prop(props, "dynamics_friction")

        # Mimic (optional)
        box.separator()
        box.prop(props, "use_mimic")
        if props.use_mimic:
            col = box.column(align=True)
            col.prop(props, "mimic_joint")
            col.prop(props, "mimic_multiplier")
            col.prop(props, "mimic_offset")

        # Actions
        box.separator()
        box.operator("linkforge.remove_joint", icon="X")


# Registration
def register():
    """Register panel."""
    if bpy is not None:
        bpy.utils.register_class(LINKFORGE_PT_joint_panel)


def unregister():
    """Unregister panel."""
    if bpy is not None:
        bpy.utils.unregister_class(LINKFORGE_PT_joint_panel)


if __name__ == "__main__":
    register()
