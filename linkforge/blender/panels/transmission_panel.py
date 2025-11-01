"""UI Panel for managing robot transmissions."""

from __future__ import annotations

try:
    import bpy
    from bpy.types import Panel
except ImportError:
    # Allow importing without Blender
    bpy = None  # type: ignore
    Panel = object


class LINKFORGE_PT_transmission_panel(Panel):
    """Panel for robot transmission properties in 3D Viewport sidebar."""

    bl_label = "Transmissions"
    bl_idname = "LINKFORGE_PT_transmission_panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "LinkForge"
    bl_order = 4

    def draw(self, context):
        """Draw the panel."""
        layout = self.layout
        obj = context.active_object

        # Create Transmission buttons (always visible)
        box = layout.box()
        box.operator("linkforge.create_transmission", icon="ADD")

        # Show "Create for Joint" button if a joint is selected
        if obj and obj.select_get() and obj.type == "EMPTY" and obj.linkforge_joint.is_robot_joint:
            box.operator("linkforge.create_simple_transmission", icon="CON_TRANSLIKE")

        if obj is None or not obj.select_get() or obj.type != "EMPTY":
            box.label(text="Select an Empty to edit transmission", icon="INFO")
            return

        props = obj.linkforge_transmission

        if not props.is_robot_transmission:
            box.label(text="This Empty is not a transmission", icon="INFO")
            return

        # Transmission properties
        box.separator()
        box.label(text=f"Transmission: {obj.name}", icon="CON_TRANSLIKE")
        box.prop(props, "transmission_name")

        # Transmission type
        box.separator()
        box.prop(props, "transmission_type")

        # Custom type field (if CUSTOM is selected)
        if props.transmission_type == "CUSTOM":
            box.prop(props, "custom_type")

        # Joint selection based on type
        box.separator()
        box.label(text="Joints:", icon="LINKED")

        if props.transmission_type == "SIMPLE":
            box.prop(props, "joint_name", text="Joint")

        elif props.transmission_type == "DIFFERENTIAL":
            box.prop(props, "joint1_name", text="Joint 1")
            box.prop(props, "joint2_name", text="Joint 2")

        # Hardware interface
        box.separator()
        box.label(text="Control:", icon="SETTINGS")
        box.prop(props, "hardware_interface")

        # Mechanical properties
        box.separator()
        box.label(text="Mechanical:", icon="TOOL_SETTINGS")
        box.prop(props, "mechanical_reduction")
        box.prop(props, "offset")

        # Actuator naming
        box.separator()
        box.label(text="Actuator:", icon="DRIVER")
        box.prop(props, "use_custom_actuator_name")

        if props.use_custom_actuator_name:
            if props.transmission_type == "SIMPLE":
                box.prop(props, "actuator_name", text="Name")
            elif props.transmission_type == "DIFFERENTIAL":
                box.prop(props, "actuator1_name", text="Actuator 1")
                box.prop(props, "actuator2_name", text="Actuator 2")

        # Delete button
        box.separator()
        box.operator("linkforge.delete_transmission", icon="TRASH", text="Delete Transmission")


# Registration
def register():
    """Register panel."""
    if bpy is not None:
        bpy.utils.register_class(LINKFORGE_PT_transmission_panel)


def unregister():
    """Unregister panel."""
    if bpy is not None:
        bpy.utils.unregister_class(LINKFORGE_PT_transmission_panel)


if __name__ == "__main__":
    register()
