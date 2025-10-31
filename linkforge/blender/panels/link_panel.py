"""UI Panel for managing robot links."""

from __future__ import annotations

try:
    import bpy
    from bpy.types import Panel
except ImportError:
    # Allow importing without Blender
    bpy = None  # type: ignore
    Panel = object


class LINKFORGE_PT_link_panel(Panel):
    """Panel for robot link properties in 3D Viewport sidebar."""

    bl_label = "Links"
    bl_idname = "LINKFORGE_PT_link_panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "LinkForge"
    bl_order = 1

    def draw(self, context):
        """Draw the panel."""
        layout = self.layout
        obj = context.active_object

        if obj is None or not obj.select_get():
            layout.label(text="No object selected", icon="INFO")
            return

        props = obj.linkforge

        # Mark as Link section
        box = layout.box()
        row = box.row()
        row.prop(props, "is_robot_link", text="")
        row.label(text=f"Selected: {obj.name}", icon="OBJECT_DATA")

        if not props.is_robot_link:
            row = box.row()
            row.operator("linkforge.mark_as_link", icon="ADD")
            return

        # Link properties
        box.separator()
        box.prop(props, "link_name")

        # Inertial properties
        box.separator()
        box.label(text="Inertial Properties:", icon="PHYSICS")
        box.prop(props, "mass")

        # Visual indicator for inertia calculation status
        row = box.row()
        row.prop(props, "use_auto_inertia")

        # Show checkmark if auto-inertia is enabled and values seem calculated
        # (checking if diagonal values are non-zero)
        if props.use_auto_inertia:
            if props.inertia_ixx > 0 or props.inertia_iyy > 0 or props.inertia_izz > 0:
                row.label(text="", icon="CHECKMARK")
            else:
                row.label(text="", icon="ERROR")

        if not props.use_auto_inertia:
            # Manual inertia
            col = box.column(align=True)
            col.label(text="Inertia Tensor:")
            row = col.row(align=True)
            row.prop(props, "inertia_ixx", text="Ixx")
            row.prop(props, "inertia_iyy", text="Iyy")
            row.prop(props, "inertia_izz", text="Izz")
            row = col.row(align=True)
            row.prop(props, "inertia_ixy", text="Ixy")
            row.prop(props, "inertia_ixz", text="Ixz")
            row.prop(props, "inertia_iyz", text="Iyz")

        # Calculate button - show for auto or to recalculate
        box.operator("linkforge.calculate_inertia", icon="FILE_REFRESH")

        # Geometry section
        box.separator()
        box.label(text="Geometry:", icon="MESH_CUBE")

        # Visual
        box.prop(props, "use_visual_geometry")
        if props.use_visual_geometry:
            box.prop(props, "visual_geometry_type", text="Visual")
            # Note about mesh export limitation
            if props.visual_geometry_type == "MESH":
                info_box = box.box()
                info_box.label(text="Note: Mesh uses bounding", icon="INFO")
                info_box.label(text="box approximation for now")

        # Collision
        box.prop(props, "export_collision")
        if props.export_collision:
            box.prop(props, "collision_geometry_type", text="Collision")
            if props.collision_geometry_type == "MESH":
                box.prop(props, "simplify_collision")
                if props.simplify_collision:
                    box.prop(props, "collision_decimation_ratio", slider=True)

        # Material section
        box.separator()
        box.label(text="Material:", icon="MATERIAL")
        box.prop(props, "use_material")

        if props.use_material:
            box.prop(props, "material_source", text="Source")

            if props.material_source == "CUSTOM":
                # Custom color picker
                box.prop(props, "material_color", text="")
            else:
                # Show current Blender material color (read-only info)
                if obj.material_slots and obj.material_slots[0].material:
                    mat = obj.material_slots[0].material
                    info_row = box.row()
                    info_row.label(text=f"Using: {mat.name}", icon="CHECKMARK")
                else:
                    warn_row = box.row()
                    warn_row.label(text="No Blender material", icon="ERROR")

            box.prop(props, "material_name", text="Name")

        # Actions
        box.separator()
        box.operator("linkforge.remove_link", icon="X")


# Registration
def register():
    """Register panel."""
    if bpy is not None:
        bpy.utils.register_class(LINKFORGE_PT_link_panel)


def unregister():
    """Unregister panel."""
    if bpy is not None:
        bpy.utils.unregister_class(LINKFORGE_PT_link_panel)


if __name__ == "__main__":
    register()
