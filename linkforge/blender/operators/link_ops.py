"""Operators for managing robot links."""

from __future__ import annotations

try:
    import bpy
    from bpy.types import Operator
except ImportError:
    # Allow importing without Blender
    bpy = None  # type: ignore
    Operator = object

from ..properties.link_props import sanitize_urdf_name


class LINKFORGE_OT_mark_as_link(Operator):
    """Mark selected object as a robot link"""

    bl_idname = "linkforge.mark_as_link"
    bl_label = "Mark as Link"
    bl_description = "Mark the selected object as a robot link"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        """Check if operator can run."""
        return context.active_object is not None

    def execute(self, context):
        """Execute the operator."""
        obj = context.active_object

        # Enable link properties
        obj.linkforge.is_robot_link = True

        # Set default link name if empty (sanitized version of object name)
        if not obj.linkforge.link_name:
            obj.linkforge.link_name = sanitize_urdf_name(obj.name)

        self.report({"INFO"}, f"Marked '{obj.name}' as robot link '{obj.linkforge.link_name}'")
        return {"FINISHED"}


class LINKFORGE_OT_remove_link(Operator):
    """Remove robot link marking from selected object (keeps the object)"""

    bl_idname = "linkforge.remove_link"
    bl_label = "Remove Link Marking"
    bl_description = "Remove robot link marking (object will not be deleted)"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        """Check if operator can run."""
        obj = context.active_object
        return obj is not None and obj.linkforge.is_robot_link

    def execute(self, context):
        """Execute the operator."""
        obj = context.active_object

        # Disable link properties (object is not deleted)
        obj.linkforge.is_robot_link = False

        self.report({"INFO"}, f"Removed link marking from '{obj.name}' (object kept)")
        return {"FINISHED"}


class LINKFORGE_OT_apply_material(Operator):
    """Apply material color to object in Blender viewport"""

    bl_idname = "linkforge.apply_material"
    bl_label = "Apply Material"
    bl_description = "Apply the selected material/color to the object in Blender viewport"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        """Check if operator can run."""
        obj = context.active_object
        return obj is not None and obj.linkforge.is_robot_link and obj.linkforge.use_material

    def execute(self, context):
        """Execute the operator."""
        obj = context.active_object
        props = obj.linkforge

        # Color mapping from presets
        preset_colors = {
            "RED": (1.0, 0.0, 0.0, 1.0),
            "GREEN": (0.0, 1.0, 0.0, 1.0),
            "BLUE": (0.0, 0.0, 1.0, 1.0),
            "YELLOW": (1.0, 1.0, 0.0, 1.0),
            "ORANGE": (1.0, 0.5, 0.0, 1.0),
            "CYAN": (0.0, 1.0, 1.0, 1.0),
            "MAGENTA": (1.0, 0.0, 1.0, 1.0),
            "WHITE": (1.0, 1.0, 1.0, 1.0),
            "BLACK": (0.0, 0.0, 0.0, 1.0),
            "GRAY": (0.5, 0.5, 0.5, 1.0),
            "ALUMINUM": (0.8, 0.8, 0.9, 1.0),
            "STEEL": (0.7, 0.7, 0.7, 1.0),
            "PLASTIC_BLACK": (0.1, 0.1, 0.1, 1.0),
            "RUBBER": (0.2, 0.2, 0.2, 1.0),
        }

        # Get color based on preset
        if props.material_preset == "CUSTOM":
            color = tuple(props.material_color)
        else:
            color = preset_colors.get(props.material_preset, (0.8, 0.8, 0.8, 1.0))

        # Create or get material
        mat_name = props.material_name if props.material_name else f"{obj.name}_material"
        mat = bpy.data.materials.get(mat_name)

        if mat is None:
            # Create new material
            mat = bpy.data.materials.new(name=mat_name)
            mat.use_nodes = True

        # Set material color
        if mat.use_nodes:
            # Use Principled BSDF
            bsdf = mat.node_tree.nodes.get("Principled BSDF")
            if bsdf:
                bsdf.inputs["Base Color"].default_value = color
                bsdf.inputs["Metallic"].default_value = 0.3 if "ALUMINUM" in props.material_preset or "STEEL" in props.material_preset else 0.0
        else:
            # Fallback for non-node materials
            mat.diffuse_color = color

        # Assign material to object
        if len(obj.data.materials) == 0:
            obj.data.materials.append(mat)
        else:
            obj.data.materials[0] = mat

        self.report({"INFO"}, f"Applied material to '{obj.name}'")
        return {"FINISHED"}


class LINKFORGE_OT_calculate_inertia(Operator):
    """Calculate inertia tensor from object geometry"""

    bl_idname = "linkforge.calculate_inertia"
    bl_label = "Calculate Inertia"
    bl_description = "Auto-calculate inertia tensor from object geometry and mass"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        """Check if operator can run."""
        obj = context.active_object
        return obj is not None and obj.linkforge.is_robot_link

    def execute(self, context):
        """Execute the operator."""
        obj = context.active_object
        props = obj.linkforge

        # Import here to avoid circular dependency
        from ...core.physics import calculate_inertia
        from ..utils.converters import get_object_geometry

        # Get geometry
        geometry = get_object_geometry(obj, props.collision_geometry_type)
        if geometry is None:
            self.report({"WARNING"}, "Could not extract geometry from object")
            return {"CANCELLED"}

        # Calculate inertia
        try:
            inertia = calculate_inertia(geometry, props.mass)

            # Update properties
            props.inertia_ixx = inertia.ixx
            props.inertia_ixy = inertia.ixy
            props.inertia_ixz = inertia.ixz
            props.inertia_iyy = inertia.iyy
            props.inertia_iyz = inertia.iyz
            props.inertia_izz = inertia.izz

            self.report({"INFO"}, f"Calculated inertia for '{obj.name}'")
            return {"FINISHED"}

        except Exception as e:
            self.report({"ERROR"}, f"Failed to calculate inertia: {e}")
            return {"CANCELLED"}


# Registration
classes = [
    LINKFORGE_OT_mark_as_link,
    LINKFORGE_OT_remove_link,
    LINKFORGE_OT_apply_material,
    LINKFORGE_OT_calculate_inertia,
]


def register():
    """Register operators."""
    if bpy is not None:
        for cls in classes:
            bpy.utils.register_class(cls)


def unregister():
    """Unregister operators."""
    if bpy is not None:
        for cls in reversed(classes):
            bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
