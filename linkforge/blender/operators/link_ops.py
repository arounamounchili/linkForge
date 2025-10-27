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
