"""Operators for managing robot transmissions."""

from __future__ import annotations

try:
    import bpy
    from bpy.types import Operator
except ImportError:
    # Allow importing without Blender
    bpy = None  # type: ignore
    Operator = object

from ..properties.link_props import sanitize_urdf_name


class LINKFORGE_OT_create_transmission(Operator):
    """Create a new robot transmission"""

    bl_idname = "linkforge.create_transmission"
    bl_label = "Create Transmission"
    bl_description = "Create a new robot transmission (Empty) at the 3D cursor location"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        """Execute the operator."""
        # Create Empty at cursor location
        bpy.ops.object.empty_add(type="SPHERE", location=context.scene.cursor.location)
        transmission_empty = context.active_object
        transmission_empty.name = "Transmission"

        # Set display size for easier selection
        transmission_empty.empty_display_size = 0.05

        # Enable transmission properties
        transmission_empty.linkforge_transmission.is_robot_transmission = True
        transmission_empty.linkforge_transmission.transmission_name = sanitize_urdf_name(
            transmission_empty.name
        )

        # Set default transmission type
        transmission_empty.linkforge_transmission.transmission_type = "SIMPLE"

        self.report({"INFO"}, f"Created transmission '{transmission_empty.name}'")
        return {"FINISHED"}


class LINKFORGE_OT_create_simple_transmission(Operator):
    """Create a simple transmission for selected joint"""

    bl_idname = "linkforge.create_simple_transmission"
    bl_label = "Create Simple Transmission for Joint"
    bl_description = "Create a simple transmission for the selected joint"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        """Check if operator can run."""
        obj = context.active_object
        return obj is not None and obj.type == "EMPTY" and obj.linkforge_joint.is_robot_joint

    def execute(self, context):
        """Execute the operator."""
        joint_obj = context.active_object
        joint_name = joint_obj.linkforge_joint.joint_name

        # Create Empty at joint location
        location = joint_obj.matrix_world.translation.copy()
        bpy.ops.object.empty_add(type="SPHERE", location=location)
        transmission_empty = context.active_object
        transmission_empty.name = f"{joint_name}_trans"

        # Set display size for easier selection
        transmission_empty.empty_display_size = 0.05

        # Enable transmission properties
        transmission_empty.linkforge_transmission.is_robot_transmission = True
        transmission_empty.linkforge_transmission.transmission_name = sanitize_urdf_name(
            transmission_empty.name
        )

        # Configure as simple transmission
        transmission_empty.linkforge_transmission.transmission_type = "SIMPLE"
        transmission_empty.linkforge_transmission.joint_name = joint_name

        self.report(
            {"INFO"},
            f"Created simple transmission '{transmission_empty.name}' for joint '{joint_name}'",
        )
        return {"FINISHED"}


class LINKFORGE_OT_delete_transmission(Operator):
    """Delete the selected transmission"""

    bl_idname = "linkforge.delete_transmission"
    bl_label = "Delete Transmission"
    bl_description = "Delete the selected robot transmission"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        """Check if operator can run."""
        obj = context.active_object
        return (
            obj is not None
            and obj.type == "EMPTY"
            and obj.linkforge_transmission.is_robot_transmission
        )

    def execute(self, context):
        """Execute the operator."""
        obj = context.active_object
        transmission_name = obj.linkforge_transmission.transmission_name or obj.name

        # Delete the object
        bpy.data.objects.remove(obj, do_unlink=True)

        self.report({"INFO"}, f"Deleted transmission '{transmission_name}'")
        return {"FINISHED"}


# Registration
def register():
    """Register operators."""
    if bpy is not None:
        bpy.utils.register_class(LINKFORGE_OT_create_transmission)
        bpy.utils.register_class(LINKFORGE_OT_create_simple_transmission)
        bpy.utils.register_class(LINKFORGE_OT_delete_transmission)


def unregister():
    """Unregister operators."""
    if bpy is not None:
        bpy.utils.unregister_class(LINKFORGE_OT_delete_transmission)
        bpy.utils.unregister_class(LINKFORGE_OT_create_simple_transmission)
        bpy.utils.unregister_class(LINKFORGE_OT_create_transmission)


if __name__ == "__main__":
    register()
