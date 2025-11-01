"""Operators for managing robot sensors."""

from __future__ import annotations

try:
    import bpy
    from bpy.types import Operator
except ImportError:
    # Allow importing without Blender
    bpy = None  # type: ignore
    Operator = object

from ..properties.link_props import sanitize_urdf_name


class LINKFORGE_OT_create_sensor(Operator):
    """Create a new robot sensor at 3D cursor"""

    bl_idname = "linkforge.create_sensor"
    bl_label = "Create Sensor at Cursor"
    bl_description = "Create a new robot sensor (Empty) at the 3D cursor location"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        """Execute the operator."""
        # Create Empty at cursor location
        bpy.ops.object.empty_add(type="PLAIN_AXES", location=context.scene.cursor.location)
        sensor_empty = context.active_object
        sensor_empty.name = "Sensor"

        # Set display size for easier selection
        sensor_empty.empty_display_size = 0.1

        # Enable sensor properties
        sensor_empty.linkforge_sensor.is_robot_sensor = True
        sensor_empty.linkforge_sensor.sensor_name = sanitize_urdf_name(sensor_empty.name)

        # Set default sensor type
        sensor_empty.linkforge_sensor.sensor_type = "CAMERA"

        # Try to auto-detect attached link from selection
        selected_links = [
            obj
            for obj in context.selected_objects
            if obj != sensor_empty and obj.linkforge.is_robot_link
        ]

        if len(selected_links) >= 1:
            sensor_empty.linkforge_sensor.attached_link = selected_links[0].linkforge.link_name

        self.report({"INFO"}, f"Created sensor '{sensor_empty.name}'")
        return {"FINISHED"}


class LINKFORGE_OT_create_sensor_at_selection(Operator):
    """Create a new robot sensor at selected link's location"""

    bl_idname = "linkforge.create_sensor_at_selection"
    bl_label = "Create Sensor at Link"
    bl_description = "Create a new robot sensor at the selected link's location and orientation"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        """Check if operator can run."""
        obj = context.active_object
        return obj is not None and obj.linkforge.is_robot_link

    def execute(self, context):
        """Execute the operator."""
        obj = context.active_object

        # Get object's world location and rotation
        location = obj.matrix_world.translation.copy()
        rotation = obj.matrix_world.to_euler()

        # Create Empty at object's location
        bpy.ops.object.empty_add(type="PLAIN_AXES", location=location)
        sensor_empty = context.active_object
        sensor_empty.name = f"{obj.linkforge.link_name}_sensor"
        sensor_empty.rotation_euler = rotation

        # Set display size for easier selection
        sensor_empty.empty_display_size = 0.1

        # Enable sensor properties
        sensor_empty.linkforge_sensor.is_robot_sensor = True
        sensor_empty.linkforge_sensor.sensor_name = sanitize_urdf_name(sensor_empty.name)

        # Set default sensor type
        sensor_empty.linkforge_sensor.sensor_type = "CAMERA"

        # Auto-set attached link to selected object
        sensor_empty.linkforge_sensor.attached_link = obj.linkforge.link_name

        self.report({"INFO"}, f"Created sensor '{sensor_empty.name}' attached to '{obj.name}'")
        return {"FINISHED"}


class LINKFORGE_OT_delete_sensor(Operator):
    """Delete the selected sensor"""

    bl_idname = "linkforge.delete_sensor"
    bl_label = "Delete Sensor"
    bl_description = "Delete the selected robot sensor"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        """Check if operator can run."""
        obj = context.active_object
        return obj is not None and obj.type == "EMPTY" and obj.linkforge_sensor.is_robot_sensor

    def execute(self, context):
        """Execute the operator."""
        obj = context.active_object
        sensor_name = obj.linkforge_sensor.sensor_name or obj.name

        # Delete the object
        bpy.data.objects.remove(obj, do_unlink=True)

        self.report({"INFO"}, f"Deleted sensor '{sensor_name}'")
        return {"FINISHED"}


# Registration
def register():
    """Register operators."""
    if bpy is not None:
        bpy.utils.register_class(LINKFORGE_OT_create_sensor)
        bpy.utils.register_class(LINKFORGE_OT_create_sensor_at_selection)
        bpy.utils.register_class(LINKFORGE_OT_delete_sensor)


def unregister():
    """Unregister operators."""
    if bpy is not None:
        bpy.utils.unregister_class(LINKFORGE_OT_delete_sensor)
        bpy.utils.unregister_class(LINKFORGE_OT_create_sensor_at_selection)
        bpy.utils.unregister_class(LINKFORGE_OT_create_sensor)


if __name__ == "__main__":
    register()
