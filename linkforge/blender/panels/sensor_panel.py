"""UI Panel for managing robot sensors."""

from __future__ import annotations

try:
    import bpy
    from bpy.types import Panel
except ImportError:
    # Allow importing without Blender
    bpy = None  # type: ignore
    Panel = object


class LINKFORGE_PT_sensor_panel(Panel):
    """Panel for robot sensor properties in 3D Viewport sidebar."""

    bl_label = "Sensors"
    bl_idname = "LINKFORGE_PT_sensor_panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "LinkForge"
    bl_order = 4  # After Joints
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        """Draw the panel."""
        layout = self.layout
        obj = context.active_object

        # Create Sensor buttons (always visible)
        box = layout.box()
        box.operator("linkforge.create_sensor", icon="ADD")

        # Show "Create at Link" button if a link is selected
        if obj and obj.select_get() and obj.linkforge.is_robot_link:
            box.operator("linkforge.create_sensor_at_selection", icon="LIGHT_SUN")

        if obj is None or not obj.select_get() or obj.type != "EMPTY":
            box.label(text="Select an Empty to edit sensor", icon="INFO")
            return

        props = obj.linkforge_sensor

        if not props.is_robot_sensor:
            box.label(text="This Empty is not a sensor", icon="INFO")
            return

        # Sensor properties
        box.separator()
        box.label(text=f"Sensor: {obj.name}", icon="LIGHT_SUN")
        box.prop(props, "sensor_name")

        # Sensor type
        box.separator()
        box.prop(props, "sensor_type")

        # Presets section
        box.separator()
        preset_box = box.box()
        preset_box.label(text="Presets:", icon="PRESET")

        from ...core.presets import PresetManager

        manager = PresetManager()
        presets = manager.list_sensor_presets()

        if presets:
            row = preset_box.row()
            row.label(text="Apply:")
            for preset_name in presets[:4]:  # Show first 4
                op = row.operator(
                    "linkforge.apply_sensor_preset", text=preset_name[:12], icon="IMPORT"
                )
                op.preset_name = preset_name

        # Save current as preset
        preset_box.operator("linkforge.save_sensor_preset", icon="FILE_TICK")

        # Attached link
        box.separator()
        box.label(text="Attachment:", icon="LINKED")
        box.prop(props, "attached_link", icon="MESH_DATA")

        # Common properties
        box.separator()
        box.label(text="Settings:", icon="SETTINGS")
        box.prop(props, "update_rate")
        box.prop(props, "topic_name")

        # Type-specific properties
        sensor_type = props.sensor_type

        if sensor_type in {"CAMERA", "DEPTH_CAMERA"}:
            box.separator()
            box.label(text="Camera Settings:", icon="CAMERA_DATA")
            box.prop(props, "camera_horizontal_fov")
            box.prop(props, "camera_width")
            box.prop(props, "camera_height")
            box.prop(props, "camera_near_clip")
            box.prop(props, "camera_far_clip")

        elif sensor_type == "LIDAR":
            box.separator()
            box.label(text="LIDAR Settings:", icon="LIGHT_SPOT")
            box.prop(props, "lidar_horizontal_samples")
            box.prop(props, "lidar_horizontal_min_angle")
            box.prop(props, "lidar_horizontal_max_angle")
            box.prop(props, "lidar_vertical_samples")
            box.prop(props, "lidar_range_min")
            box.prop(props, "lidar_range_max")

        elif sensor_type == "IMU":
            box.separator()
            box.label(text="IMU Settings:", icon="ORIENTATION_GIMBAL")
            box.prop(props, "imu_gravity_magnitude")

        # Noise settings
        box.separator()
        box.label(text="Noise:", icon="RNDCURVE")
        box.prop(props, "use_noise")

        if props.use_noise:
            box.prop(props, "noise_type")
            box.prop(props, "noise_mean")
            box.prop(props, "noise_stddev")

        # Gazebo plugin
        box.separator()
        box.label(text="Gazebo Plugin:", icon="PLUGIN")
        box.prop(props, "use_gazebo_plugin")

        if props.use_gazebo_plugin:
            box.prop(props, "plugin_filename")

        # Delete button
        box.separator()
        box.operator("linkforge.delete_sensor", icon="TRASH", text="Delete Sensor")


# Registration
def register():
    """Register panel."""
    if bpy is not None:
        bpy.utils.register_class(LINKFORGE_PT_sensor_panel)


def unregister():
    """Unregister panel."""
    if bpy is not None:
        bpy.utils.unregister_class(LINKFORGE_PT_sensor_panel)


if __name__ == "__main__":
    register()
