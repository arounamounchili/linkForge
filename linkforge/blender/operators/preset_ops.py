"""Operators for managing presets."""

from __future__ import annotations

try:
    import bpy
    from bpy.types import Operator
except ImportError:
    # Allow importing without Blender
    bpy = None  # type: ignore
    Operator = object

from ...core.presets import JointPreset, MaterialPreset, PresetManager, SensorPreset

# Global preset manager instance (cached for performance)
_PRESET_MANAGER = None


def get_preset_manager():
    """Get the global preset manager instance."""
    global _PRESET_MANAGER
    if _PRESET_MANAGER is None:
        _PRESET_MANAGER = PresetManager()
    return _PRESET_MANAGER


class LINKFORGE_OT_apply_joint_preset(Operator):
    """Apply a joint preset to the selected joint"""

    bl_idname = "linkforge.apply_joint_preset"
    bl_label = "Apply Joint Preset"
    bl_description = "Apply a saved joint preset to the selected joint"
    bl_options = {"REGISTER", "UNDO"}

    preset_name: bpy.props.StringProperty(name="Preset Name")  # type: ignore

    @classmethod
    def poll(cls, context):
        """Check if operator can run."""
        obj = context.active_object
        return obj is not None and obj.type == "EMPTY" and obj.linkforge_joint.is_robot_joint

    def execute(self, context):
        """Execute the operator."""
        obj = context.active_object
        props = obj.linkforge_joint

        # Load preset
        manager = get_preset_manager()
        preset = manager.load_joint_preset(self.preset_name)

        if preset is None:
            self.report({"ERROR"}, f"Preset '{self.preset_name}' not found")
            return {"CANCELLED"}

        # Apply preset to joint
        props.joint_type = preset.joint_type
        props.axis = preset.axis
        props.use_limits = preset.use_limits
        props.limit_lower = preset.limit_lower
        props.limit_upper = preset.limit_upper
        props.limit_effort = preset.limit_effort
        props.limit_velocity = preset.limit_velocity
        props.use_dynamics = preset.use_dynamics
        props.dynamics_damping = preset.dynamics_damping
        props.dynamics_friction = preset.dynamics_friction

        self.report({"INFO"}, f"Applied preset '{self.preset_name}' to joint '{obj.name}'")
        return {"FINISHED"}


class LINKFORGE_OT_save_joint_preset(Operator):
    """Save current joint configuration as a preset"""

    bl_idname = "linkforge.save_joint_preset"
    bl_label = "Save Joint Preset"
    bl_description = "Save the current joint configuration as a reusable preset"
    bl_options = {"REGISTER", "UNDO"}

    preset_name: bpy.props.StringProperty(  # type: ignore
        name="Preset Name", description="Name for this preset"
    )
    preset_description: bpy.props.StringProperty(  # type: ignore
        name="Description", description="Optional description", default=""
    )

    @classmethod
    def poll(cls, context):
        """Check if operator can run."""
        obj = context.active_object
        return obj is not None and obj.type == "EMPTY" and obj.linkforge_joint.is_robot_joint

    def invoke(self, context, event):
        """Show dialog to enter preset name."""
        return context.window_manager.invoke_props_dialog(self)

    def execute(self, context):
        """Execute the operator."""
        if not self.preset_name:
            self.report({"ERROR"}, "Preset name cannot be empty")
            return {"CANCELLED"}

        obj = context.active_object
        props = obj.linkforge_joint

        # Create preset from current settings
        preset = JointPreset(
            name=self.preset_name,
            description=self.preset_description,
            joint_type=props.joint_type,
            axis=props.axis,
            use_limits=props.use_limits,
            limit_lower=props.limit_lower,
            limit_upper=props.limit_upper,
            limit_effort=props.limit_effort,
            limit_velocity=props.limit_velocity,
            use_dynamics=props.use_dynamics,
            dynamics_damping=props.dynamics_damping,
            dynamics_friction=props.dynamics_friction,
        )

        # Save preset
        manager = get_preset_manager()
        manager.save_joint_preset(preset)

        self.report({"INFO"}, f"Saved joint preset '{self.preset_name}'")
        return {"FINISHED"}


class LINKFORGE_OT_apply_material_preset(Operator):
    """Apply a material preset to the selected link"""

    bl_idname = "linkforge.apply_material_preset"
    bl_label = "Apply Material Preset"
    bl_description = "Apply a saved material preset to the selected link"
    bl_options = {"REGISTER", "UNDO"}

    preset_name: bpy.props.StringProperty(name="Preset Name")  # type: ignore

    @classmethod
    def poll(cls, context):
        """Check if operator can run."""
        obj = context.active_object
        return obj is not None and obj.linkforge.is_robot_link

    def execute(self, context):
        """Execute the operator."""
        obj = context.active_object
        props = obj.linkforge

        # Load preset
        manager = get_preset_manager()
        preset = manager.load_material_preset(self.preset_name)

        if preset is None:
            self.report({"ERROR"}, f"Preset '{self.preset_name}' not found")
            return {"CANCELLED"}

        # Apply preset to link
        props.use_material = True
        props.material_source = "CUSTOM"
        props.material_color = preset.color
        props.material_name = preset.material_name

        self.report({"INFO"}, f"Applied material preset '{self.preset_name}' to link '{obj.name}'")
        return {"FINISHED"}


class LINKFORGE_OT_save_material_preset(Operator):
    """Save current material configuration as a preset"""

    bl_idname = "linkforge.save_material_preset"
    bl_label = "Save Material Preset"
    bl_description = "Save the current material configuration as a reusable preset"
    bl_options = {"REGISTER", "UNDO"}

    preset_name: bpy.props.StringProperty(  # type: ignore
        name="Preset Name", description="Name for this preset"
    )
    preset_description: bpy.props.StringProperty(  # type: ignore
        name="Description", description="Optional description", default=""
    )

    @classmethod
    def poll(cls, context):
        """Check if operator can run."""
        obj = context.active_object
        return obj is not None and obj.linkforge.is_robot_link

    def invoke(self, context, event):
        """Show dialog to enter preset name."""
        return context.window_manager.invoke_props_dialog(self)

    def execute(self, context):
        """Execute the operator."""
        if not self.preset_name:
            self.report({"ERROR"}, "Preset name cannot be empty")
            return {"CANCELLED"}

        obj = context.active_object
        props = obj.linkforge

        # Create preset from current settings
        preset = MaterialPreset(
            name=self.preset_name,
            description=self.preset_description,
            color=list(props.material_color),
            material_name=props.material_name,
        )

        # Save preset
        manager = get_preset_manager()
        manager.save_material_preset(preset)

        self.report({"INFO"}, f"Saved material preset '{self.preset_name}'")
        return {"FINISHED"}


class LINKFORGE_OT_apply_sensor_preset(Operator):
    """Apply a sensor preset to the selected sensor"""

    bl_idname = "linkforge.apply_sensor_preset"
    bl_label = "Apply Sensor Preset"
    bl_description = "Apply a saved sensor preset to the selected sensor"
    bl_options = {"REGISTER", "UNDO"}

    preset_name: bpy.props.StringProperty(name="Preset Name")  # type: ignore

    @classmethod
    def poll(cls, context):
        """Check if operator can run."""
        obj = context.active_object
        return obj is not None and obj.type == "EMPTY" and obj.linkforge_sensor.is_robot_sensor

    def execute(self, context):
        """Execute the operator."""
        obj = context.active_object
        props = obj.linkforge_sensor

        # Load preset
        manager = get_preset_manager()
        preset = manager.load_sensor_preset(self.preset_name)

        if preset is None:
            self.report({"ERROR"}, f"Preset '{self.preset_name}' not found")
            return {"CANCELLED"}

        # Apply preset to sensor
        props.sensor_type = preset.sensor_type
        props.update_rate = preset.update_rate

        # Camera settings
        props.camera_horizontal_fov = preset.camera_horizontal_fov
        props.camera_width = preset.camera_width
        props.camera_height = preset.camera_height
        props.camera_near_clip = preset.camera_near_clip
        props.camera_far_clip = preset.camera_far_clip

        # LIDAR settings
        props.lidar_horizontal_samples = preset.lidar_horizontal_samples
        props.lidar_horizontal_min_angle = preset.lidar_horizontal_min_angle
        props.lidar_horizontal_max_angle = preset.lidar_horizontal_max_angle
        props.lidar_vertical_samples = preset.lidar_vertical_samples
        props.lidar_range_min = preset.lidar_range_min
        props.lidar_range_max = preset.lidar_range_max

        # IMU settings
        props.imu_gravity_magnitude = preset.imu_gravity_magnitude

        # Noise settings
        props.use_noise = preset.use_noise
        props.noise_type = preset.noise_type
        props.noise_mean = preset.noise_mean
        props.noise_stddev = preset.noise_stddev

        self.report({"INFO"}, f"Applied sensor preset '{self.preset_name}' to sensor '{obj.name}'")
        return {"FINISHED"}


class LINKFORGE_OT_save_sensor_preset(Operator):
    """Save current sensor configuration as a preset"""

    bl_idname = "linkforge.save_sensor_preset"
    bl_label = "Save Sensor Preset"
    bl_description = "Save the current sensor configuration as a reusable preset"
    bl_options = {"REGISTER", "UNDO"}

    preset_name: bpy.props.StringProperty(  # type: ignore
        name="Preset Name", description="Name for this preset"
    )
    preset_description: bpy.props.StringProperty(  # type: ignore
        name="Description", description="Optional description", default=""
    )

    @classmethod
    def poll(cls, context):
        """Check if operator can run."""
        obj = context.active_object
        return obj is not None and obj.type == "EMPTY" and obj.linkforge_sensor.is_robot_sensor

    def invoke(self, context, event):
        """Show dialog to enter preset name."""
        return context.window_manager.invoke_props_dialog(self)

    def execute(self, context):
        """Execute the operator."""
        if not self.preset_name:
            self.report({"ERROR"}, "Preset name cannot be empty")
            return {"CANCELLED"}

        obj = context.active_object
        props = obj.linkforge_sensor

        # Create preset from current settings
        preset = SensorPreset(
            name=self.preset_name,
            description=self.preset_description,
            sensor_type=props.sensor_type,
            update_rate=props.update_rate,
            camera_horizontal_fov=props.camera_horizontal_fov,
            camera_width=props.camera_width,
            camera_height=props.camera_height,
            camera_near_clip=props.camera_near_clip,
            camera_far_clip=props.camera_far_clip,
            lidar_horizontal_samples=props.lidar_horizontal_samples,
            lidar_horizontal_min_angle=props.lidar_horizontal_min_angle,
            lidar_horizontal_max_angle=props.lidar_horizontal_max_angle,
            lidar_vertical_samples=props.lidar_vertical_samples,
            lidar_range_min=props.lidar_range_min,
            lidar_range_max=props.lidar_range_max,
            imu_gravity_magnitude=props.imu_gravity_magnitude,
            use_noise=props.use_noise,
            noise_type=props.noise_type,
            noise_mean=props.noise_mean,
            noise_stddev=props.noise_stddev,
        )

        # Save preset
        manager = get_preset_manager()
        manager.save_sensor_preset(preset)

        self.report({"INFO"}, f"Saved sensor preset '{self.preset_name}'")
        return {"FINISHED"}


# Registration
def register():
    """Register operators."""
    if bpy is not None:
        bpy.utils.register_class(LINKFORGE_OT_apply_joint_preset)
        bpy.utils.register_class(LINKFORGE_OT_save_joint_preset)
        bpy.utils.register_class(LINKFORGE_OT_apply_material_preset)
        bpy.utils.register_class(LINKFORGE_OT_save_material_preset)
        bpy.utils.register_class(LINKFORGE_OT_apply_sensor_preset)
        bpy.utils.register_class(LINKFORGE_OT_save_sensor_preset)


def unregister():
    """Unregister operators."""
    if bpy is not None:
        bpy.utils.unregister_class(LINKFORGE_OT_save_sensor_preset)
        bpy.utils.unregister_class(LINKFORGE_OT_apply_sensor_preset)
        bpy.utils.unregister_class(LINKFORGE_OT_save_material_preset)
        bpy.utils.unregister_class(LINKFORGE_OT_apply_material_preset)
        bpy.utils.unregister_class(LINKFORGE_OT_save_joint_preset)
        bpy.utils.unregister_class(LINKFORGE_OT_apply_joint_preset)


if __name__ == "__main__":
    register()
