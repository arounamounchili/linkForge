"""Blender Property Groups for robot sensors.

These properties are stored on Empty objects and define sensor characteristics.
"""

from __future__ import annotations

try:
    import bpy
    from bpy.props import BoolProperty, EnumProperty, FloatProperty, IntProperty, StringProperty
    from bpy.types import PropertyGroup
except ImportError:
    # Allow importing without Blender for testing
    PropertyGroup = object
    BoolProperty = EnumProperty = FloatProperty = IntProperty = StringProperty = None


def update_sensor_name(self, context):
    """Update callback when sensor_name changes - sync to object name."""
    if not bpy:
        return

    # Find the object that owns this property
    obj = None
    for o in context.scene.objects:
        if hasattr(o, "linkforge_sensor") and o.linkforge_sensor == self:
            obj = o
            break

    if obj is None:
        return

    # Only sync if this is marked as a sensor and has a name
    if not self.is_robot_sensor or not self.sensor_name:
        return

    # Import sanitize function from link_props
    from .link_props import sanitize_urdf_name

    # Sanitize sensor name for URDF
    sanitized_name = sanitize_urdf_name(self.sensor_name)

    # Update object name to match sensor name
    if obj.name != sanitized_name:
        obj.name = sanitized_name


def get_available_links(self, context):
    """Get list of available links in the scene for dropdown."""
    import bpy

    items = [("NONE", "None", "No link selected")]

    # Use bpy.context.scene to ensure we get the current scene
    try:
        scene = bpy.context.scene
    except AttributeError:
        # Fallback to passed context
        if context and hasattr(context, "scene"):
            scene = context.scene
        else:
            return items

    # Find all objects marked as links
    for obj in scene.objects:
        if hasattr(obj, "linkforge") and obj.linkforge.is_robot_link:
            link_name = obj.linkforge.link_name
            if link_name:
                items.append((link_name, link_name, f"Link: {link_name}"))

    return items


class SensorPropertyGroup(PropertyGroup):
    """Properties for a robot sensor stored on an Empty object."""

    # Sensor identification
    is_robot_sensor: BoolProperty(  # type: ignore
        name="Is Robot Sensor",
        description="Mark this Empty as a robot sensor",
        default=False,
    )

    sensor_name: StringProperty(  # type: ignore
        name="Sensor Name",
        description="Name of the sensor in URDF (must be unique)",
        default="",
        maxlen=64,
        update=update_sensor_name,
    )

    # Sensor type
    sensor_type: EnumProperty(  # type: ignore
        name="Sensor Type",
        description="Type of sensor",
        items=[
            ("CAMERA", "Camera", "RGB camera sensor"),
            ("DEPTH_CAMERA", "Depth Camera", "Depth/RGBD camera sensor"),
            ("LIDAR", "LIDAR", "2D/3D laser scanner"),
            ("IMU", "IMU", "Inertial measurement unit"),
            ("GPS", "GPS", "Global positioning system"),
            ("CONTACT", "Contact", "Contact sensor"),
            ("FORCE_TORQUE", "Force/Torque", "Force-torque sensor"),
        ],
        default="CAMERA",
    )

    # Attached link
    attached_link: EnumProperty(  # type: ignore
        name="Attached Link",
        description="Select the link this sensor is attached to",
        items=get_available_links,
    )

    # Common sensor properties
    update_rate: FloatProperty(  # type: ignore
        name="Update Rate",
        description="Sensor update rate in Hz",
        default=30.0,
        min=0.1,
        soft_max=100.0,
        precision=1,
    )

    topic_name: StringProperty(  # type: ignore
        name="Topic Name",
        description="ROS topic name for sensor data",
        default="",
        maxlen=128,
    )

    # Camera-specific properties
    camera_horizontal_fov: FloatProperty(  # type: ignore
        name="Horizontal FOV",
        description="Camera horizontal field of view in radians",
        default=1.047,  # ~60 degrees
        min=0.1,
        max=3.14159,
        precision=3,
        subtype="ANGLE",
    )

    camera_width: IntProperty(  # type: ignore
        name="Image Width",
        description="Camera image width in pixels",
        default=640,
        min=1,
        soft_max=1920,
    )

    camera_height: IntProperty(  # type: ignore
        name="Image Height",
        description="Camera image height in pixels",
        default=480,
        min=1,
        soft_max=1080,
    )

    camera_near_clip: FloatProperty(  # type: ignore
        name="Near Clip",
        description="Camera near clipping plane distance",
        default=0.1,
        min=0.001,
        soft_max=10.0,
        precision=3,
    )

    camera_far_clip: FloatProperty(  # type: ignore
        name="Far Clip",
        description="Camera far clipping plane distance",
        default=100.0,
        min=0.1,
        soft_max=1000.0,
        precision=1,
    )

    # LIDAR-specific properties
    lidar_horizontal_samples: IntProperty(  # type: ignore
        name="Horizontal Samples",
        description="Number of horizontal scan samples",
        default=640,
        min=1,
        soft_max=2048,
    )

    lidar_horizontal_min_angle: FloatProperty(  # type: ignore
        name="Horizontal Min Angle",
        description="Minimum horizontal scan angle in radians",
        default=-1.570796,  # -90 degrees
        min=-3.14159,
        max=3.14159,
        precision=3,
        subtype="ANGLE",
    )

    lidar_horizontal_max_angle: FloatProperty(  # type: ignore
        name="Horizontal Max Angle",
        description="Maximum horizontal scan angle in radians",
        default=1.570796,  # 90 degrees
        min=-3.14159,
        max=3.14159,
        precision=3,
        subtype="ANGLE",
    )

    lidar_vertical_samples: IntProperty(  # type: ignore
        name="Vertical Samples",
        description="Number of vertical scan samples (1 for 2D LIDAR)",
        default=1,
        min=1,
        soft_max=128,
    )

    lidar_range_min: FloatProperty(  # type: ignore
        name="Range Min",
        description="Minimum detection range in meters",
        default=0.1,
        min=0.001,
        soft_max=10.0,
        precision=3,
    )

    lidar_range_max: FloatProperty(  # type: ignore
        name="Range Max",
        description="Maximum detection range in meters",
        default=10.0,
        min=0.1,
        soft_max=100.0,
        precision=1,
    )

    # IMU-specific properties
    imu_gravity_magnitude: FloatProperty(  # type: ignore
        name="Gravity Magnitude",
        description="Gravity magnitude in m/s²",
        default=9.80665,
        min=0.0,
        soft_max=20.0,
        precision=5,
    )

    # Noise properties
    use_noise: BoolProperty(  # type: ignore
        name="Use Noise",
        description="Add realistic noise to sensor measurements",
        default=False,
    )

    noise_type: EnumProperty(  # type: ignore
        name="Noise Type",
        description="Type of noise model",
        items=[
            ("gaussian", "Gaussian", "Gaussian noise"),
            ("gaussian_quantized", "Gaussian Quantized", "Quantized Gaussian noise"),
        ],
        default="gaussian",
    )

    noise_mean: FloatProperty(  # type: ignore
        name="Noise Mean",
        description="Mean of the noise distribution",
        default=0.0,
        precision=5,
    )

    noise_stddev: FloatProperty(  # type: ignore
        name="Noise Std Dev",
        description="Standard deviation of the noise",
        default=0.0,
        min=0.0,
        precision=5,
    )

    # Gazebo plugin settings
    use_gazebo_plugin: BoolProperty(  # type: ignore
        name="Use Gazebo Plugin",
        description="Enable Gazebo plugin for this sensor",
        default=True,
    )

    plugin_filename: StringProperty(  # type: ignore
        name="Plugin Filename",
        description="Gazebo plugin library filename (e.g., libgazebo_ros_camera.so)",
        default="",
        maxlen=128,
    )


# Registration
def register():
    """Register property group."""
    if bpy is not None:
        bpy.utils.register_class(SensorPropertyGroup)
        bpy.types.Object.linkforge_sensor = bpy.props.PointerProperty(type=SensorPropertyGroup)


def unregister():
    """Unregister property group."""
    if bpy is not None:
        del bpy.types.Object.linkforge_sensor
        bpy.utils.unregister_class(SensorPropertyGroup)


if __name__ == "__main__":
    register()
