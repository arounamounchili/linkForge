"""Blender Property Groups for robot transmissions.

These properties are stored on Empty objects and define transmission characteristics
for ros2_control integration.
"""

from __future__ import annotations

try:
    import bpy
    from bpy.props import BoolProperty, EnumProperty, FloatProperty, StringProperty
    from bpy.types import PropertyGroup
except ImportError:
    # Allow importing without Blender for testing
    PropertyGroup = object
    BoolProperty = EnumProperty = FloatProperty = StringProperty = None


def update_transmission_name(self, context):
    """Update callback when transmission_name changes - sync to object name."""
    if not bpy:
        return

    # Find the object that owns this property
    obj = None
    for o in context.scene.objects:
        if hasattr(o, "linkforge_transmission") and o.linkforge_transmission == self:
            obj = o
            break

    if obj is None:
        return

    # Only sync if this is marked as a transmission and has a name
    if not self.is_robot_transmission or not self.transmission_name:
        return

    # Import sanitize function from link_props
    from .link_props import sanitize_urdf_name

    # Sanitize transmission name for URDF
    sanitized_name = sanitize_urdf_name(self.transmission_name)

    # Update object name to match transmission name
    if obj.name != sanitized_name:
        obj.name = sanitized_name


def get_available_joints(self, context):
    """Get list of available joints in the scene for dropdown."""
    import bpy

    items = [("NONE", "None", "No joint selected")]

    # Use bpy.context.scene to ensure we get the current scene
    try:
        scene = bpy.context.scene
    except AttributeError:
        # Fallback to passed context
        if context and hasattr(context, "scene"):
            scene = context.scene
        else:
            return items

    # Find all objects marked as joints
    for obj in scene.objects:
        if hasattr(obj, "linkforge_joint") and obj.linkforge_joint.is_robot_joint:
            joint_name = obj.linkforge_joint.joint_name
            if joint_name:
                items.append((joint_name, joint_name, f"Joint: {joint_name}"))

    return items


class TransmissionPropertyGroup(PropertyGroup):
    """Properties for a robot transmission stored on an Empty object."""

    # Transmission identification
    is_robot_transmission: BoolProperty(  # type: ignore
        name="Is Robot Transmission",
        description="Mark this Empty as a robot transmission",
        default=False,
    )

    transmission_name: StringProperty(  # type: ignore
        name="Transmission Name",
        description="Name of the transmission in URDF (must be unique)",
        default="",
        maxlen=64,
        update=update_transmission_name,
    )

    # Transmission type
    transmission_type: EnumProperty(  # type: ignore
        name="Transmission Type",
        description="Type of transmission mechanism",
        items=[
            ("SIMPLE", "Simple", "1:1 joint to actuator transmission"),
            ("DIFFERENTIAL", "Differential", "2 joints to 2 actuators (differential drive)"),
            ("FOUR_BAR_LINKAGE", "Four-Bar Linkage", "Four-bar linkage transmission"),
            ("CUSTOM", "Custom", "Custom transmission type"),
        ],
        default="SIMPLE",
    )

    # Custom type (when transmission_type is CUSTOM)
    custom_type: StringProperty(  # type: ignore
        name="Custom Type",
        description="Custom transmission type identifier",
        default="",
        maxlen=128,
    )

    # Joint selection (for simple transmission)
    joint_name: EnumProperty(  # type: ignore
        name="Joint",
        description="Joint controlled by this transmission",
        items=get_available_joints,
    )

    # Joint selection (for differential transmission)
    joint1_name: EnumProperty(  # type: ignore
        name="Joint 1",
        description="First joint in differential transmission",
        items=get_available_joints,
    )

    joint2_name: EnumProperty(  # type: ignore
        name="Joint 2",
        description="Second joint in differential transmission",
        items=get_available_joints,
    )

    # Hardware interface
    hardware_interface: EnumProperty(  # type: ignore
        name="Hardware Interface",
        description="Control interface type",
        items=[
            ("POSITION", "Position", "Position control (ROS2 format)"),
            ("VELOCITY", "Velocity", "Velocity control (ROS2 format)"),
            ("EFFORT", "Effort", "Effort/torque control (ROS2 format)"),
            ("POSITION_ROS1", "Position (ROS1)", "Position control (ROS1 format)"),
            ("VELOCITY_ROS1", "Velocity (ROS1)", "Velocity control (ROS1 format)"),
            ("EFFORT_ROS1", "Effort (ROS1)", "Effort control (ROS1 format)"),
        ],
        default="POSITION",
    )

    # Mechanical properties
    mechanical_reduction: FloatProperty(  # type: ignore
        name="Mechanical Reduction",
        description="Gear reduction ratio (actuator/joint)",
        default=1.0,
        min=0.001,
        soft_max=1000.0,
        precision=3,
    )

    offset: FloatProperty(  # type: ignore
        name="Offset",
        description="Joint offset in radians or meters",
        default=0.0,
        precision=5,
    )

    # Actuator naming
    use_custom_actuator_name: BoolProperty(  # type: ignore
        name="Custom Actuator Name",
        description="Use custom actuator name instead of auto-generated",
        default=False,
    )

    actuator_name: StringProperty(  # type: ignore
        name="Actuator Name",
        description="Name of the actuator (motor)",
        default="",
        maxlen=64,
    )

    # For differential transmissions
    actuator1_name: StringProperty(  # type: ignore
        name="Actuator 1 Name",
        description="Name of the first actuator",
        default="",
        maxlen=64,
    )

    actuator2_name: StringProperty(  # type: ignore
        name="Actuator 2 Name",
        description="Name of the second actuator",
        default="",
        maxlen=64,
    )


# Registration
def register():
    """Register property group."""
    if bpy is not None:
        bpy.utils.register_class(TransmissionPropertyGroup)
        bpy.types.Object.linkforge_transmission = bpy.props.PointerProperty(
            type=TransmissionPropertyGroup
        )


def unregister():
    """Unregister property group."""
    if bpy is not None:
        del bpy.types.Object.linkforge_transmission
        bpy.utils.unregister_class(TransmissionPropertyGroup)


if __name__ == "__main__":
    register()
