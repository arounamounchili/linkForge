"""Blender Property Groups for robot joints.

These properties are stored on Empty objects and define joint characteristics.
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


def update_joint_name(self, context):
    """Update callback when joint_name changes - sync to object name."""
    if not bpy:
        return

    # Find the object that owns this property
    obj = None
    for o in context.scene.objects:
        if hasattr(o, "linkforge_joint") and o.linkforge_joint == self:
            obj = o
            break

    if obj is None:
        return

    # Only sync if this is marked as a robot joint and has a name
    if not self.is_robot_joint or not self.joint_name:
        return

    # Import sanitize function from link_props
    from .link_props import sanitize_urdf_name

    # Sanitize joint name for URDF
    sanitized_name = sanitize_urdf_name(self.joint_name)

    # Update object name to match joint name
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


class JointPropertyGroup(PropertyGroup):
    """Properties for a robot joint stored on an Empty object."""

    # Joint identification
    is_robot_joint: BoolProperty(  # type: ignore
        name="Is Robot Joint",
        description="Mark this Empty as a robot joint",
        default=False,
    )

    joint_name: StringProperty(  # type: ignore
        name="Joint Name",
        description="Name of the joint in URDF (must be unique)",
        default="",
        maxlen=64,
        update=update_joint_name,
    )

    # Joint type
    joint_type: EnumProperty(  # type: ignore
        name="Joint Type",
        description="Type of joint connection",
        items=[
            ("REVOLUTE", "Revolute", "Rotates around axis with limits"),
            ("CONTINUOUS", "Continuous", "Rotates around axis without limits"),
            ("PRISMATIC", "Prismatic", "Slides along axis with limits"),
            ("FIXED", "Fixed", "No motion allowed"),
            ("FLOATING", "Floating", "6 DOF free in space"),
            ("PLANAR", "Planar", "2D motion in a plane"),
        ],
        default="REVOLUTE",
    )

    # Parent and child links
    parent_link: EnumProperty(  # type: ignore
        name="Parent Link",
        description="Select the parent link",
        items=get_available_links,
    )

    child_link: EnumProperty(  # type: ignore
        name="Child Link",
        description="Select the child link",
        items=get_available_links,
    )

    # Joint axis
    axis: EnumProperty(  # type: ignore
        name="Axis",
        description="Joint rotation/translation axis",
        items=[
            ("X", "X", "X axis (red)"),
            ("Y", "Y", "Y axis (green)"),
            ("Z", "Z", "Z axis (blue)"),
            ("CUSTOM", "Custom", "Custom axis direction"),
        ],
        default="Z",
    )

    # Custom axis (when axis is CUSTOM)
    custom_axis_x: FloatProperty(  # type: ignore
        name="Axis X",
        description="Custom axis X component",
        default=0.0,
        min=-1.0,
        max=1.0,
    )

    custom_axis_y: FloatProperty(  # type: ignore
        name="Axis Y",
        description="Custom axis Y component",
        default=0.0,
        min=-1.0,
        max=1.0,
    )

    custom_axis_z: FloatProperty(  # type: ignore
        name="Axis Z",
        description="Custom axis Z component",
        default=1.0,
        min=-1.0,
        max=1.0,
    )

    # Joint limits (for revolute and prismatic)
    use_limits: BoolProperty(  # type: ignore
        name="Use Limits",
        description="Enable joint limits",
        default=True,
    )

    limit_lower: FloatProperty(  # type: ignore
        name="Lower Limit",
        description="Lower limit (radians for revolute, meters for prismatic)",
        default=-3.14159,  # -π
        soft_min=-6.28318,  # -2π
        soft_max=6.28318,  # 2π
    )

    limit_upper: FloatProperty(  # type: ignore
        name="Upper Limit",
        description="Upper limit (radians for revolute, meters for prismatic)",
        default=3.14159,  # π
        soft_min=-6.28318,
        soft_max=6.28318,
    )

    limit_effort: FloatProperty(  # type: ignore
        name="Max Effort",
        description="Maximum effort (Nm for revolute, N for prismatic)",
        default=10.0,
        min=0.0,
        soft_max=100.0,
    )

    limit_velocity: FloatProperty(  # type: ignore
        name="Max Velocity",
        description="Maximum velocity (rad/s for revolute, m/s for prismatic)",
        default=1.0,
        min=0.0,
        soft_max=10.0,
    )

    # Joint dynamics
    use_dynamics: BoolProperty(  # type: ignore
        name="Use Dynamics",
        description="Enable dynamics properties",
        default=False,
    )

    dynamics_damping: FloatProperty(  # type: ignore
        name="Damping",
        description="Damping coefficient",
        default=0.0,
        min=0.0,
        soft_max=10.0,
    )

    dynamics_friction: FloatProperty(  # type: ignore
        name="Friction",
        description="Friction coefficient",
        default=0.0,
        min=0.0,
        soft_max=10.0,
    )

    # Mimic joint
    use_mimic: BoolProperty(  # type: ignore
        name="Mimic Another Joint",
        description="This joint mimics another joint's motion",
        default=False,
    )

    mimic_joint: StringProperty(  # type: ignore
        name="Mimic Joint",
        description="Name of the joint to mimic",
        default="",
    )

    mimic_multiplier: FloatProperty(  # type: ignore
        name="Multiplier",
        description="Motion multiplier",
        default=1.0,
    )

    mimic_offset: FloatProperty(  # type: ignore
        name="Offset",
        description="Motion offset",
        default=0.0,
    )


# Registration
def register():
    """Register property group."""
    if bpy is not None:
        bpy.utils.register_class(JointPropertyGroup)
        bpy.types.Object.linkforge_joint = bpy.props.PointerProperty(type=JointPropertyGroup)


def unregister():
    """Unregister property group."""
    if bpy is not None:
        del bpy.types.Object.linkforge_joint
        bpy.utils.unregister_class(JointPropertyGroup)


if __name__ == "__main__":
    register()
