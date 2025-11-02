"""Blender Property Groups for robot links.

These properties are stored on Blender objects and define link characteristics.
"""

from __future__ import annotations

try:
    import bpy
    from bpy.props import (
        BoolProperty,
        EnumProperty,
        FloatProperty,
        FloatVectorProperty,
        StringProperty,
    )
    from bpy.types import PropertyGroup
except ImportError:
    # Allow importing without Blender for testing
    PropertyGroup = object
    BoolProperty = EnumProperty = FloatProperty = FloatVectorProperty = StringProperty = None


def update_link_name(self, context):
    """Update callback when link_name changes - sync to object name."""
    if not bpy:
        return

    # Find the object that owns this property
    obj = None
    for o in context.scene.objects:
        if hasattr(o, "linkforge") and o.linkforge == self:
            obj = o
            break

    if obj is None:
        return

    # Only sync if this is marked as a robot link and has a name
    if not self.is_robot_link or not self.link_name:
        return

    # Sanitize link name for URDF (remove invalid characters)
    sanitized_name = sanitize_urdf_name(self.link_name)

    # Update object name to match link name
    # Note: Blender may append .001, .002 etc if name conflicts exist
    if obj.name != sanitized_name:
        obj.name = sanitized_name


def sanitize_urdf_name(name: str) -> str:
    """Sanitize name for URDF compatibility.

    URDF names should only contain alphanumeric, underscore, and hyphen.
    """
    if not name:
        return name

    # Replace spaces with underscores
    name = name.replace(" ", "_")

    # Remove invalid characters (keep alphanumeric, underscore, hyphen)
    valid_chars = []
    for char in name:
        if char.isalnum() or char in ("_", "-"):
            valid_chars.append(char)

    return "".join(valid_chars)


class LinkPropertyGroup(PropertyGroup):
    """Properties for a robot link stored on a Blender object."""

    # Link identification
    is_robot_link: BoolProperty(  # type: ignore
        name="Is Robot Link",
        description="Mark this object as a robot link",
        default=False,
    )

    link_name: StringProperty(  # type: ignore
        name="Link Name",
        description="Name of the link in URDF (must be unique)",
        default="",
        maxlen=64,
        update=update_link_name,
    )

    # Inertial properties
    use_auto_inertia: BoolProperty(  # type: ignore
        name="Auto-Calculate Inertia",
        description="Automatically calculate inertia tensor from geometry",
        default=True,
    )

    mass: FloatProperty(  # type: ignore
        name="Mass",
        description="Link mass in kilograms",
        default=1.0,
        min=0.0,
        soft_max=1000.0,
        unit="MASS",
        precision=3,
    )

    # Manual inertia tensor (when auto_inertia is disabled)
    inertia_ixx: FloatProperty(  # type: ignore
        name="Ixx",
        description="Inertia tensor Ixx component",
        default=1.0,
        min=0.0,
        precision=6,
    )

    inertia_ixy: FloatProperty(  # type: ignore
        name="Ixy",
        description="Inertia tensor Ixy component",
        default=0.0,
        precision=6,
    )

    inertia_ixz: FloatProperty(  # type: ignore
        name="Ixz",
        description="Inertia tensor Ixz component",
        default=0.0,
        precision=6,
    )

    inertia_iyy: FloatProperty(  # type: ignore
        name="Iyy",
        description="Inertia tensor Iyy component",
        default=1.0,
        min=0.0,
        precision=6,
    )

    inertia_iyz: FloatProperty(  # type: ignore
        name="Iyz",
        description="Inertia tensor Iyz component",
        default=0.0,
        precision=6,
    )

    inertia_izz: FloatProperty(  # type: ignore
        name="Izz",
        description="Inertia tensor Izz component",
        default=1.0,
        min=0.0,
        precision=6,
    )

    # Geometry type for collision
    collision_geometry_type: EnumProperty(  # type: ignore
        name="Collision Geometry",
        description="Geometry type for collision detection",
        items=[
            ("MESH", "Mesh", "Use mesh geometry (convex hull)"),
            ("BOX", "Box", "Use bounding box"),
            ("CYLINDER", "Cylinder", "Use bounding cylinder"),
            ("SPHERE", "Sphere", "Use bounding sphere"),
            ("CAPSULE", "Capsule", "Use bounding capsule"),
        ],
        default="MESH",
    )

    # Visual geometry
    use_visual_geometry: BoolProperty(  # type: ignore
        name="Use Visual Geometry",
        description="Include visual geometry in URDF",
        default=True,
    )

    visual_geometry_type: EnumProperty(  # type: ignore
        name="Visual Geometry",
        description="Geometry type for visual representation",
        items=[
            ("MESH", "Mesh", "Export as mesh (STL/DAE)"),
            ("BOX", "Box", "Use bounding box"),
            ("CYLINDER", "Cylinder", "Use bounding cylinder"),
            ("SPHERE", "Sphere", "Use bounding sphere"),
            ("CAPSULE", "Capsule", "Use bounding capsule"),
        ],
        default="MESH",
    )

    # Export options
    export_collision: BoolProperty(  # type: ignore
        name="Export Collision",
        description="Include collision geometry in URDF",
        default=True,
    )

    simplify_collision: BoolProperty(  # type: ignore
        name="Simplify Collision Mesh",
        description="Simplify collision mesh using decimation",
        default=True,
    )

    collision_decimation_ratio: FloatProperty(  # type: ignore
        name="Decimation Ratio",
        description="Target ratio of faces to keep (0.1 = 10% of original)",
        default=0.5,
        min=0.01,
        max=1.0,
        precision=2,
    )

    # Material properties
    use_material: BoolProperty(  # type: ignore
        name="Use Material",
        description="Export material/color for this link",
        default=False,
    )

    material_preset: EnumProperty(  # type: ignore
        name="Material Preset",
        description="Choose from predefined colors or custom",
        items=[
            ("CUSTOM", "Custom", "Use custom color picker"),
            ("RED", "Red", "Red color"),
            ("GREEN", "Green", "Green color"),
            ("BLUE", "Blue", "Blue color"),
            ("YELLOW", "Yellow", "Yellow color"),
            ("ORANGE", "Orange", "Orange color"),
            ("CYAN", "Cyan", "Cyan color"),
            ("MAGENTA", "Magenta", "Magenta color"),
            ("WHITE", "White", "White color"),
            ("BLACK", "Black", "Black color"),
            ("GRAY", "Gray", "Gray color"),
            ("ALUMINUM", "Aluminum", "Metallic aluminum color"),
            ("STEEL", "Steel", "Steel gray color"),
            ("PLASTIC_BLACK", "Plastic Black", "Black plastic color"),
            ("RUBBER", "Rubber", "Dark rubber color"),
        ],
        default="GRAY",
    )

    material_color: FloatVectorProperty(  # type: ignore
        name="Custom Color",
        description="Custom RGBA color for this link",
        subtype="COLOR",
        size=4,
        min=0.0,
        max=1.0,
        default=(0.8, 0.8, 0.8, 1.0),  # Light gray
    )

    material_name: StringProperty(  # type: ignore
        name="Material Name",
        description="Name of the material in URDF",
        default="",
        maxlen=64,
    )


# Registration
def register():
    """Register property group."""
    if bpy is not None:
        bpy.utils.register_class(LinkPropertyGroup)
        bpy.types.Object.linkforge = bpy.props.PointerProperty(type=LinkPropertyGroup)


def unregister():
    """Unregister property group."""
    if bpy is not None:
        del bpy.types.Object.linkforge
        bpy.utils.unregister_class(LinkPropertyGroup)


if __name__ == "__main__":
    register()
