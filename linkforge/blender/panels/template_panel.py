"""UI Panel for robot templates."""

from __future__ import annotations

try:
    import bpy
    from bpy.props import EnumProperty
    from bpy.types import Panel, PropertyGroup
except ImportError:
    # Allow importing without Blender
    bpy = None  # type: ignore
    Panel = object
    PropertyGroup = object
    EnumProperty = lambda **kwargs: None  # type: ignore  # noqa: E731


def get_template_items(self, context):
    """Get template items for enum property."""
    try:
        from linkforge.templates import library  # noqa: F401
        from linkforge.templates.loader import get_all_templates

        templates = get_all_templates()
        items = []

        for template in templates:
            cat_name = (
                template.category.value
                if hasattr(template.category, "value")
                else str(template.category)
            )
            items.append(
                (
                    template.id,
                    f"{template.name} ({cat_name})",
                    template.description,
                )
            )

        return items if items else [("NONE", "No templates available", "")]

    except Exception as e:
        print(f"Error loading templates: {e}")
        return [("ERROR", "Error loading templates", str(e))]


class TemplateProperties(PropertyGroup):
    """Properties for template selection."""

    selected_template: EnumProperty(  # type: ignore
        name="Template",
        description="Select a robot template to create",
        items=get_template_items,
    )


class LINKFORGE_PT_template_panel(Panel):
    """Panel for robot templates in 3D Viewport sidebar."""

    bl_label = "Templates"
    bl_idname = "LINKFORGE_PT_template_panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "LinkForge"
    bl_order = 1

    def draw(self, context):
        """Draw the panel."""
        layout = self.layout
        props = context.scene.linkforge_templates

        # Header
        box = layout.box()
        box.label(text="Create Robot from Template", icon="PRESET")

        # Template selector
        box.prop(props, "selected_template", text="")

        # Create button
        row = box.row()
        row.scale_y = 1.5
        op = row.operator(
            "linkforge.instantiate_template",
            text="Create Robot",
            icon="ADD",
        )
        op.template_id = props.selected_template

        # Info text
        if props.selected_template and props.selected_template not in ("NONE", "ERROR"):
            box.separator()
            col = box.column(align=True)
            col.scale_y = 0.8
            col.label(text="Creates a ready-to-use robot", icon="INFO")
            col.label(text="with all links and joints")


classes = [
    TemplateProperties,
    LINKFORGE_PT_template_panel,
]


def register():
    """Register panels and properties."""
    if bpy is not None:
        for cls in classes:
            bpy.utils.register_class(cls)
        # Register template properties on scene
        bpy.types.Scene.linkforge_templates = bpy.props.PointerProperty(type=TemplateProperties)


def unregister():
    """Unregister panels and properties."""
    if bpy is not None:
        # Unregister scene property
        del bpy.types.Scene.linkforge_templates
        # Unregister classes
        for cls in reversed(classes):
            bpy.utils.unregister_class(cls)
