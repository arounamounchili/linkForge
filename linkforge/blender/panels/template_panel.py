"""UI Panel for robot templates."""

from __future__ import annotations

try:
    import bpy
    from bpy.types import Panel
except ImportError:
    # Allow importing without Blender
    bpy = None  # type: ignore
    Panel = object


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

        # Import templates
        try:
            from ...templates import library  # noqa: F401
            from ...templates.loader import get_all_templates, get_template_categories
        except ImportError:
            layout.label(text="Template system not available", icon="ERROR")
            return

        # Get all templates organized by category
        templates = get_all_templates()
        categories = get_template_categories()

        if not templates:
            layout.label(text="No templates available", icon="INFO")
            return

        # Header
        box = layout.box()
        box.label(text="Create Robot from Template", icon="PRESET")

        # Group templates by category
        templates_by_category = {}
        for template in templates:
            cat = template.category
            if cat not in templates_by_category:
                templates_by_category[cat] = []
            templates_by_category[cat].append(template)

        # Display templates by category
        for category in sorted(categories):
            if category not in templates_by_category:
                continue

            # Category header
            box = layout.box()
            cat_name = category.value if hasattr(category, "value") else str(category)
            box.label(text=cat_name.title(), icon="OUTLINER_DATA_ARMATURE")

            # Templates in this category
            for template in templates_by_category[category]:
                row = box.row(align=True)
                row.scale_y = 1.2

                # Template button
                op = row.operator(
                    "linkforge.instantiate_template",
                    text=template.name,
                    icon="ADD",
                )
                op.template_id = template.id

                # Info icon (shows description on hover)
                info_row = row.row(align=True)
                info_row.enabled = False
                info_row.label(text="", icon="INFO")

                # Description as sublabel
                desc_row = box.row()
                desc_row.scale_y = 0.7
                desc_row.label(text=template.description)
                box.separator(factor=0.5)


classes = [
    LINKFORGE_PT_template_panel,
]


def register():
    """Register panels."""
    if bpy is not None:
        for cls in classes:
            bpy.utils.register_class(cls)


def unregister():
    """Unregister panels."""
    if bpy is not None:
        for cls in reversed(classes):
            bpy.utils.unregister_class(cls)
