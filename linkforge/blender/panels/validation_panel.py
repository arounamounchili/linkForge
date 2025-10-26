"""UI Panel for validation results."""

from __future__ import annotations

try:
    import bpy
    from bpy.types import Panel
except ImportError:
    # Allow importing without Blender
    bpy = None  # type: ignore
    Panel = object


class LINKFORGE_PT_validation_panel(Panel):
    """Panel showing validation results in 3D Viewport sidebar."""

    bl_label = "Validation"
    bl_idname = "LINKFORGE_PT_validation_panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "LinkForge"
    bl_order = 5

    def draw(self, context):
        """Draw the panel."""
        layout = self.layout

        # Validation button
        box = layout.box()
        box.label(text="Robot Validation:", icon="CHECKMARK")
        box.operator("linkforge.validate_robot", icon="PLAY")

        # Get stored validation results from window manager
        wm = context.window_manager
        if not hasattr(wm, "linkforge_validation"):
            return

        validation = wm.linkforge_validation

        # Only show results if validation has been run
        if not validation.has_results:
            box.separator()
            box.label(text="Run validation to see results", icon="INFO")
            return

        # Results box = layout.box()
        results_box = layout.box()

        # Status header
        if validation.is_valid and validation.warning_count == 0:
            # Valid, no warnings
            results_box.label(text="✓ Robot is valid!", icon="CHECKMARK")
            col = results_box.column(align=True)
            col.label(text=f"Links: {validation.link_count}", icon="MESH_CUBE")
            col.label(text=f"Joints: {validation.joint_count}", icon="EMPTY_AXIS")
            col.label(text=f"DOF: {validation.dof_count}", icon="CON_ROTLIKE")

        elif validation.is_valid:
            # Valid but has warnings
            results_box.label(
                text=f"⚠ Valid with {validation.warning_count} warning(s)",
                icon="ERROR",
            )

        else:
            # Has errors
            results_box.label(
                text=f"✗ {validation.error_count} error(s), {validation.warning_count} warning(s)",
                icon="CANCEL",
            )

        # Errors section
        if validation.error_count > 0:
            results_box.separator()
            error_box = results_box.box()
            error_box.label(text=f"Errors ({validation.error_count}):", icon="ERROR")

            # Toggle to show/hide errors
            error_box.prop(validation, "show_errors", toggle=True, icon="TRIA_DOWN" if validation.show_errors else "TRIA_RIGHT")

            if validation.show_errors:
                for i in range(validation.error_count):
                    error_item_box = error_box.box()
                    error_item = validation.get_error(i)

                    # Error title
                    row = error_item_box.row()
                    row.label(text=f"• {error_item.title}", icon="CANCEL")

                    # Error message
                    col = error_item_box.column(align=True)
                    # Wrap message text
                    for line in error_item.message_lines:
                        col.label(text=line)

                    # Affected objects
                    if error_item.has_objects:
                        col.separator(factor=0.5)
                        col.label(text=f"Affected: {error_item.objects_str}", icon="OBJECT_DATA")

                    # Suggestion
                    if error_item.has_suggestion:
                        col.separator(factor=0.5)
                        col.label(text="Fix:", icon="INFO")
                        for line in error_item.suggestion_lines:
                            col.label(text=f"  {line}")

        # Warnings section
        if validation.warning_count > 0:
            results_box.separator()
            warning_box = results_box.box()
            warning_box.label(text=f"Warnings ({validation.warning_count}):", icon="ERROR")

            # Toggle to show/hide warnings
            warning_box.prop(validation, "show_warnings", toggle=True, icon="TRIA_DOWN" if validation.show_warnings else "TRIA_RIGHT")

            if validation.show_warnings:
                for i in range(validation.warning_count):
                    warning_item_box = warning_box.box()
                    warning_item = validation.get_warning(i)

                    # Warning title
                    row = warning_item_box.row()
                    row.label(text=f"• {warning_item.title}", icon="ERROR")

                    # Warning message
                    col = warning_item_box.column(align=True)
                    for line in warning_item.message_lines:
                        col.label(text=line)

                    # Affected objects
                    if warning_item.has_objects:
                        col.separator(factor=0.5)
                        col.label(text=f"Affected: {warning_item.objects_str}", icon="OBJECT_DATA")

                    # Suggestion
                    if warning_item.has_suggestion:
                        col.separator(factor=0.5)
                        col.label(text="Suggestion:", icon="INFO")
                        for line in warning_item.suggestion_lines:
                            col.label(text=f"  {line}")


# Registration
classes = [
    LINKFORGE_PT_validation_panel,
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


if __name__ == "__main__":
    register()
