"""Operators for robot templates."""

from __future__ import annotations

try:
    import bpy
    from bpy.props import StringProperty
    from bpy.types import Operator
except ImportError:
    # Allow importing without Blender
    bpy = None  # type: ignore
    Operator = object
    StringProperty = lambda **kwargs: None  # type: ignore  # noqa: E731


class LINKFORGE_OT_instantiate_template(Operator):
    """Create a robot from a template"""

    bl_idname = "linkforge.instantiate_template"
    bl_label = "Create from Template"
    bl_description = "Create a robot from the selected template"
    bl_options = {"REGISTER", "UNDO"}

    template_id: StringProperty(  # type: ignore
        name="Template ID",
        description="ID of the template to instantiate",
        default="",
    )

    def execute(self, context):
        """Execute the operator."""
        if not self.template_id:
            self.report({"ERROR"}, "No template ID specified")
            return {"CANCELLED"}

        try:
            # Import with absolute imports for Blender extension compatibility
            from pathlib import Path

            from linkforge.templates.loader import get_template
            from linkforge.blender.utils.urdf_importer import import_robot_to_scene

            # Get the template
            template = get_template(self.template_id)
            if template is None:
                self.report({"ERROR"}, f"Template '{self.template_id}' not found")
                return {"CANCELLED"}

            # Create the robot model
            robot = template.create_robot()

            # Import to scene (use dummy path since templates don't have external meshes)
            dummy_path = Path("/tmp/template.urdf")
            success = import_robot_to_scene(robot, dummy_path, context)

            if success:
                self.report(
                    {"INFO"}, f"Created robot '{robot.name}' from template '{template.name}'"
                )
                return {"FINISHED"}
            else:
                self.report({"ERROR"}, "Failed to import robot to scene")
                return {"CANCELLED"}

        except Exception as e:
            self.report({"ERROR"}, f"Failed to create template: {e}")
            return {"CANCELLED"}


classes = [
    LINKFORGE_OT_instantiate_template,
]


def register():
    """Register operators."""
    if bpy is not None:
        for cls in classes:
            bpy.utils.register_class(cls)


def unregister():
    """Unregister operators."""
    if bpy is not None:
        for cls in reversed(classes):
            bpy.utils.unregister_class(cls)
