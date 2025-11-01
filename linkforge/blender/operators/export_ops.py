"""Operators for exporting robot to URDF/XACRO."""

from __future__ import annotations

try:
    import bpy
    from bpy.props import StringProperty
    from bpy.types import Operator
    from bpy_extras.io_utils import ExportHelper, ImportHelper
except ImportError:
    # Allow importing without Blender
    bpy = None  # type: ignore
    Operator = object
    ExportHelper = object
    ImportHelper = object
    StringProperty = None  # type: ignore


class LINKFORGE_OT_export_urdf(Operator, ExportHelper):
    """Export robot to URDF file"""

    bl_idname = "linkforge.export_urdf"
    bl_label = "Export URDF"
    bl_description = "Export robot to URDF/XACRO file"

    # ExportHelper properties
    filename_ext = ".urdf"
    filter_glob: StringProperty(  # type: ignore
        default="*.urdf;*.xacro",
        options={"HIDDEN"},
    )

    def execute(self, context):
        """Execute the export."""
        # Import here to avoid circular dependencies
        from pathlib import Path

        from ...core.generators import URDFGenerator, XACROGenerator
        from ..utils.converters import scene_to_robot

        scene = context.scene
        robot_props = scene.linkforge

        # Prepare meshes directory if exporting meshes
        output_path = Path(self.filepath)
        meshes_dir = None

        if robot_props.export_meshes:
            # Create meshes directory alongside URDF
            meshes_dir = output_path.parent / robot_props.mesh_directory_name
            meshes_dir.mkdir(parents=True, exist_ok=True)

        # Convert scene to robot model
        try:
            robot = scene_to_robot(context, meshes_dir=meshes_dir)
        except Exception as e:
            self.report({"ERROR"}, f"Failed to convert scene to robot: {e}")
            return {"CANCELLED"}

        # Validate if requested
        if robot_props.validate_before_export:
            from ...core.validation import RobotValidator

            validator = RobotValidator(robot)
            result = validator.validate()

            if not result.is_valid:
                self.report(
                    {"ERROR"},
                    f"Cannot export: {result.error_count} validation error(s). "
                    f"Run validation to see details.",
                )
                return {"CANCELLED"}

        # Generate URDF/XACRO
        try:
            if robot_props.export_format == "URDF":
                urdf_generator = URDFGenerator(pretty_print=True, urdf_path=output_path)
                urdf_generator.write(robot, output_path)
                msg = f"✓ Exported URDF to {output_path}"
                if meshes_dir:
                    msg += f" (meshes in {meshes_dir})"
                self.report({"INFO"}, msg)
                print(msg)  # Also print to console
            else:  # XACRO
                xacro_generator = XACROGenerator(pretty_print=True)
                xacro_generator.write(robot, output_path)
                msg = f"✓ Exported XACRO to {output_path}"
                if meshes_dir:
                    msg += f" (meshes in {meshes_dir})"
                self.report({"INFO"}, msg)
                print(msg)  # Also print to console

            return {"FINISHED"}

        except Exception as e:
            self.report({"ERROR"}, f"Export failed: {e}")
            return {"CANCELLED"}


class LINKFORGE_OT_import_urdf(Operator, ImportHelper):
    """Import robot from URDF or XACRO file"""

    bl_idname = "linkforge.import_urdf"
    bl_label = "Import Robot"
    bl_description = "Import robot from URDF or XACRO file (auto-detects format)"

    # ImportHelper properties
    filename_ext = ".urdf"
    filter_glob: StringProperty(  # type: ignore
        default="*.urdf;*.xacro;*.urdf.xacro",
        options={"HIDDEN"},
    )

    def execute(self, context):
        """Execute the import."""
        from pathlib import Path

        from ...core.parsers.urdf_parser import parse_urdf, parse_urdf_string
        from ..utils.urdf_importer import import_robot_to_scene

        # Parse URDF/XACRO file
        urdf_path = Path(self.filepath)

        # Detect if this is a XACRO file
        is_xacro = urdf_path.suffix == ".xacro" or urdf_path.name.endswith(".urdf.xacro")

        try:
            if is_xacro:
                # Convert XACRO to URDF using xacrodoc (bundled dependency)
                from xacrodoc import XacroDoc

                self.report({"INFO"}, f"Converting XACRO file: {urdf_path.name}")
                doc = XacroDoc.from_file(str(urdf_path))
                urdf_string = doc.to_urdf_string()

                # Parse URDF string
                self.report({"INFO"}, "Parsing URDF...")
                robot = parse_urdf_string(urdf_string)
            else:
                # Standard URDF import
                robot = parse_urdf(urdf_path)

        except FileNotFoundError:
            self.report({"ERROR"}, f"File not found: {urdf_path}")
            return {"CANCELLED"}
        except Exception as e:
            self.report({"ERROR"}, f"Failed to parse file: {e}")
            import traceback

            traceback.print_exc()
            return {"CANCELLED"}

        # Import to scene
        try:
            success = import_robot_to_scene(robot, urdf_path, context)
            if success:
                file_type = "XACRO" if is_xacro else "URDF"
                self.report(
                    {"INFO"},
                    f"Imported {file_type}: '{robot.name}' "
                    f"({len(robot.links)} links, {len(robot.joints)} joints)",
                )
                return {"FINISHED"}
            else:
                self.report({"ERROR"}, "Failed to import robot to scene")
                return {"CANCELLED"}

        except Exception as e:
            self.report({"ERROR"}, f"Import failed: {e}")
            import traceback

            traceback.print_exc()
            return {"CANCELLED"}


class LINKFORGE_OT_validate_robot(Operator):
    """Validate robot structure"""

    bl_idname = "linkforge.validate_robot"
    bl_label = "Validate Robot"
    bl_description = "Validate the robot structure for errors"

    def execute(self, context):
        """Execute validation."""
        from ...core.validation import RobotValidator
        from ..utils.converters import scene_to_robot

        # Clear previous results
        validation_props = context.window_manager.linkforge_validation
        validation_props.clear()

        # Convert scene to robot
        try:
            robot = scene_to_robot(context)
        except Exception as e:
            self.report({"ERROR"}, f"Failed to build robot: {e}")
            return {"CANCELLED"}

        # Validate using new validator
        validator = RobotValidator(robot)
        result = validator.validate()

        # Store results in window manager
        validation_props.has_results = True
        validation_props.is_valid = result.is_valid
        validation_props.error_count = result.error_count
        validation_props.warning_count = result.warning_count
        validation_props.link_count = len(robot.links)
        validation_props.joint_count = len(robot.joints)
        validation_props.dof_count = robot.degrees_of_freedom

        # Store errors
        for error in result.errors:
            error_prop = validation_props.errors.add()
            error_prop.title = error.title
            error_prop.message = error.message
            error_prop.suggestion = error.suggestion or ""
            error_prop.affected_objects = ", ".join(error.affected_objects)

        # Store warnings
        for warning in result.warnings:
            warning_prop = validation_props.warnings.add()
            warning_prop.title = warning.title
            warning_prop.message = warning.message
            warning_prop.suggestion = warning.suggestion or ""
            warning_prop.affected_objects = ", ".join(warning.affected_objects)

        # Report result
        if result.is_valid and not result.has_warnings:
            self.report(
                {"INFO"},
                f"✓ Robot '{robot.name}' is valid! "
                f"({len(robot.links)} links, {len(robot.joints)} joints, "
                f"{robot.degrees_of_freedom} DOF)",
            )
            return {"FINISHED"}
        elif result.is_valid:
            self.report(
                {"WARNING"},
                f"⚠ Robot valid with {result.warning_count} warning(s). Check Validation panel.",
            )
            return {"FINISHED"}
        else:
            self.report(
                {"ERROR"},
                f"✗ Validation failed: {result.error_count} error(s), {result.warning_count} warning(s). Check Validation panel.",
            )
            return {"CANCELLED"}


# Registration
classes = [
    LINKFORGE_OT_export_urdf,
    LINKFORGE_OT_import_urdf,
    LINKFORGE_OT_validate_robot,
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


if __name__ == "__main__":
    register()
