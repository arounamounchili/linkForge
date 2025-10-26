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
        from ..utils.converters import scene_to_robot
        from ...core.generators import URDFGenerator, XACROGenerator

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
            errors = robot.validate_tree_structure()
            if errors:
                error_msg = "Validation errors:\\n" + "\\n".join(errors[:5])
                if len(errors) > 5:
                    error_msg += f"\\n... and {len(errors) - 5} more errors"
                self.report({"ERROR"}, error_msg)
                return {"CANCELLED"}

        # Generate URDF/XACRO
        try:
            if robot_props.export_format == "URDF":
                generator = URDFGenerator(pretty_print=True, urdf_path=output_path)
                generator.write(robot, output_path)
                self.report({"INFO"}, f"Exported URDF to {output_path}")
            else:  # XACRO
                generator = XACROGenerator(pretty_print=True)
                generator.write(robot, output_path)
                self.report({"INFO"}, f"Exported XACRO to {output_path}")

            return {"FINISHED"}

        except Exception as e:
            self.report({"ERROR"}, f"Export failed: {e}")
            return {"CANCELLED"}


class LINKFORGE_OT_import_urdf(Operator, ImportHelper):
    """Import robot from URDF file"""

    bl_idname = "linkforge.import_urdf"
    bl_label = "Import URDF"
    bl_description = "Import robot from URDF/XACRO file"

    # ImportHelper properties
    filename_ext = ".urdf"
    filter_glob: StringProperty(  # type: ignore
        default="*.urdf;*.xacro",
        options={"HIDDEN"},
    )

    def execute(self, context):
        """Execute the import."""
        from pathlib import Path
        from ..utils.urdf_importer import import_robot_to_scene
        from ...core.parsers.urdf_parser import parse_urdf

        # Parse URDF file
        urdf_path = Path(self.filepath)

        try:
            robot = parse_urdf(urdf_path)
        except FileNotFoundError:
            self.report({"ERROR"}, f"URDF file not found: {urdf_path}")
            return {"CANCELLED"}
        except Exception as e:
            self.report({"ERROR"}, f"Failed to parse URDF: {e}")
            return {"CANCELLED"}

        # Import to scene
        try:
            success = import_robot_to_scene(robot, urdf_path, context)
            if success:
                self.report(
                    {"INFO"},
                    f"Imported robot '{robot.name}' "
                    f"({len(robot.links)} links, {len(robot.joints)} joints)"
                )
                return {"FINISHED"}
            else:
                self.report({"ERROR"}, "Failed to import robot to scene")
                return {"CANCELLED"}

        except Exception as e:
            self.report({"ERROR"}, f"Import failed: {e}")
            return {"CANCELLED"}


class LINKFORGE_OT_validate_robot(Operator):
    """Validate robot structure"""

    bl_idname = "linkforge.validate_robot"
    bl_label = "Validate Robot"
    bl_description = "Validate the robot structure for errors"

    def execute(self, context):
        """Execute validation."""
        from ..utils.converters import scene_to_robot

        # Convert scene to robot
        try:
            robot = scene_to_robot(context)
        except Exception as e:
            self.report({"ERROR"}, f"Failed to build robot: {e}")
            return {"CANCELLED"}

        # Validate
        errors = robot.validate_tree_structure()

        if not errors:
            self.report(
                {"INFO"},
                f"✓ Robot '{robot.name}' is valid! "
                f"({len(robot.links)} links, {len(robot.joints)} joints, "
                f"{robot.degrees_of_freedom} DOF)"
            )
            return {"FINISHED"}
        else:
            # Report first few errors
            error_summary = "\\n".join(errors[:5])
            if len(errors) > 5:
                error_summary += f"\\n... and {len(errors) - 5} more errors"

            self.report({"WARNING"}, f"Validation errors:\\n{error_summary}")

            # Print all errors to console
            print("\\n=== LinkForge Validation Errors ===")
            for i, error in enumerate(errors, 1):
                print(f"{i}. {error}")
            print("===================================\\n")

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
