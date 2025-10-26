"""LinkForge - Professional URDF/XACRO Exporter for Blender.

Convert 3D robot models to valid URDF/XACRO files for ROS2 and Gazebo.

This is a Blender Extension compatible with Blender 4.2+.
Metadata is defined in blender_manifest.toml at the root of the extension.
"""

from __future__ import annotations

# Blender Extension Entry Point
# The manifest is loaded from blender_manifest.toml

# Note: bl_info is deprecated in Blender 4.2+ Extensions
# All metadata should be in blender_manifest.toml

# Package version and metadata
__version__ = "0.1.0"
__all__ = ["register", "unregister"]

# Import bpy and blender module at top level
# This prevents namespace package issues
try:
    import bpy

    from . import blender
except ImportError:
    # Allow import for testing without Blender
    bpy = None
    blender = None


def register() -> None:
    """Register the extension with Blender.

    This function is called when the extension is enabled.
    It registers all operators, panels, property groups, and other Blender types.
    """
    if bpy is None or blender is None:
        # Not running in Blender environment
        return

    # Register Blender components
    blender.register()

    print("LinkForge: Extension registered successfully")


def unregister() -> None:
    """Unregister the extension from Blender.

    This function is called when the extension is disabled.
    It unregisters all operators, panels, property groups, and other Blender types.
    """
    if bpy is None or blender is None:
        # Not running in Blender environment
        return

    # Unregister Blender components
    blender.unregister()

    print("LinkForge: Extension unregistered")


# Entry point for Blender Extension system
if __name__ == "__main__":
    register()
