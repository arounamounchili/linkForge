"""Blender UI Panels for LinkForge.

Panels provide the user interface in the 3D Viewport sidebar.
"""

from __future__ import annotations

from . import joint_panel, link_panel, robot_panel

# Module list for registration
modules = [
    link_panel,
    joint_panel,
    robot_panel,
]


def register():
    """Register all panels."""
    for module in modules:
        module.register()


def unregister():
    """Unregister all panels."""
    for module in reversed(modules):
        module.unregister()


__all__ = [
    "link_panel",
    "joint_panel",
    "robot_panel",
    "register",
    "unregister",
]
