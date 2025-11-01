"""Blender integration layer for LinkForge.

This module contains all Blender-specific code:
- Property Groups: Store data on Blender objects
- Operators: User actions/commands
- Panels: User interface
- Handlers: Event handlers for scene updates
"""

from __future__ import annotations

from . import handlers, operators, panels, preferences, properties
from .utils import joint_gizmos

# Registration order matters: properties first, then operators, then panels, then handlers, then gizmos
modules = [
    properties,
    preferences,
    operators,
    panels,
    handlers,
    joint_gizmos,
]


def register():
    """Register all Blender components."""
    for module in modules:
        module.register()


def unregister():
    """Unregister all Blender components."""
    for module in reversed(modules):
        module.unregister()


__all__ = [
    "properties",
    "preferences",
    "operators",
    "panels",
    "handlers",
    "joint_gizmos",
    "register",
    "unregister",
]
