"""Blender integration layer for LinkForge.

This module contains all Blender-specific code:
- Property Groups: Store data on Blender objects
- Operators: User actions/commands
- Panels: User interface
- Handlers: Event handlers for scene updates
"""

from __future__ import annotations

from . import handlers, operators, panels, properties

# Registration order matters: properties first, then operators, then panels, then handlers
modules = [
    properties,
    operators,
    panels,
    handlers,
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
    "operators",
    "panels",
    "handlers",
    "register",
    "unregister",
]
