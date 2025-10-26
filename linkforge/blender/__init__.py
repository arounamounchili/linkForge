"""Blender integration layer for LinkForge.

This module contains all Blender-specific code:
- Property Groups: Store data on Blender objects
- Operators: User actions/commands
- Panels: User interface
- Gizmos: Visual helpers (future)
"""

from __future__ import annotations

from . import operators, panels, properties

# Registration order matters: properties first, then operators, then panels
modules = [
    properties,
    operators,
    panels,
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
    "register",
    "unregister",
]
