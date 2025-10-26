"""Blender Operators for LinkForge.

Operators are user actions/commands in Blender.
"""

from __future__ import annotations

from . import export_ops, joint_ops, link_ops

# Module list for registration
modules = [
    link_ops,
    joint_ops,
    export_ops,
]


def register():
    """Register all operators."""
    for module in modules:
        module.register()


def unregister():
    """Unregister all operators."""
    for module in reversed(modules):
        module.unregister()


__all__ = [
    "link_ops",
    "joint_ops",
    "export_ops",
    "register",
    "unregister",
]
