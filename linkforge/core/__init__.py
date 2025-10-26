"""LinkForge Core Library.

This is the Blender-independent core library for robot URDF/XACRO generation.
It can be used standalone without Blender for testing and CLI tools.

Modules:
    models: Data structures for robots, links, joints, geometry
    physics: Inertia and mass calculations
    generators: URDF and XACRO file generation
    parsers: URDF and XACRO file parsing
"""

from __future__ import annotations

from . import generators, models, parsers, physics

__all__ = [
    "models",
    "physics",
    "generators",
    "parsers",
]
