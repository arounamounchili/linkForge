"""Robot template data structure for LinkForge.

This module defines the RobotTemplate class used to represent
pre-built robot configurations that users can instantiate.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from ..core.models import Robot


@dataclass(frozen=True)
class RobotTemplate:
    """A pre-configured robot template.

    Templates provide ready-to-use robot configurations that users
    can instantiate with a single click, serving as starting points
    for new projects or learning examples.

    Attributes:
        id: Unique identifier for the template (e.g., "arm_2dof")
        name: Human-readable name (e.g., "2-DOF Robot Arm")
        description: Brief description of the template
        category: Template category (e.g., "arm", "mobile", "gripper")
        factory: Callable that creates the Robot instance
        thumbnail_path: Optional path to preview image
        tags: Optional list of tags for searching/filtering
    """

    id: str
    name: str
    description: str
    category: str
    factory: Callable[[], Robot]
    thumbnail_path: str | None = None
    tags: list[str] | None = None

    def create_robot(self) -> Robot:
        """Create a Robot instance from this template.

        Returns:
            A new Robot instance with the template configuration
        """
        return self.factory()


# Template categories
class TemplateCategory:
    """Standard template categories."""

    ARM = "arm"
    MOBILE = "mobile"
    GRIPPER = "gripper"
    FULL_ROBOT = "full"
    SENSOR = "sensor"
    OTHER = "other"
