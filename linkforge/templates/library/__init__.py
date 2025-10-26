"""Robot template library.

This module contains all built-in robot templates and registers them
on import.
"""

from __future__ import annotations

from ..loader import register_template
from .arm_2dof import ARM_2DOF_TEMPLATE
from .gripper_parallel import GRIPPER_PARALLEL_TEMPLATE
from .mobile_4wheel import MOBILE_4WHEEL_TEMPLATE

# Register all templates
register_template(ARM_2DOF_TEMPLATE)
register_template(MOBILE_4WHEEL_TEMPLATE)
register_template(GRIPPER_PARALLEL_TEMPLATE)

__all__ = [
    "ARM_2DOF_TEMPLATE",
    "MOBILE_4WHEEL_TEMPLATE",
    "GRIPPER_PARALLEL_TEMPLATE",
]
