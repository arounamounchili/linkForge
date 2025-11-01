"""Pytest configuration and fixtures for Blender layer tests."""

from __future__ import annotations

import sys
from typing import Any
from unittest.mock import MagicMock, Mock

import pytest


def pytest_configure(config):
    """Configure pytest with Blender mocks before any imports."""
    # Mock bpy_extras first with proper base classes
    mock_bpy_extras = Mock()
    mock_bpy_extras.io_utils = Mock()

    # Create unique base classes for ExportHelper and ImportHelper
    class ExportHelperBase:
        """Mock base class for ExportHelper."""

        pass

    class ImportHelperBase:
        """Mock base class for ImportHelper."""

        pass

    mock_bpy_extras.io_utils.ExportHelper = ExportHelperBase
    mock_bpy_extras.io_utils.ImportHelper = ImportHelperBase

    # Create mock bpy module
    mock_bpy_module = MagicMock()

    # Mock types
    class MockPanel:
        """Mock base class for Blender panels."""

        pass

    class MockOperator:
        """Mock base class for Blender operators."""

        pass

    class MockPropertyGroup:
        """Mock base class for Blender property groups."""

        pass

    mock_bpy_module.types = Mock()
    mock_bpy_module.types.Panel = MockPanel
    mock_bpy_module.types.Operator = MockOperator
    mock_bpy_module.types.PropertyGroup = MockPropertyGroup

    # Mock props
    mock_bpy_module.props = Mock()
    mock_bpy_module.props.BoolProperty = Mock(return_value=None)
    mock_bpy_module.props.StringProperty = Mock(return_value=None)
    mock_bpy_module.props.FloatProperty = Mock(return_value=None)
    mock_bpy_module.props.EnumProperty = Mock(return_value=None)
    mock_bpy_module.props.FloatVectorProperty = Mock(return_value=None)
    mock_bpy_module.props.PointerProperty = Mock(return_value=None)

    # Mock utils
    mock_bpy_module.utils = Mock()
    mock_bpy_module.utils.register_class = Mock()
    mock_bpy_module.utils.unregister_class = Mock()

    # Install mocks into sys.modules BEFORE any imports
    # This must happen before linkforge.blender is imported

    sys.modules["bpy"] = mock_bpy_module
    sys.modules["bpy.types"] = mock_bpy_module.types
    sys.modules["bpy.props"] = mock_bpy_module.props
    sys.modules["bpy.utils"] = mock_bpy_module.utils
    sys.modules["bpy_extras"] = mock_bpy_extras
    sys.modules["bpy_extras.io_utils"] = mock_bpy_extras.io_utils


class MockVector:
    """Mock for mathutils.Vector."""

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        """Initialize mock vector."""
        self.x = x
        self.y = y
        self.z = z

    def __getitem__(self, index: int) -> float:
        """Get component by index."""
        if index == 0:  # noqa: SIM116
            return self.x
        elif index == 1:
            return self.y
        elif index == 2:
            return self.z
        raise IndexError(f"Vector index out of range: {index}")

    def __len__(self) -> int:
        """Return vector length."""
        return 3


class MockEuler:
    """Mock for mathutils.Euler."""

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        """Initialize mock euler."""
        self.x = x
        self.y = y
        self.z = z


class MockMatrix:
    """Mock for mathutils.Matrix."""

    @staticmethod
    def Identity(size: int) -> MockMatrix:  # noqa: N802
        """Create identity matrix."""
        mat = MockMatrix()
        mat._is_identity = True
        return mat

    def __init__(self):
        """Initialize mock matrix."""
        self._translation = MockVector(0.0, 0.0, 0.0)
        self._rotation = MockEuler(0.0, 0.0, 0.0)
        self._is_identity = False

    def to_translation(self) -> MockVector:
        """Return translation component."""
        return self._translation

    def to_euler(self, order: str = "XYZ") -> MockEuler:
        """Return rotation as Euler angles."""
        return self._rotation

    def inverted(self) -> MockMatrix:
        """Return inverted matrix."""
        # Simplified: just return self for testing
        return self

    def __matmul__(self, other: Any) -> MockMatrix:
        """Matrix multiplication."""
        # Simplified: return other for testing
        if isinstance(other, MockMatrix):
            return other
        return self


class MockObject:
    """Mock for bpy.types.Object."""

    def __init__(self, name: str = "MockObject", obj_type: str = "MESH"):
        """Initialize mock object."""
        self.name = name
        self.type = obj_type
        self.dimensions = MockVector(1.0, 1.0, 1.0)
        self.matrix_world = MockMatrix()
        self.material_slots = []
        self.data = Mock()

        # LinkForge properties
        self.linkforge = Mock()
        self.linkforge.is_robot_link = False
        self.linkforge.link_name = ""
        self.linkforge.mass = 1.0
        self.linkforge.use_auto_inertia = True
        self.linkforge.use_visual_geometry = True
        self.linkforge.visual_geometry_type = "MESH"
        self.linkforge.export_collision = True
        self.linkforge.collision_geometry_type = "BOX"
        self.linkforge.simplify_collision = False
        self.linkforge.collision_decimation_ratio = 0.5
        self.linkforge.use_material = False
        self.linkforge.material_source = "BLENDER"
        self.linkforge.material_name = ""
        self.linkforge.material_color = [0.8, 0.8, 0.8, 1.0]
        self.linkforge.inertia_ixx = 0.1
        self.linkforge.inertia_ixy = 0.0
        self.linkforge.inertia_ixz = 0.0
        self.linkforge.inertia_iyy = 0.1
        self.linkforge.inertia_iyz = 0.0
        self.linkforge.inertia_izz = 0.1

        # Joint properties
        self.linkforge_joint = Mock()
        self.linkforge_joint.is_robot_joint = False
        self.linkforge_joint.joint_name = ""
        self.linkforge_joint.joint_type = "REVOLUTE"
        self.linkforge_joint.parent_link = "NONE"
        self.linkforge_joint.child_link = "NONE"
        self.linkforge_joint.axis = "Z"
        self.linkforge_joint.custom_axis_x = 0.0
        self.linkforge_joint.custom_axis_y = 0.0
        self.linkforge_joint.custom_axis_z = 1.0
        self.linkforge_joint.use_limits = False
        self.linkforge_joint.limit_lower = 0.0
        self.linkforge_joint.limit_upper = 0.0
        self.linkforge_joint.limit_effort = 0.0
        self.linkforge_joint.limit_velocity = 0.0
        self.linkforge_joint.use_dynamics = False
        self.linkforge_joint.dynamics_damping = 0.0
        self.linkforge_joint.dynamics_friction = 0.0
        self.linkforge_joint.use_mimic = False
        self.linkforge_joint.mimic_joint = ""
        self.linkforge_joint.mimic_multiplier = 1.0
        self.linkforge_joint.mimic_offset = 0.0

    def select_get(self) -> bool:
        """Return selection state."""
        return True


@pytest.fixture
def mock_bpy():
    """Provide bpy mock to tests."""
    import bpy  # This will be our mocked module

    # Add context if not present
    if not hasattr(bpy, "context") or bpy.context is None:
        mock_context = Mock()
        mock_context.scene = Mock()
        mock_context.scene.linkforge = Mock()
        mock_context.scene.linkforge.robot_name = "test_robot"
        mock_context.scene.linkforge.mesh_format = "STL"
        mock_context.scene.objects = []
        mock_context.active_object = None
        bpy.context = mock_context

    return bpy


@pytest.fixture(autouse=True)
def mock_mathutils(monkeypatch):
    """Mock the mathutils module for testing (automatically used for all tests)."""
    mock_mathutils_module = Mock()
    mock_mathutils_module.Vector = MockVector
    mock_mathutils_module.Matrix = MockMatrix
    mock_mathutils_module.Euler = MockEuler

    # Monkeypatch mathutils into sys.modules
    monkeypatch.setitem(__import__("sys").modules, "mathutils", mock_mathutils_module)

    return mock_mathutils_module


@pytest.fixture
def mock_blender_object() -> MockObject:
    """Create a mock Blender object for testing."""
    return MockObject()


@pytest.fixture
def mock_link_object() -> MockObject:
    """Create a mock Blender object configured as a robot link."""
    obj = MockObject(name="test_link", obj_type="MESH")
    obj.linkforge.is_robot_link = True
    obj.linkforge.link_name = "test_link"
    obj.linkforge.mass = 5.0
    obj.linkforge.use_auto_inertia = True
    obj.linkforge.use_visual_geometry = True
    obj.linkforge.visual_geometry_type = "BOX"
    obj.linkforge.export_collision = True
    obj.linkforge.collision_geometry_type = "BOX"
    obj.dimensions = MockVector(2.0, 3.0, 4.0)
    return obj


@pytest.fixture
def mock_joint_object() -> MockObject:
    """Create a mock Blender empty configured as a robot joint."""
    obj = MockObject(name="test_joint", obj_type="EMPTY")
    obj.linkforge_joint.is_robot_joint = True
    obj.linkforge_joint.joint_name = "test_joint"
    obj.linkforge_joint.joint_type = "REVOLUTE"
    obj.linkforge_joint.parent_link = "base_link"
    obj.linkforge_joint.child_link = "test_link"
    obj.linkforge_joint.axis = "Z"
    obj.linkforge_joint.use_limits = True
    obj.linkforge_joint.limit_lower = -1.57
    obj.linkforge_joint.limit_upper = 1.57
    obj.linkforge_joint.limit_effort = 10.0
    obj.linkforge_joint.limit_velocity = 1.0
    return obj
