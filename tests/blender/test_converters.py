"""Tests for Blender to Core model converters."""

from __future__ import annotations

import pytest

from linkforge.core.models import Box, Cylinder, Sphere, Transform, Vector3


class TestBlenderToVector3:
    """Tests for blender_to_vector3 converter."""

    def test_converts_mock_vector(self, mock_bpy, mock_mathutils):
        """Test conversion of mock Blender vector to Vector3."""
        from linkforge.blender.utils.converters import blender_to_vector3
        from tests.blender.conftest import MockVector

        vec = MockVector(1.5, 2.5, 3.5)
        result = blender_to_vector3(vec)

        assert isinstance(result, Vector3)
        assert result.x == 1.5
        assert result.y == 2.5
        assert result.z == 3.5

    def test_zero_vector(self, mock_bpy, mock_mathutils):
        """Test conversion of zero vector."""
        from linkforge.blender.utils.converters import blender_to_vector3
        from tests.blender.conftest import MockVector

        vec = MockVector(0.0, 0.0, 0.0)
        result = blender_to_vector3(vec)

        assert result.x == 0.0
        assert result.y == 0.0
        assert result.z == 0.0

    def test_negative_values(self, mock_bpy, mock_mathutils):
        """Test conversion with negative values."""
        from linkforge.blender.utils.converters import blender_to_vector3
        from tests.blender.conftest import MockVector

        vec = MockVector(-1.0, -2.0, -3.0)
        result = blender_to_vector3(vec)

        assert result.x == -1.0
        assert result.y == -2.0
        assert result.z == -3.0


class TestCleanFloat:
    """Tests for clean_float helper function."""

    def test_cleans_small_values(self, mock_bpy, mock_mathutils):
        """Test that very small values are cleaned to zero."""
        from linkforge.blender.utils.converters import clean_float

        assert clean_float(1e-15) == 0.0
        assert clean_float(-1e-15) == 0.0
        assert clean_float(1e-11) == 0.0

    def test_preserves_normal_values(self, mock_bpy, mock_mathutils):
        """Test that normal values are preserved."""
        from linkforge.blender.utils.converters import clean_float

        assert clean_float(1.5) == 1.5
        assert clean_float(-2.5) == -2.5
        assert clean_float(0.001) == 0.001

    def test_cleans_negative_zero(self, mock_bpy, mock_mathutils):
        """Test that -0.0 is cleaned to 0.0."""
        from linkforge.blender.utils.converters import clean_float

        result = clean_float(-0.0)
        assert result == 0.0


class TestMatrixToTransform:
    """Tests for matrix_to_transform converter."""

    def test_identity_matrix(self, mock_bpy, mock_mathutils):
        """Test conversion of identity matrix."""
        from linkforge.blender.utils.converters import matrix_to_transform
        from tests.blender.conftest import MockMatrix

        mat = MockMatrix.Identity(4)
        result = matrix_to_transform(mat)

        assert isinstance(result, Transform)
        assert result.xyz.x == 0.0
        assert result.xyz.y == 0.0
        assert result.xyz.z == 0.0
        assert result.rpy.x == 0.0
        assert result.rpy.y == 0.0
        assert result.rpy.z == 0.0

    def test_translation_only(self, mock_bpy, mock_mathutils):
        """Test matrix with translation only."""
        from linkforge.blender.utils.converters import matrix_to_transform
        from tests.blender.conftest import MockEuler, MockMatrix, MockVector

        mat = MockMatrix()
        mat._translation = MockVector(1.0, 2.0, 3.0)
        mat._rotation = MockEuler(0.0, 0.0, 0.0)

        result = matrix_to_transform(mat)

        assert result.xyz.x == 1.0
        assert result.xyz.y == 2.0
        assert result.xyz.z == 3.0
        assert result.rpy.x == 0.0
        assert result.rpy.y == 0.0
        assert result.rpy.z == 0.0

    def test_rotation_only(self, mock_bpy, mock_mathutils):
        """Test matrix with rotation only."""
        from linkforge.blender.utils.converters import matrix_to_transform
        from tests.blender.conftest import MockEuler, MockMatrix, MockVector

        mat = MockMatrix()
        mat._translation = MockVector(0.0, 0.0, 0.0)
        mat._rotation = MockEuler(1.57, 0.0, 0.0)  # 90 degrees in radians

        result = matrix_to_transform(mat)

        assert result.xyz.x == 0.0
        assert result.xyz.y == 0.0
        assert result.xyz.z == 0.0
        assert result.rpy.x == pytest.approx(1.57)
        assert result.rpy.y == 0.0
        assert result.rpy.z == 0.0

    def test_combined_transform(self, mock_bpy, mock_mathutils):
        """Test matrix with both translation and rotation."""
        from linkforge.blender.utils.converters import matrix_to_transform
        from tests.blender.conftest import MockEuler, MockMatrix, MockVector

        mat = MockMatrix()
        mat._translation = MockVector(1.0, 2.0, 3.0)
        mat._rotation = MockEuler(0.5, 1.0, 1.5)

        result = matrix_to_transform(mat)

        assert result.xyz.x == 1.0
        assert result.xyz.y == 2.0
        assert result.xyz.z == 3.0
        assert result.rpy.x == pytest.approx(0.5)
        assert result.rpy.y == pytest.approx(1.0)
        assert result.rpy.z == pytest.approx(1.5)

    def test_none_matrix(self, mock_bpy, mock_mathutils):
        """Test handling of None matrix."""
        from linkforge.blender.utils.converters import matrix_to_transform

        result = matrix_to_transform(None)

        assert result == Transform.identity()


class TestGetObjectGeometry:
    """Tests for get_object_geometry function."""

    def test_box_geometry(self, mock_bpy, mock_mathutils, mock_blender_object):
        """Test extraction of box geometry."""
        from linkforge.blender.utils.converters import get_object_geometry
        from tests.blender.conftest import MockVector

        mock_blender_object.dimensions = MockVector(2.0, 3.0, 4.0)

        result = get_object_geometry(mock_blender_object, geometry_type="BOX")

        assert isinstance(result, Box)
        assert result.size.x == 2.0
        assert result.size.y == 3.0
        assert result.size.z == 4.0

    def test_cylinder_geometry(self, mock_bpy, mock_mathutils, mock_blender_object):
        """Test extraction of cylinder geometry."""
        from linkforge.blender.utils.converters import get_object_geometry
        from tests.blender.conftest import MockVector

        mock_blender_object.dimensions = MockVector(2.0, 2.0, 5.0)

        result = get_object_geometry(mock_blender_object, geometry_type="CYLINDER")

        assert isinstance(result, Cylinder)
        assert result.radius == 1.0  # max(2.0, 2.0) / 2.0
        assert result.length == 5.0

    def test_sphere_geometry(self, mock_bpy, mock_mathutils, mock_blender_object):
        """Test extraction of sphere geometry."""
        from linkforge.blender.utils.converters import get_object_geometry
        from tests.blender.conftest import MockVector

        mock_blender_object.dimensions = MockVector(4.0, 4.0, 4.0)

        result = get_object_geometry(mock_blender_object, geometry_type="SPHERE")

        assert isinstance(result, Sphere)
        assert result.radius == 2.0  # max(4.0, 4.0, 4.0) / 2.0

    def test_none_object(self, mock_bpy, mock_mathutils):
        """Test handling of None object."""
        from linkforge.blender.utils.converters import get_object_geometry

        result = get_object_geometry(None, geometry_type="BOX")

        assert result is None


class TestGetObjectMaterial:
    """Tests for get_object_material function."""

    def test_no_material_used(self, mock_bpy, mock_mathutils, mock_blender_object):
        """Test when material is not used."""
        from linkforge.blender.utils.converters import get_object_material

        mock_blender_object.linkforge.use_material = False

        result = get_object_material(mock_blender_object, mock_blender_object.linkforge)

        assert result is None

    def test_custom_material(self, mock_bpy, mock_mathutils, mock_blender_object):
        """Test custom material color."""
        from linkforge.blender.utils.converters import get_object_material

        mock_blender_object.linkforge.use_material = True
        mock_blender_object.linkforge.material_source = "CUSTOM"
        mock_blender_object.linkforge.material_color = [0.5, 0.6, 0.7, 0.8]
        mock_blender_object.linkforge.material_name = "custom_mat"

        result = get_object_material(mock_blender_object, mock_blender_object.linkforge)

        assert result is not None
        assert result.name == "custom_mat"
        assert result.color.r == 0.5
        assert result.color.g == 0.6
        assert result.color.b == 0.7
        assert result.color.a == 0.8


class TestBlenderJointToCore:
    """Tests for blender_joint_to_core converter."""

    def test_basic_revolute_joint(self, mock_bpy, mock_mathutils, mock_joint_object):
        """Test conversion of basic revolute joint."""
        from linkforge.blender.utils.converters import blender_joint_to_core

        mock_scene = mock_bpy.context.scene
        result = blender_joint_to_core(mock_joint_object, mock_scene)

        assert result is not None
        assert result.name == "test_joint"
        assert result.type.value == "revolute"
        assert result.parent == "base_link"
        assert result.child == "test_link"

    def test_joint_with_z_axis(self, mock_bpy, mock_mathutils, mock_joint_object):
        """Test joint with Z axis."""
        from linkforge.blender.utils.converters import blender_joint_to_core

        mock_joint_object.linkforge_joint.axis = "Z"
        mock_scene = mock_bpy.context.scene

        result = blender_joint_to_core(mock_joint_object, mock_scene)

        assert result.axis.x == 0.0
        assert result.axis.y == 0.0
        assert result.axis.z == 1.0

    def test_joint_with_x_axis(self, mock_bpy, mock_mathutils, mock_joint_object):
        """Test joint with X axis."""
        from linkforge.blender.utils.converters import blender_joint_to_core

        mock_joint_object.linkforge_joint.axis = "X"
        mock_scene = mock_bpy.context.scene

        result = blender_joint_to_core(mock_joint_object, mock_scene)

        assert result.axis.x == 1.0
        assert result.axis.y == 0.0
        assert result.axis.z == 0.0

    def test_joint_with_y_axis(self, mock_bpy, mock_mathutils, mock_joint_object):
        """Test joint with Y axis."""
        from linkforge.blender.utils.converters import blender_joint_to_core

        mock_joint_object.linkforge_joint.axis = "Y"
        mock_scene = mock_bpy.context.scene

        result = blender_joint_to_core(mock_joint_object, mock_scene)

        assert result.axis.x == 0.0
        assert result.axis.y == 1.0
        assert result.axis.z == 0.0

    def test_joint_with_custom_axis(self, mock_bpy, mock_mathutils, mock_joint_object):
        """Test joint with custom axis."""
        from linkforge.blender.utils.converters import blender_joint_to_core

        mock_joint_object.linkforge_joint.axis = "CUSTOM"
        mock_joint_object.linkforge_joint.custom_axis_x = 0.577
        mock_joint_object.linkforge_joint.custom_axis_y = 0.577
        mock_joint_object.linkforge_joint.custom_axis_z = 0.577
        mock_scene = mock_bpy.context.scene

        result = blender_joint_to_core(mock_joint_object, mock_scene)

        assert result.axis.x == pytest.approx(0.577)
        assert result.axis.y == pytest.approx(0.577)
        assert result.axis.z == pytest.approx(0.577)

    def test_joint_with_limits(self, mock_bpy, mock_mathutils, mock_joint_object):
        """Test joint with limits."""
        from linkforge.blender.utils.converters import blender_joint_to_core

        mock_joint_object.linkforge_joint.use_limits = True
        mock_joint_object.linkforge_joint.limit_lower = -1.57
        mock_joint_object.linkforge_joint.limit_upper = 1.57
        mock_joint_object.linkforge_joint.limit_effort = 10.0
        mock_joint_object.linkforge_joint.limit_velocity = 2.0
        mock_scene = mock_bpy.context.scene

        result = blender_joint_to_core(mock_joint_object, mock_scene)

        assert result.limits is not None
        assert result.limits.lower == -1.57
        assert result.limits.upper == 1.57
        assert result.limits.effort == 10.0
        assert result.limits.velocity == 2.0

    def test_joint_with_dynamics(self, mock_bpy, mock_mathutils, mock_joint_object):
        """Test joint with dynamics."""
        from linkforge.blender.utils.converters import blender_joint_to_core

        mock_joint_object.linkforge_joint.use_dynamics = True
        mock_joint_object.linkforge_joint.dynamics_damping = 0.5
        mock_joint_object.linkforge_joint.dynamics_friction = 0.1
        mock_scene = mock_bpy.context.scene

        result = blender_joint_to_core(mock_joint_object, mock_scene)

        assert result.dynamics is not None
        assert result.dynamics.damping == 0.5
        assert result.dynamics.friction == 0.1

    def test_joint_with_mimic(self, mock_bpy, mock_mathutils, mock_joint_object):
        """Test joint with mimic."""
        from linkforge.blender.utils.converters import blender_joint_to_core

        mock_joint_object.linkforge_joint.use_mimic = True
        mock_joint_object.linkforge_joint.mimic_joint = "other_joint"
        mock_joint_object.linkforge_joint.mimic_multiplier = 2.0
        mock_joint_object.linkforge_joint.mimic_offset = 0.5
        mock_scene = mock_bpy.context.scene

        result = blender_joint_to_core(mock_joint_object, mock_scene)

        assert result.mimic is not None
        assert result.mimic.joint == "other_joint"
        assert result.mimic.multiplier == 2.0
        assert result.mimic.offset == 0.5

    def test_non_joint_object(self, mock_bpy, mock_mathutils, mock_blender_object):
        """Test handling of non-joint object."""
        from linkforge.blender.utils.converters import blender_joint_to_core

        mock_blender_object.linkforge_joint.is_robot_joint = False
        mock_scene = mock_bpy.context.scene

        result = blender_joint_to_core(mock_blender_object, mock_scene)

        assert result is None

    def test_none_object(self, mock_bpy, mock_mathutils):
        """Test handling of None object."""
        from linkforge.blender.utils.converters import blender_joint_to_core

        mock_scene = mock_bpy.context.scene
        result = blender_joint_to_core(None, mock_scene)

        assert result is None

    def test_handles_none_parent_child(self, mock_bpy, mock_mathutils, mock_joint_object):
        """Test handling of NONE parent/child links."""
        import pytest

        from linkforge.blender.utils.converters import blender_joint_to_core

        mock_joint_object.linkforge_joint.parent_link = "NONE"
        mock_joint_object.linkforge_joint.child_link = "NONE"
        mock_scene = mock_bpy.context.scene

        # Joint validation should reject joints with empty parent/child
        # This is correct behavior - joints must connect two links
        with pytest.raises(ValueError, match="Parent link name cannot be empty"):
            blender_joint_to_core(mock_joint_object, mock_scene)


class TestBlenderLinkToCore:
    """Tests for blender_link_to_core_with_origin converter."""

    def test_basic_link_conversion(self, mock_bpy, mock_mathutils, mock_link_object):
        """Test conversion of basic link."""
        from linkforge.blender.utils.converters import blender_link_to_core_with_origin

        visual_origin = Transform.identity()
        result = blender_link_to_core_with_origin(mock_link_object, visual_origin)

        assert result is not None
        assert result.name == "test_link"
        assert result.inertial is not None
        assert result.inertial.mass == 5.0

    def test_link_with_auto_inertia(self, mock_bpy, mock_mathutils, mock_link_object):
        """Test link with auto-calculated inertia."""
        from linkforge.blender.utils.converters import blender_link_to_core_with_origin

        mock_link_object.linkforge.use_auto_inertia = True
        mock_link_object.linkforge.mass = 10.0
        visual_origin = Transform.identity()

        result = blender_link_to_core_with_origin(mock_link_object, visual_origin)

        assert result.inertial is not None
        assert result.inertial.mass == 10.0
        assert result.inertial.inertia.ixx > 0
        assert result.inertial.inertia.iyy > 0
        assert result.inertial.inertia.izz > 0

    def test_link_with_manual_inertia(self, mock_bpy, mock_mathutils, mock_link_object):
        """Test link with manual inertia values."""
        from linkforge.blender.utils.converters import blender_link_to_core_with_origin

        mock_link_object.linkforge.use_auto_inertia = False
        mock_link_object.linkforge.mass = 5.0
        mock_link_object.linkforge.inertia_ixx = 0.5
        mock_link_object.linkforge.inertia_iyy = 0.6
        mock_link_object.linkforge.inertia_izz = 0.7
        mock_link_object.linkforge.inertia_ixy = 0.1
        visual_origin = Transform.identity()

        result = blender_link_to_core_with_origin(mock_link_object, visual_origin)

        assert result.inertial is not None
        assert result.inertial.inertia.ixx == 0.5
        assert result.inertial.inertia.iyy == 0.6
        assert result.inertial.inertia.izz == 0.7
        assert result.inertial.inertia.ixy == 0.1

    def test_link_with_visual_geometry(self, mock_bpy, mock_mathutils, mock_link_object):
        """Test link with visual geometry."""
        from linkforge.blender.utils.converters import blender_link_to_core_with_origin

        mock_link_object.linkforge.use_visual_geometry = True
        mock_link_object.linkforge.visual_geometry_type = "BOX"
        visual_origin = Transform.identity()

        result = blender_link_to_core_with_origin(mock_link_object, visual_origin)

        assert result.visual is not None
        assert isinstance(result.visual.geometry, Box)

    def test_link_with_collision(self, mock_bpy, mock_mathutils, mock_link_object):
        """Test link with collision geometry."""
        from linkforge.blender.utils.converters import blender_link_to_core_with_origin

        mock_link_object.linkforge.export_collision = True
        mock_link_object.linkforge.collision_geometry_type = "CYLINDER"
        visual_origin = Transform.identity()

        result = blender_link_to_core_with_origin(mock_link_object, visual_origin)

        assert result.collision is not None
        assert isinstance(result.collision.geometry, Cylinder)

    def test_link_without_visual(self, mock_bpy, mock_mathutils, mock_link_object):
        """Test link without visual geometry."""
        from linkforge.blender.utils.converters import blender_link_to_core_with_origin

        mock_link_object.linkforge.use_visual_geometry = False
        visual_origin = Transform.identity()

        result = blender_link_to_core_with_origin(mock_link_object, visual_origin)

        assert result.visual is None

    def test_link_without_collision(self, mock_bpy, mock_mathutils, mock_link_object):
        """Test link without collision geometry."""
        from linkforge.blender.utils.converters import blender_link_to_core_with_origin

        mock_link_object.linkforge.export_collision = False
        visual_origin = Transform.identity()

        result = blender_link_to_core_with_origin(mock_link_object, visual_origin)

        assert result.collision is None

    def test_link_with_zero_mass(self, mock_bpy, mock_mathutils, mock_link_object):
        """Test link with zero mass has no inertial."""
        from linkforge.blender.utils.converters import blender_link_to_core_with_origin

        mock_link_object.linkforge.mass = 0.0
        visual_origin = Transform.identity()

        result = blender_link_to_core_with_origin(mock_link_object, visual_origin)

        assert result.inertial is None

    def test_non_link_object(self, mock_bpy, mock_mathutils, mock_blender_object):
        """Test handling of non-link object."""
        from linkforge.blender.utils.converters import blender_link_to_core_with_origin

        mock_blender_object.linkforge.is_robot_link = False
        visual_origin = Transform.identity()

        result = blender_link_to_core_with_origin(mock_blender_object, visual_origin)

        assert result is None

    def test_none_object(self, mock_bpy, mock_mathutils):
        """Test handling of None object."""
        from linkforge.blender.utils.converters import blender_link_to_core_with_origin

        visual_origin = Transform.identity()
        result = blender_link_to_core_with_origin(None, visual_origin)

        assert result is None
