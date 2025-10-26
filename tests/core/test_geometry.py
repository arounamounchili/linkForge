"""Tests for geometry primitives."""

from __future__ import annotations

import math

import pytest

from linkforge.core.models import Box, Capsule, Cylinder, Sphere, Transform, Vector3


class TestVector3:
    """Tests for Vector3."""

    def test_creation(self):
        """Test creating a vector."""
        v = Vector3(1.0, 2.0, 3.0)
        assert v.x == 1.0
        assert v.y == 2.0
        assert v.z == 3.0

    def test_to_tuple(self):
        """Test converting to tuple."""
        v = Vector3(1.0, 2.0, 3.0)
        assert v.to_tuple() == (1.0, 2.0, 3.0)

    def test_iteration(self):
        """Test unpacking."""
        v = Vector3(1.0, 2.0, 3.0)
        x, y, z = v
        assert x == 1.0
        assert y == 2.0
        assert z == 3.0

    def test_string_representation(self):
        """Test string conversion for URDF."""
        v = Vector3(1.0, 2.0, 3.0)
        assert str(v) == "1.0 2.0 3.0"


class TestTransform:
    """Tests for Transform."""

    def test_identity(self):
        """Test identity transform."""
        t = Transform.identity()
        assert t.xyz == Vector3(0.0, 0.0, 0.0)
        assert t.rpy == Vector3(0.0, 0.0, 0.0)

    def test_creation(self):
        """Test creating a transform."""
        t = Transform(xyz=Vector3(1.0, 2.0, 3.0), rpy=Vector3(0.1, 0.2, 0.3))
        assert t.xyz.x == 1.0
        assert t.rpy.x == 0.1

    def test_string_representation(self):
        """Test string representation."""
        t = Transform(xyz=Vector3(1.0, 2.0, 3.0), rpy=Vector3(0.1, 0.2, 0.3))
        assert "xyz" in str(t)
        assert "rpy" in str(t)


class TestBox:
    """Tests for Box geometry."""

    def test_creation(self):
        """Test creating a box."""
        box = Box(size=Vector3(1.0, 2.0, 3.0))
        assert box.size.x == 1.0
        assert box.size.y == 2.0
        assert box.size.z == 3.0

    def test_volume(self):
        """Test volume calculation."""
        box = Box(size=Vector3(2.0, 3.0, 4.0))
        assert box.volume() == 24.0

    def test_type(self):
        """Test geometry type."""
        from linkforge.core.models import GeometryType

        box = Box(size=Vector3(1.0, 1.0, 1.0))
        assert box.type == GeometryType.BOX


class TestCylinder:
    """Tests for Cylinder geometry."""

    def test_creation(self):
        """Test creating a cylinder."""
        cyl = Cylinder(radius=1.0, length=2.0)
        assert cyl.radius == 1.0
        assert cyl.length == 2.0

    def test_volume(self):
        """Test volume calculation."""
        cyl = Cylinder(radius=1.0, length=2.0)
        expected = math.pi * 1.0**2 * 2.0
        assert cyl.volume() == pytest.approx(expected)

    def test_type(self):
        """Test geometry type."""
        from linkforge.core.models import GeometryType

        cyl = Cylinder(radius=1.0, length=2.0)
        assert cyl.type == GeometryType.CYLINDER


class TestSphere:
    """Tests for Sphere geometry."""

    def test_creation(self):
        """Test creating a sphere."""
        sphere = Sphere(radius=1.0)
        assert sphere.radius == 1.0

    def test_volume(self):
        """Test volume calculation."""
        sphere = Sphere(radius=1.0)
        expected = (4.0 / 3.0) * math.pi * 1.0**3
        assert sphere.volume() == pytest.approx(expected)

    def test_type(self):
        """Test geometry type."""
        from linkforge.core.models import GeometryType

        sphere = Sphere(radius=1.0)
        assert sphere.type == GeometryType.SPHERE


class TestCapsule:
    """Tests for Capsule geometry."""

    def test_creation(self):
        """Test creating a capsule."""
        capsule = Capsule(radius=1.0, length=2.0)
        assert capsule.radius == 1.0
        assert capsule.length == 2.0

    def test_volume(self):
        """Test volume calculation."""
        capsule = Capsule(radius=1.0, length=2.0)
        cylinder_vol = math.pi * 1.0**2 * 2.0
        sphere_vol = (4.0 / 3.0) * math.pi * 1.0**3
        expected = cylinder_vol + sphere_vol
        assert capsule.volume() == pytest.approx(expected)

    def test_type(self):
        """Test geometry type."""
        from linkforge.core.models import GeometryType

        capsule = Capsule(radius=1.0, length=2.0)
        assert capsule.type == GeometryType.CAPSULE
