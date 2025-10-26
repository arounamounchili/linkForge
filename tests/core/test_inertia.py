"""Tests for inertia calculations."""

from __future__ import annotations

import math

import pytest

from linkforge.core.models import Box, Capsule, Cylinder, InertiaTensor, Sphere, Vector3
from linkforge.core.physics import (
    calculate_box_inertia,
    calculate_capsule_inertia,
    calculate_cylinder_inertia,
    calculate_inertia,
    calculate_sphere_inertia,
)


class TestInertiaTensor:
    """Tests for InertiaTensor."""

    def test_creation(self):
        """Test creating an inertia tensor."""
        inertia = InertiaTensor(ixx=1.0, ixy=0.0, ixz=0.0, iyy=1.0, iyz=0.0, izz=1.0)
        assert inertia.ixx == 1.0
        assert inertia.iyy == 1.0
        assert inertia.izz == 1.0

    def test_zero_inertia(self):
        """Test zero inertia tensor."""
        inertia = InertiaTensor.zero()
        assert inertia.ixx > 0
        assert inertia.iyy > 0
        assert inertia.izz > 0

    def test_invalid_negative_diagonal(self):
        """Test that negative diagonal elements raise error."""
        with pytest.raises(ValueError, match="positive"):
            InertiaTensor(ixx=-1.0, ixy=0.0, ixz=0.0, iyy=1.0, iyz=0.0, izz=1.0)

    def test_invalid_triangle_inequality(self):
        """Test that triangle inequality violation raises error."""
        with pytest.raises(ValueError, match="triangle inequality"):
            # This violates Ixx + Iyy >= Izz
            InertiaTensor(ixx=1.0, ixy=0.0, ixz=0.0, iyy=1.0, iyz=0.0, izz=10.0)


class TestBoxInertia:
    """Tests for box inertia calculation."""

    def test_unit_cube(self):
        """Test inertia of unit cube with unit mass."""
        box = Box(size=Vector3(1.0, 1.0, 1.0))
        inertia = calculate_box_inertia(box, mass=1.0)

        # For unit cube: I = (1/12) * m * (1² + 1²) = 1/6
        expected = 1.0 / 6.0
        assert inertia.ixx == pytest.approx(expected)
        assert inertia.iyy == pytest.approx(expected)
        assert inertia.izz == pytest.approx(expected)
        assert inertia.ixy == 0.0
        assert inertia.ixz == 0.0
        assert inertia.iyz == 0.0

    def test_rectangular_box(self):
        """Test inertia of rectangular box."""
        box = Box(size=Vector3(2.0, 3.0, 4.0))
        mass = 10.0
        inertia = calculate_box_inertia(box, mass)

        # Ixx = (1/12) * m * (y² + z²) = (1/12) * 10 * (9 + 16) = 20.833...
        ixx = (1.0 / 12.0) * mass * (3.0**2 + 4.0**2)
        iyy = (1.0 / 12.0) * mass * (2.0**2 + 4.0**2)
        izz = (1.0 / 12.0) * mass * (2.0**2 + 3.0**2)

        assert inertia.ixx == pytest.approx(ixx)
        assert inertia.iyy == pytest.approx(iyy)
        assert inertia.izz == pytest.approx(izz)

    def test_zero_mass(self):
        """Test that zero mass returns zero inertia."""
        box = Box(size=Vector3(1.0, 1.0, 1.0))
        inertia = calculate_box_inertia(box, mass=0.0)
        assert inertia == InertiaTensor.zero()


class TestCylinderInertia:
    """Tests for cylinder inertia calculation."""

    def test_unit_cylinder(self):
        """Test inertia of unit cylinder."""
        cyl = Cylinder(radius=1.0, length=2.0)
        mass = 1.0
        inertia = calculate_cylinder_inertia(cyl, mass)

        # Ixx = Iyy = (1/12) * m * (3r² + h²) = (1/12) * 1 * (3 + 4) = 7/12
        # Izz = (1/2) * m * r² = 0.5
        ixx = (1.0 / 12.0) * mass * (3 * 1.0**2 + 2.0**2)
        izz = 0.5 * mass * 1.0**2

        assert inertia.ixx == pytest.approx(ixx)
        assert inertia.iyy == pytest.approx(ixx)
        assert inertia.izz == pytest.approx(izz)
        assert inertia.ixy == 0.0

    def test_zero_mass(self):
        """Test that zero mass returns zero inertia."""
        cyl = Cylinder(radius=1.0, length=1.0)
        inertia = calculate_cylinder_inertia(cyl, mass=0.0)
        assert inertia == InertiaTensor.zero()


class TestSphereInertia:
    """Tests for sphere inertia calculation."""

    def test_unit_sphere(self):
        """Test inertia of unit sphere."""
        sphere = Sphere(radius=1.0)
        mass = 1.0
        inertia = calculate_sphere_inertia(sphere, mass)

        # I = (2/5) * m * r² = 0.4
        expected = (2.0 / 5.0) * mass * 1.0**2

        assert inertia.ixx == pytest.approx(expected)
        assert inertia.iyy == pytest.approx(expected)
        assert inertia.izz == pytest.approx(expected)

    def test_zero_mass(self):
        """Test that zero mass returns zero inertia."""
        sphere = Sphere(radius=1.0)
        inertia = calculate_sphere_inertia(sphere, mass=0.0)
        assert inertia == InertiaTensor.zero()


class TestCapsuleInertia:
    """Tests for capsule inertia calculation."""

    def test_capsule(self):
        """Test inertia of capsule."""
        capsule = Capsule(radius=0.5, length=2.0)
        mass = 5.0
        inertia = calculate_capsule_inertia(capsule, mass)

        # Should have symmetric Ixx = Iyy and smaller Izz
        assert inertia.ixx == pytest.approx(inertia.iyy)
        assert inertia.izz < inertia.ixx

    def test_zero_mass(self):
        """Test that zero mass returns zero inertia."""
        capsule = Capsule(radius=1.0, length=2.0)
        inertia = calculate_capsule_inertia(capsule, mass=0.0)
        assert inertia == InertiaTensor.zero()


class TestCalculateInertia:
    """Tests for generic calculate_inertia function."""

    def test_box(self):
        """Test calculation for box."""
        box = Box(size=Vector3(1.0, 1.0, 1.0))
        inertia = calculate_inertia(box, mass=1.0)
        expected = calculate_box_inertia(box, mass=1.0)
        assert inertia == expected

    def test_cylinder(self):
        """Test calculation for cylinder."""
        cyl = Cylinder(radius=1.0, length=1.0)
        inertia = calculate_inertia(cyl, mass=1.0)
        expected = calculate_cylinder_inertia(cyl, mass=1.0)
        assert inertia == expected

    def test_sphere(self):
        """Test calculation for sphere."""
        sphere = Sphere(radius=1.0)
        inertia = calculate_inertia(sphere, mass=1.0)
        expected = calculate_sphere_inertia(sphere, mass=1.0)
        assert inertia == expected

    def test_capsule(self):
        """Test calculation for capsule."""
        capsule = Capsule(radius=1.0, length=2.0)
        inertia = calculate_inertia(capsule, mass=1.0)
        expected = calculate_capsule_inertia(capsule, mass=1.0)
        assert inertia == expected
