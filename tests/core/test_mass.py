"""Tests for mass and center of mass calculations."""

from __future__ import annotations

import math

import pytest

from linkforge.core.models.geometry import Box, Capsule, Cylinder, Mesh, Sphere, Vector3
from linkforge.core.physics.mass import (
    MaterialDensity,
    calculate_center_of_mass,
    calculate_mass_from_density,
    calculate_volume,
)


class TestCalculateVolume:
    """Tests for calculate_volume function."""

    def test_box_volume(self):
        """Test volume calculation for a box."""
        box = Box(size=Vector3(2.0, 3.0, 4.0))
        volume = calculate_volume(box)
        assert volume == pytest.approx(24.0)  # 2 * 3 * 4

    def test_box_unit_cube(self):
        """Test volume of a unit cube."""
        box = Box(size=Vector3(1.0, 1.0, 1.0))
        volume = calculate_volume(box)
        assert volume == pytest.approx(1.0)

    def test_cylinder_volume(self):
        """Test volume calculation for a cylinder."""
        cylinder = Cylinder(radius=1.0, length=2.0)
        volume = calculate_volume(cylinder)
        expected = math.pi * 1.0**2 * 2.0
        assert volume == pytest.approx(expected)

    def test_cylinder_small(self):
        """Test volume for a small cylinder."""
        cylinder = Cylinder(radius=0.1, length=0.5)
        volume = calculate_volume(cylinder)
        expected = math.pi * 0.1**2 * 0.5
        assert volume == pytest.approx(expected)

    def test_sphere_volume(self):
        """Test volume calculation for a sphere."""
        sphere = Sphere(radius=1.0)
        volume = calculate_volume(sphere)
        expected = (4.0 / 3.0) * math.pi * 1.0**3
        assert volume == pytest.approx(expected)

    def test_sphere_unit(self):
        """Test volume of unit sphere."""
        sphere = Sphere(radius=1.0)
        volume = calculate_volume(sphere)
        assert volume == pytest.approx(4.18879, rel=1e-4)  # 4/3 * π

    def test_sphere_radius_2(self):
        """Test volume of sphere with radius 2."""
        sphere = Sphere(radius=2.0)
        volume = calculate_volume(sphere)
        expected = (4.0 / 3.0) * math.pi * 2.0**3
        assert volume == pytest.approx(expected)

    def test_capsule_volume(self):
        """Test volume calculation for a capsule."""
        capsule = Capsule(radius=1.0, length=2.0)
        volume = calculate_volume(capsule)

        # Capsule volume = cylinder volume + sphere volume
        # Cylinder: π * r² * h
        # Sphere: 4/3 * π * r³
        cylinder_vol = math.pi * 1.0**2 * 2.0
        sphere_vol = (4.0 / 3.0) * math.pi * 1.0**3
        expected = cylinder_vol + sphere_vol

        assert volume == pytest.approx(expected)

    def test_capsule_small(self):
        """Test volume for a small capsule."""
        capsule = Capsule(radius=0.5, length=1.0)
        volume = calculate_volume(capsule)

        cylinder_vol = math.pi * 0.5**2 * 1.0
        sphere_vol = (4.0 / 3.0) * math.pi * 0.5**3
        expected = cylinder_vol + sphere_vol

        assert volume == pytest.approx(expected)

    def test_mesh_volume_placeholder(self):
        """Test that mesh returns placeholder volume (0.0)."""
        mesh = Mesh(filepath="test.stl")
        volume = calculate_volume(mesh)
        # Mesh volume computation is a placeholder for now
        assert volume == 0.0

    def test_unsupported_geometry_type(self):
        """Test that unsupported geometry raises ValueError."""

        class UnsupportedGeometry:
            """Fake unsupported geometry."""

            pass

        with pytest.raises(ValueError, match="Unsupported geometry type"):
            calculate_volume(UnsupportedGeometry())  # type: ignore


class TestCalculateMassFromDensity:
    """Tests for calculate_mass_from_density function."""

    def test_mass_box_aluminum(self):
        """Test mass calculation for aluminum box."""
        box = Box(size=Vector3(1.0, 1.0, 1.0))  # 1 m³
        mass = calculate_mass_from_density(box, MaterialDensity.ALUMINUM)
        # 1 m³ * 2700 kg/m³ = 2700 kg
        assert mass == pytest.approx(2700.0)

    def test_mass_box_steel(self):
        """Test mass calculation for steel box."""
        box = Box(size=Vector3(0.1, 0.1, 0.1))  # 0.001 m³
        mass = calculate_mass_from_density(box, MaterialDensity.STEEL)
        # 0.001 m³ * 7850 kg/m³ = 7.85 kg
        assert mass == pytest.approx(7.85)

    def test_mass_cylinder_plastic(self):
        """Test mass calculation for plastic cylinder."""
        cylinder = Cylinder(radius=0.5, length=1.0)
        mass = calculate_mass_from_density(cylinder, MaterialDensity.ABS)

        volume = math.pi * 0.5**2 * 1.0  # π * r² * h
        expected_mass = volume * MaterialDensity.ABS

        assert mass == pytest.approx(expected_mass)

    def test_mass_sphere_titanium(self):
        """Test mass calculation for titanium sphere."""
        sphere = Sphere(radius=0.1)
        mass = calculate_mass_from_density(sphere, MaterialDensity.TITANIUM)

        volume = (4.0 / 3.0) * math.pi * 0.1**3
        expected_mass = volume * MaterialDensity.TITANIUM

        assert mass == pytest.approx(expected_mass)

    def test_mass_capsule_carbon_fiber(self):
        """Test mass calculation for carbon fiber capsule."""
        capsule = Capsule(radius=0.05, length=0.2)
        mass = calculate_mass_from_density(capsule, MaterialDensity.CARBON_FIBER)

        # Calculate expected volume
        cylinder_vol = math.pi * 0.05**2 * 0.2
        sphere_vol = (4.0 / 3.0) * math.pi * 0.05**3
        volume = cylinder_vol + sphere_vol
        expected_mass = volume * MaterialDensity.CARBON_FIBER

        assert mass == pytest.approx(expected_mass)

    def test_mass_custom_density(self):
        """Test mass calculation with custom density."""
        box = Box(size=Vector3(2.0, 2.0, 2.0))  # 8 m³
        custom_density = 1500.0  # kg/m³
        mass = calculate_mass_from_density(box, custom_density)
        assert mass == pytest.approx(12000.0)  # 8 * 1500

    def test_mass_negative_density_raises_error(self):
        """Test that negative density raises ValueError."""
        box = Box(size=Vector3(1.0, 1.0, 1.0))
        with pytest.raises(ValueError, match="Density must be positive"):
            calculate_mass_from_density(box, -100.0)

    def test_mass_zero_density_raises_error(self):
        """Test that zero density raises ValueError."""
        cylinder = Cylinder(radius=1.0, length=1.0)
        with pytest.raises(ValueError, match="Density must be positive"):
            calculate_mass_from_density(cylinder, 0.0)

    def test_mass_very_small_density(self):
        """Test mass with very small density (foam)."""
        box = Box(size=Vector3(1.0, 1.0, 1.0))
        mass = calculate_mass_from_density(box, MaterialDensity.FOAM)
        assert mass == pytest.approx(50.0)  # 1 * 50


class TestCalculateCenterOfMass:
    """Tests for calculate_center_of_mass function."""

    def test_com_box_at_origin(self):
        """Test that box COM is at origin."""
        box = Box(size=Vector3(1.0, 1.0, 1.0))
        com = calculate_center_of_mass(box)
        assert com.x == pytest.approx(0.0)
        assert com.y == pytest.approx(0.0)
        assert com.z == pytest.approx(0.0)

    def test_com_cylinder_at_origin(self):
        """Test that cylinder COM is at origin."""
        cylinder = Cylinder(radius=0.5, length=1.0)
        com = calculate_center_of_mass(cylinder)
        assert com.x == pytest.approx(0.0)
        assert com.y == pytest.approx(0.0)
        assert com.z == pytest.approx(0.0)

    def test_com_sphere_at_origin(self):
        """Test that sphere COM is at origin."""
        sphere = Sphere(radius=1.0)
        com = calculate_center_of_mass(sphere)
        assert com.x == pytest.approx(0.0)
        assert com.y == pytest.approx(0.0)
        assert com.z == pytest.approx(0.0)

    def test_com_capsule_at_origin(self):
        """Test that capsule COM is at origin."""
        capsule = Capsule(radius=0.1, length=0.5)
        com = calculate_center_of_mass(capsule)
        assert com.x == pytest.approx(0.0)
        assert com.y == pytest.approx(0.0)
        assert com.z == pytest.approx(0.0)

    def test_com_mesh_placeholder(self):
        """Test that mesh COM returns origin (placeholder)."""
        mesh = Mesh(filepath="test.stl")
        com = calculate_center_of_mass(mesh)
        # Mesh COM computation is a placeholder for now
        assert com.x == pytest.approx(0.0)
        assert com.y == pytest.approx(0.0)
        assert com.z == pytest.approx(0.0)

    def test_com_unsupported_geometry_type(self):
        """Test that unsupported geometry raises ValueError."""

        class UnsupportedGeometry:
            """Fake unsupported geometry."""

            pass

        with pytest.raises(ValueError, match="Unsupported geometry type"):
            calculate_center_of_mass(UnsupportedGeometry())  # type: ignore


class TestMaterialDensity:
    """Tests for MaterialDensity constants."""

    def test_metal_densities(self):
        """Test that metal densities are reasonable."""
        assert MaterialDensity.ALUMINUM == 2700.0
        assert MaterialDensity.STEEL == 7850.0
        assert MaterialDensity.STAINLESS_STEEL == 8000.0
        assert MaterialDensity.TITANIUM == 4500.0
        assert MaterialDensity.BRASS == 8500.0
        assert MaterialDensity.COPPER == 8960.0

    def test_plastic_densities(self):
        """Test that plastic densities are reasonable."""
        assert MaterialDensity.ABS == 1040.0
        assert MaterialDensity.PLA == 1250.0
        assert MaterialDensity.NYLON == 1150.0
        assert MaterialDensity.POLYCARBONATE == 1200.0
        assert MaterialDensity.ACRYLIC == 1180.0

    def test_composite_densities(self):
        """Test that composite densities are reasonable."""
        assert MaterialDensity.CARBON_FIBER == 1600.0
        assert MaterialDensity.FIBERGLASS == 1800.0

    def test_other_densities(self):
        """Test other material densities."""
        assert MaterialDensity.RUBBER == 1100.0
        assert MaterialDensity.WOOD == 700.0
        assert MaterialDensity.FOAM == 50.0

    def test_density_ordering(self):
        """Test that densities are in expected order."""
        # Foam should be lightest
        assert MaterialDensity.FOAM < MaterialDensity.WOOD
        assert MaterialDensity.WOOD < MaterialDensity.ABS
        assert MaterialDensity.ABS < MaterialDensity.ALUMINUM
        assert MaterialDensity.ALUMINUM < MaterialDensity.STEEL
        # Copper should be one of the heaviest
        assert MaterialDensity.COPPER > MaterialDensity.ALUMINUM


class TestIntegration:
    """Integration tests combining volume, density, and mass calculations."""

    def test_realistic_robot_link_mass(self):
        """Test realistic mass calculation for a robot link."""
        # Small aluminum cylinder (typical robot link)
        link_geometry = Cylinder(radius=0.025, length=0.15)  # 2.5cm radius, 15cm long

        volume = calculate_volume(link_geometry)
        mass = calculate_mass_from_density(link_geometry, MaterialDensity.ALUMINUM)

        # Verify reasonable values
        assert volume > 0
        assert mass > 0
        assert mass < 1.0  # Should be less than 1 kg for this small part

        # Calculate expected mass manually
        expected_volume = math.pi * 0.025**2 * 0.15
        expected_mass = expected_volume * MaterialDensity.ALUMINUM
        assert mass == pytest.approx(expected_mass)

    def test_comparison_materials_same_geometry(self):
        """Test that different materials give different masses for same geometry."""
        box = Box(size=Vector3(0.1, 0.1, 0.1))

        mass_aluminum = calculate_mass_from_density(box, MaterialDensity.ALUMINUM)
        mass_steel = calculate_mass_from_density(box, MaterialDensity.STEEL)
        mass_plastic = calculate_mass_from_density(box, MaterialDensity.ABS)
        mass_foam = calculate_mass_from_density(box, MaterialDensity.FOAM)

        # Steel should be heaviest, foam lightest
        assert mass_foam < mass_plastic < mass_aluminum < mass_steel

        # Check ratios match density ratios
        density_ratio = MaterialDensity.STEEL / MaterialDensity.ALUMINUM
        mass_ratio = mass_steel / mass_aluminum
        assert mass_ratio == pytest.approx(density_ratio)

    def test_large_structure_mass(self):
        """Test mass calculation for a large structure."""
        # Large steel box (1m x 1m x 1m)
        large_box = Box(size=Vector3(1.0, 1.0, 1.0))
        mass = calculate_mass_from_density(large_box, MaterialDensity.STEEL)

        # 1 m³ of steel should be 7850 kg
        assert mass == pytest.approx(7850.0)

    def test_com_independent_of_size(self):
        """Test that COM is at origin regardless of geometry size."""
        small_box = Box(size=Vector3(0.1, 0.1, 0.1))
        large_box = Box(size=Vector3(10.0, 10.0, 10.0))

        com_small = calculate_center_of_mass(small_box)
        com_large = calculate_center_of_mass(large_box)

        # Both should be at origin
        assert com_small == com_large
        assert com_small.x == 0.0
        assert com_small.y == 0.0
        assert com_small.z == 0.0
