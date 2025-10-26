"""Mass and center of mass calculations for geometries."""

from __future__ import annotations

from ..models.geometry import Box, Capsule, Cylinder, Geometry, Mesh, Sphere, Vector3


def calculate_volume(geometry: Geometry) -> float:
    """Calculate volume of a geometry.

    Args:
        geometry: Geometry (Box, Cylinder, Sphere, Capsule, or Mesh)

    Returns:
        Volume in m³

    Raises:
        ValueError: If geometry type is not supported
    """
    if isinstance(geometry, (Box, Cylinder, Sphere, Capsule)):
        return geometry.volume()
    elif isinstance(geometry, Mesh):
        # For meshes, volume must be computed from actual geometry
        # This is a placeholder - will be computed in Blender integration
        return 0.0
    else:
        raise ValueError(f"Unsupported geometry type: {type(geometry)}")


def calculate_mass_from_density(geometry: Geometry, density: float) -> float:
    """Calculate mass from geometry volume and material density.

    Args:
        geometry: Geometry
        density: Material density in kg/m³

    Returns:
        Mass in kg

    Common material densities:
        - Aluminum: 2700 kg/m³
        - Steel: 7850 kg/m³
        - Plastic (ABS): 1040 kg/m³
        - Carbon fiber: 1600 kg/m³
        - Rubber: 1100 kg/m³
    """
    if density <= 0:
        raise ValueError(f"Density must be positive, got {density}")

    volume = calculate_volume(geometry)
    return volume * density


def calculate_center_of_mass(geometry: Geometry) -> Vector3:
    """Calculate center of mass for a geometry.

    Args:
        geometry: Geometry

    Returns:
        Center of mass position (in geometry's local frame)

    Note:
        For symmetric primitive shapes centered at origin, COM is at (0, 0, 0).
        For meshes, this should be computed from actual geometry.
    """
    # For all primitive shapes centered at origin, COM is at origin
    if isinstance(geometry, (Box, Cylinder, Sphere, Capsule)):
        return Vector3(0.0, 0.0, 0.0)
    elif isinstance(geometry, Mesh):
        # For meshes, COM should be computed from actual mesh geometry
        # This will be implemented in Blender integration
        return Vector3(0.0, 0.0, 0.0)
    else:
        raise ValueError(f"Unsupported geometry type: {type(geometry)}")


# Common material densities (kg/m³)
class MaterialDensity:
    """Common material densities in kg/m³."""

    # Metals
    ALUMINUM = 2700.0
    STEEL = 7850.0
    STAINLESS_STEEL = 8000.0
    TITANIUM = 4500.0
    BRASS = 8500.0
    COPPER = 8960.0

    # Plastics
    ABS = 1040.0
    PLA = 1250.0  # 3D printing
    NYLON = 1150.0
    POLYCARBONATE = 1200.0
    ACRYLIC = 1180.0

    # Composites
    CARBON_FIBER = 1600.0
    FIBERGLASS = 1800.0

    # Other
    RUBBER = 1100.0
    WOOD = 700.0  # Average for hardwood
    FOAM = 50.0  # Polyurethane foam
