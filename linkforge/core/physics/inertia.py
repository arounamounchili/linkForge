"""Inertia tensor calculation for primitive geometries.

Based on standard formulas for common shapes:
https://en.wikipedia.org/wiki/List_of_moments_of_inertia
"""

from __future__ import annotations

import math

from ..models.geometry import Box, Capsule, Cylinder, Geometry, Mesh, Sphere
from ..models.link import InertiaTensor


def calculate_box_inertia(box: Box, mass: float) -> InertiaTensor:
    """Calculate inertia tensor for a box (rectangular cuboid).

    Args:
        box: Box geometry
        mass: Mass in kg

    Returns:
        Inertia tensor about the center of mass

    Formula for box with dimensions (x, y, z):
        Ixx = (1/12) * m * (y² + z²)
        Iyy = (1/12) * m * (x² + z²)
        Izz = (1/12) * m * (x² + y²)
        Ixy = Ixz = Iyz = 0 (off-diagonal terms are zero for symmetrical shapes)
    """
    if mass <= 0:
        return InertiaTensor.zero()

    x, y, z = box.size.x, box.size.y, box.size.z

    ixx = (1.0 / 12.0) * mass * (y * y + z * z)
    iyy = (1.0 / 12.0) * mass * (x * x + z * z)
    izz = (1.0 / 12.0) * mass * (x * x + y * y)

    return InertiaTensor(ixx=ixx, ixy=0.0, ixz=0.0, iyy=iyy, iyz=0.0, izz=izz)


def calculate_cylinder_inertia(cylinder: Cylinder, mass: float) -> InertiaTensor:
    """Calculate inertia tensor for a cylinder (axis along Z).

    Args:
        cylinder: Cylinder geometry
        mass: Mass in kg

    Returns:
        Inertia tensor about the center of mass

    Formula for cylinder with radius r and height h (along Z):
        Ixx = Iyy = (1/12) * m * (3r² + h²)
        Izz = (1/2) * m * r²
        Ixy = Ixz = Iyz = 0
    """
    if mass <= 0:
        return InertiaTensor.zero()

    r = cylinder.radius
    h = cylinder.length

    ixx = iyy = (1.0 / 12.0) * mass * (3 * r * r + h * h)
    izz = 0.5 * mass * r * r

    return InertiaTensor(ixx=ixx, ixy=0.0, ixz=0.0, iyy=iyy, iyz=0.0, izz=izz)


def calculate_sphere_inertia(sphere: Sphere, mass: float) -> InertiaTensor:
    """Calculate inertia tensor for a sphere.

    Args:
        sphere: Sphere geometry
        mass: Mass in kg

    Returns:
        Inertia tensor about the center of mass

    Formula for sphere with radius r:
        Ixx = Iyy = Izz = (2/5) * m * r²
        Ixy = Ixz = Iyz = 0
    """
    if mass <= 0:
        return InertiaTensor.zero()

    r = sphere.radius
    i = (2.0 / 5.0) * mass * r * r

    return InertiaTensor(ixx=i, ixy=0.0, ixz=0.0, iyy=i, iyz=0.0, izz=i)


def calculate_capsule_inertia(capsule: Capsule, mass: float) -> InertiaTensor:
    """Calculate inertia tensor for a capsule (cylinder + hemispherical ends).

    Args:
        capsule: Capsule geometry
        mass: Mass in kg

    Returns:
        Inertia tensor about the center of mass

    A capsule consists of a cylinder and a sphere (two hemispheres).
    We use the parallel axis theorem to combine them.
    """
    if mass <= 0:
        return InertiaTensor.zero()

    r = capsule.radius
    h = capsule.length  # Length of cylindrical portion

    # Calculate volumes
    cylinder_volume = math.pi * r * r * h
    sphere_volume = (4.0 / 3.0) * math.pi * r * r * r
    total_volume = cylinder_volume + sphere_volume

    # Mass distribution
    cylinder_mass = mass * (cylinder_volume / total_volume)
    sphere_mass = mass * (sphere_volume / total_volume)

    # Cylinder inertia (centered at origin)
    cyl_ixx = (1.0 / 12.0) * cylinder_mass * (3 * r * r + h * h)
    cyl_izz = 0.5 * cylinder_mass * r * r

    # Sphere inertia (two hemispheres at distance h/2 from center)
    # Using parallel axis theorem: I = Icm + m*d²
    sphere_i_cm = (2.0 / 5.0) * sphere_mass * r * r

    # Distance from capsule center to hemisphere center
    # For hemisphere, COM is at 3r/8 from flat face
    # Distance from capsule center: h/2 + 3r/8
    d = h / 2.0 + (3.0 * r / 8.0)

    # Parallel axis theorem for Ixx, Iyy
    sphere_ixx = sphere_i_cm + sphere_mass * d * d

    # Izz is unaffected by translation along Z
    sphere_izz = sphere_i_cm

    # Combine cylinder and sphere
    ixx = iyy = cyl_ixx + sphere_ixx
    izz = cyl_izz + sphere_izz

    return InertiaTensor(ixx=ixx, ixy=0.0, ixz=0.0, iyy=iyy, iyz=0.0, izz=izz)


def calculate_mesh_inertia(mesh: Mesh, mass: float, method: str = "box") -> InertiaTensor:
    """Calculate approximate inertia tensor for a mesh.

    Args:
        mesh: Mesh geometry
        mass: Mass in kg
        method: Approximation method ("box", "sphere", or "cylinder")

    Returns:
        Approximate inertia tensor

    Note:
        This is a placeholder. Actual mesh inertia calculation requires:
        1. Loading the mesh file
        2. Computing the mesh's bounding volume or convex hull
        3. Using triangle mesh integration or voxelization

        For now, we approximate using bounding box/sphere/cylinder.
        In the Blender integration, we'll use actual mesh geometry.
    """
    if mass <= 0:
        return InertiaTensor.zero()

    # Placeholder: return conservative estimate
    # In practice, this will be computed from actual mesh geometry in Blender
    # Default to a scaled unit cube for safety
    unit_box = Box(size=mesh.scale)
    return calculate_box_inertia(unit_box, mass)


def calculate_inertia(geometry: Geometry, mass: float) -> InertiaTensor:
    """Calculate inertia tensor for any geometry type.

    Args:
        geometry: Geometry (Box, Cylinder, Sphere, Capsule, or Mesh)
        mass: Mass in kg

    Returns:
        Inertia tensor about the center of mass

    Raises:
        ValueError: If geometry type is not supported
    """
    if mass <= 0:
        return InertiaTensor.zero()

    if isinstance(geometry, Box):
        return calculate_box_inertia(geometry, mass)
    elif isinstance(geometry, Cylinder):
        return calculate_cylinder_inertia(geometry, mass)
    elif isinstance(geometry, Sphere):
        return calculate_sphere_inertia(geometry, mass)
    elif isinstance(geometry, Capsule):
        return calculate_capsule_inertia(geometry, mass)
    elif isinstance(geometry, Mesh):
        return calculate_mesh_inertia(geometry, mass)
    else:
        raise ValueError(f"Unsupported geometry type: {type(geometry)}")
