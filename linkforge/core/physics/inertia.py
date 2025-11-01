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


def calculate_mesh_inertia_from_triangles(
    vertices: list[tuple[float, float, float]],
    triangles: list[tuple[int, int, int]],
    mass: float,
) -> InertiaTensor:
    """Calculate inertia tensor for a triangle mesh using tetrahedralization.

    Decompose the mesh into tetrahedra and integrate their inertia contributions.
    Based on "Computing the moment of inertia of a solid defined by a triangle mesh"
    and standard tetrahedral inertia formulas.

    The algorithm assumes the mesh is closed and represents a solid volume.

    Args:
        vertices: List of (x, y, z) vertex coordinates
        triangles: List of (v0, v1, v2) triangle indices (into vertices list)
        mass: Total mass in kg

    Returns:
        Inertia tensor about center of mass
    """
    if mass <= 0:
        return InertiaTensor.zero()

    if not vertices or not triangles:
        return InertiaTensor.zero()

    # Accumulators for volume-weighted properties
    total_volume = 0.0
    weighted_com = [0.0, 0.0, 0.0]
    # Inertia tensor components about origin
    ixx, iyy, izz = 0.0, 0.0, 0.0
    ixy, ixz, iyz = 0.0, 0.0, 0.0

    # Process each triangle as a tetrahedron with apex at origin
    for tri in triangles:
        # Get vertices
        a = vertices[tri[0]]
        b = vertices[tri[1]]
        c = vertices[tri[2]]

        # Compute signed volume of tetrahedron
        # V = (1/6) | a · (b × c) |
        det = (
            a[0] * (b[1] * c[2] - b[2] * c[1])
            - a[1] * (b[0] * c[2] - b[2] * c[0])
            + a[2] * (b[0] * c[1] - b[1] * c[0])
        )
        tet_vol = det / 6.0

        total_volume += tet_vol

        # Centroid of tetrahedron (origin + a + b + c) / 4
        tet_com = [(a[i] + b[i] + c[i]) / 4.0 for i in range(3)]

        # Weighted COM contribution
        for i in range(3):
            weighted_com[i] += tet_vol * tet_com[i]

        # For a tetrahedron with one vertex at origin and others at a, b, c:
        # The inertia tensor can be computed using canonical tetrahedral formulas
        # We use the covariance matrix approach for tetrahedra

        # Vertices of tetrahedron (origin, a, b, c)
        verts = [(0, 0, 0), a, b, c]

        # Compute inertia using discrete mass distribution at vertices
        # For a tetrahedron, integrate over the volume
        #
        # Use formula: I = (density * volume / 20) * sum over edges
        coeff = tet_vol / 20.0

        for v in verts:
            for w in verts:
                ixx += coeff * (v[1] * w[1] + v[2] * w[2])
                iyy += coeff * (v[0] * w[0] + v[2] * w[2])
                izz += coeff * (v[0] * w[0] + v[1] * w[1])
                ixy -= coeff * (v[0] * w[1] + v[1] * w[0]) / 2.0
                ixz -= coeff * (v[0] * w[2] + v[2] * w[0]) / 2.0
                iyz -= coeff * (v[1] * w[2] + v[2] * w[1]) / 2.0

    # Check for degenerate mesh
    if abs(total_volume) < 1e-10:
        return InertiaTensor.zero()

    # Compute center of mass
    cx = weighted_com[0] / total_volume
    cy = weighted_com[1] / total_volume
    cz = weighted_com[2] / total_volume

    # Compute density from mass and volume
    density = mass / abs(total_volume)

    # Scale inertia by density
    ixx *= density
    iyy *= density
    izz *= density
    ixy *= density
    ixz *= density
    iyz *= density

    # Apply parallel axis theorem to translate to center of mass
    # I_cm = I_origin - m * d^2
    ixx -= mass * (cy * cy + cz * cz)
    iyy -= mass * (cx * cx + cz * cz)
    izz -= mass * (cx * cx + cy * cy)
    ixy += mass * cx * cy
    ixz += mass * cx * cz
    iyz += mass * cy * cz

    return InertiaTensor(
        ixx=abs(ixx),
        ixy=ixy,
        ixz=ixz,
        iyy=abs(iyy),
        iyz=iyz,
        izz=abs(izz),
    )


def calculate_mesh_inertia(mesh: Mesh, mass: float, method: str = "box") -> InertiaTensor:
    """Calculate approximate inertia tensor for a mesh.

    Args:
        mesh: Mesh geometry
        mass: Mass in kg
        method: Approximation method ("box", "sphere", or "cylinder")

    Returns:
        Approximate inertia tensor

    Note:
        This function is for use in the core layer when mesh geometry is not available.
        In the Blender integration layer, use calculate_mesh_inertia_from_triangles()
        with actual mesh data for accurate results.

        For now, we approximate using bounding box (scaled by mesh.scale).
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
