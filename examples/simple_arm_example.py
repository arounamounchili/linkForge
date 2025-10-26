"""Example: Create a simple 2-link robot arm and export to URDF.

This demonstrates the core LinkForge API for building robot models.
"""

import math
from pathlib import Path

from linkforge.core.generators import URDFGenerator
from linkforge.core.models import (
    Box,
    Collision,
    Color,
    Cylinder,
    Inertial,
    Joint,
    JointLimits,
    JointType,
    Link,
    Material,
    Robot,
    Transform,
    Vector3,
    Visual,
)
from linkforge.core.physics import calculate_inertia


def create_simple_arm() -> Robot:
    """Create a simple 2-DOF robot arm."""
    robot = Robot(name="simple_arm")

    # Base link
    base_geometry = Cylinder(radius=0.1, length=0.05)
    base_material = Material(name="gray", color=Color(0.7, 0.7, 0.7, 1.0))
    base_mass = 1.0

    base_link = Link(
        name="base_link",
        visual=Visual(geometry=base_geometry, material=base_material),
        collision=Collision(geometry=base_geometry),
        inertial=Inertial(
            mass=base_mass,
            inertia=calculate_inertia(base_geometry, base_mass),
        ),
    )
    robot.add_link(base_link)

    # Link 1 (first arm segment)
    link1_geometry = Box(size=Vector3(0.05, 0.05, 0.3))
    link1_material = Material(name="blue", color=Color(0.2, 0.4, 0.8, 1.0))
    link1_mass = 0.5

    link1 = Link(
        name="link1",
        visual=Visual(
            geometry=link1_geometry,
            material=link1_material,
            origin=Transform(xyz=Vector3(0, 0, 0.15)),  # Center visual at joint origin
        ),
        collision=Collision(
            geometry=link1_geometry, origin=Transform(xyz=Vector3(0, 0, 0.15))
        ),
        inertial=Inertial(
            mass=link1_mass,
            origin=Transform(xyz=Vector3(0, 0, 0.15)),
            inertia=calculate_inertia(link1_geometry, link1_mass),
        ),
    )
    robot.add_link(link1)

    # Link 2 (second arm segment)
    link2_geometry = Box(size=Vector3(0.05, 0.05, 0.25))
    link2_material = Material(name="orange", color=Color(1.0, 0.5, 0.0, 1.0))
    link2_mass = 0.3

    link2 = Link(
        name="link2",
        visual=Visual(
            geometry=link2_geometry,
            material=link2_material,
            origin=Transform(xyz=Vector3(0, 0, 0.125)),
        ),
        collision=Collision(
            geometry=link2_geometry, origin=Transform(xyz=Vector3(0, 0, 0.125))
        ),
        inertial=Inertial(
            mass=link2_mass,
            origin=Transform(xyz=Vector3(0, 0, 0.125)),
            inertia=calculate_inertia(link2_geometry, link2_mass),
        ),
    )
    robot.add_link(link2)

    # Joint 1 (base to link1)
    joint1 = Joint(
        name="joint1",
        type=JointType.REVOLUTE,
        parent="base_link",
        child="link1",
        origin=Transform(xyz=Vector3(0, 0, 0.025)),  # Top of base
        axis=Vector3(0, 0, 1),  # Rotate around Z
        limits=JointLimits(lower=-math.pi, upper=math.pi, effort=10.0, velocity=2.0),
    )
    robot.add_joint(joint1)

    # Joint 2 (link1 to link2)
    joint2 = Joint(
        name="joint2",
        type=JointType.REVOLUTE,
        parent="link1",
        child="link2",
        origin=Transform(xyz=Vector3(0, 0, 0.3)),  # Top of link1
        axis=Vector3(0, 1, 0),  # Rotate around Y
        limits=JointLimits(lower=-math.pi / 2, upper=math.pi / 2, effort=5.0, velocity=2.0),
    )
    robot.add_joint(joint2)

    return robot


def main():
    """Generate URDF for simple arm."""
    # Create robot
    robot = create_simple_arm()

    # Validate
    errors = robot.validate_tree_structure()
    if errors:
        print("❌ Validation errors:")
        for error in errors:
            print(f"  - {error}")
        return

    print("✅ Robot validation passed!")
    print(f"   Name: {robot.name}")
    print(f"   Links: {len(robot.links)}")
    print(f"   Joints: {len(robot.joints)}")
    print(f"   DOF: {robot.degrees_of_freedom}")
    print(f"   Total mass: {robot.total_mass:.2f} kg")
    print()

    # Generate URDF
    generator = URDFGenerator(pretty_print=True)
    urdf_content = generator.generate(robot)

    # Write to file
    output_path = Path(__file__).parent / "simple_arm.urdf"
    generator.write(robot, output_path)

    print(f"✅ URDF generated successfully!")
    print(f"   Output: {output_path}")
    print()
    print("First 50 lines of URDF:")
    print("-" * 80)
    for i, line in enumerate(urdf_content.split("\n")[:50], 1):
        print(f"{i:3d} | {line}")
    print("-" * 80)


if __name__ == "__main__":
    main()
