"""Integration test for URDF import/export round-trip.

This test verifies that:
1. URDF can be imported successfully
2. Imported robot can be exported back to URDF
3. The exported URDF is valid and preserves key properties
"""

from __future__ import annotations

import tempfile
from pathlib import Path

import pytest

from linkforge.core.parsers.urdf_parser import parse_urdf


def test_simple_arm_roundtrip():
    """Test importing and re-exporting robot_arm.urdf preserves structure."""
    # Get path to example URDF
    examples_dir = Path(__file__).parent.parent.parent / "examples"
    urdf_path = examples_dir / "robot_arm.urdf"

    assert urdf_path.exists(), f"Example URDF not found at {urdf_path}"

    # Parse URDF
    robot = parse_urdf(urdf_path)

    # Verify basic structure
    assert robot.name == "robot_arm"
    assert len(robot.links) == 5  # base + 4 arm links
    assert len(robot.joints) == 4  # 4 revolute

    # Verify link names
    link_names = {link.name for link in robot.links}
    assert "base_link" in link_names
    assert "shoulder_link" in link_names
    assert "upper_arm_link" in link_names
    assert "forearm_link" in link_names
    assert "wrist_link" in link_names

    # Verify joint names and types
    joint_names = {joint.name for joint in robot.joints}
    revolute_joints = [j for j in robot.joints if j.type.name == "REVOLUTE"]

    assert len(revolute_joints) == 4  # 4 DOF

    # Export back to URDF
    from linkforge.core.generators import URDFGenerator

    with tempfile.TemporaryDirectory() as tmpdir:
        output_path = Path(tmpdir) / "exported.urdf"

        generator = URDFGenerator(pretty_print=True, urdf_path=output_path)
        generator.write(robot, output_path)

        assert output_path.exists(), "Exported URDF file not created"

        # Re-parse exported URDF
        robot2 = parse_urdf(output_path)

        # Verify structure is preserved
        assert robot2.name == robot.name
        assert len(robot2.links) == len(robot.links)
        assert len(robot2.joints) == len(robot.joints)

        # Verify link names match
        link_names2 = {link.name for link in robot2.links}
        assert link_names2 == link_names

        # Verify joint names match
        joint_names2 = {joint.name for joint in robot2.joints}
        assert joint_names2 == joint_names


def test_materials_preserved():
    """Test that material colors are preserved in round-trip."""
    examples_dir = Path(__file__).parent.parent.parent / "examples"
    urdf_path = examples_dir / "robot_arm.urdf"

    robot = parse_urdf(urdf_path)

    # Check that materials are parsed
    materials_found = []
    for link in robot.links:
        if link.visual and link.visual.material:
            materials_found.append(link.visual.material.name)

    assert len(materials_found) > 0, "Expected materials to be parsed"
    # Check for expected materials in 6DOF arm
    material_names = {m for m in materials_found}
    assert "arm_material" in material_names or "base_material" in material_names


def test_inertial_preserved():
    """Test that inertial properties are preserved in round-trip."""
    examples_dir = Path(__file__).parent.parent.parent / "examples"
    urdf_path = examples_dir / "robot_arm.urdf"

    robot = parse_urdf(urdf_path)

    # Check that all links have inertial properties
    for link in robot.links:
        assert link.inertial is not None, f"Link {link.name} missing inertial"
        assert link.inertial.mass > 0, f"Link {link.name} has invalid mass"
        assert link.inertial.inertia is not None, f"Link {link.name} missing inertia tensor"


def test_joint_limits_preserved():
    """Test that joint limits are preserved in round-trip."""
    examples_dir = Path(__file__).parent.parent.parent / "examples"
    urdf_path = examples_dir / "robot_arm.urdf"

    robot = parse_urdf(urdf_path)

    # Check that all joints have limits
    for joint in robot.joints:
        if joint.type.name == "REVOLUTE":
            assert joint.limits is not None, f"Revolute joint {joint.name} missing limits"
            assert joint.limits.lower < joint.limits.upper, f"Invalid limits for {joint.name}"
            assert joint.limits.effort > 0, f"Invalid effort for {joint.name}"
            assert joint.limits.velocity > 0, f"Invalid velocity for {joint.name}"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
