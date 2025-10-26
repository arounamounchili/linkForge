"""Tests for the enhanced validation system."""

from linkforge.core.models.geometry import Box
from linkforge.core.models.joint import Joint, JointLimits, JointType
from linkforge.core.models.link import Inertial, InertiaTensor, Link, Visual
from linkforge.core.models.robot import Robot
from linkforge.core.validation import RobotValidator, Severity, ValidationResult


def test_validation_result_creation():
    """Test creating a validation result."""
    result = ValidationResult(robot_name="test_robot")

    assert result.robot_name == "test_robot"
    assert result.is_valid
    assert not result.has_warnings
    assert result.error_count == 0
    assert result.warning_count == 0


def test_validation_result_add_error():
    """Test adding an error to validation result."""
    result = ValidationResult()

    result.add_error(
        title="Test Error",
        message="This is a test error",
        affected_objects=["link1", "link2"],
        suggestion="Fix the error",
    )

    assert not result.is_valid
    assert result.error_count == 1
    assert result.warning_count == 0
    assert len(result.errors) == 1

    error = result.errors[0]
    assert error.title == "Test Error"
    assert error.message == "This is a test error"
    assert error.affected_objects == ["link1", "link2"]
    assert error.suggestion == "Fix the error"
    assert error.is_error
    assert not error.is_warning


def test_validation_result_add_warning():
    """Test adding a warning to validation result."""
    result = ValidationResult()

    result.add_warning(
        title="Test Warning",
        message="This is a test warning",
        suggestion="Consider fixing",
    )

    assert result.is_valid  # Warnings don't block validity
    assert result.has_warnings
    assert result.error_count == 0
    assert result.warning_count == 1
    assert len(result.warnings) == 1

    warning = result.warnings[0]
    assert warning.title == "Test Warning"
    assert warning.is_warning
    assert not warning.is_error


def test_valid_robot():
    """Test validation of a valid robot."""
    robot = Robot(name="valid_robot")

    # Create valid robot structure
    base = Link(name="base", inertial=Inertial(mass=1.0))
    link1 = Link(name="link1", inertial=Inertial(mass=0.5))

    robot.add_link(base)
    robot.add_link(link1)
    robot.add_joint(
        Joint(
            name="joint1",
            type=JointType.REVOLUTE,
            parent="base",
            child="link1",
            limits=JointLimits(lower=-1.57, upper=1.57, effort=10.0, velocity=1.0),
        )
    )

    # Validate
    validator = RobotValidator(robot)
    result = validator.validate()

    assert result.is_valid
    assert result.error_count == 0
    # Note: Will have warnings about missing geometry


def test_robot_with_no_links():
    """Test validation of robot with no links."""
    robot = Robot(name="empty_robot")

    validator = RobotValidator(robot)
    result = validator.validate()

    assert not result.is_valid
    assert result.error_count == 1
    assert result.errors[0].title == "No links"


def test_duplicate_link_names():
    """Test validation of robot with duplicate link names."""
    robot = Robot(name="duplicate_links")

    # This should raise ValueError when adding second link
    robot.links.append(Link(name="duplicate", inertial=Inertial(mass=1.0)))
    robot.links.append(Link(name="duplicate", inertial=Inertial(mass=1.0)))

    validator = RobotValidator(robot)
    result = validator.validate()

    assert not result.is_valid
    errors = [e for e in result.errors if e.title == "Duplicate link name"]
    assert len(errors) == 1


def test_duplicate_joint_names():
    """Test validation of robot with duplicate joint names."""
    robot = Robot(name="duplicate_joints")

    base = Link(name="base", inertial=Inertial(mass=1.0))
    link1 = Link(name="link1", inertial=Inertial(mass=0.5))
    link2 = Link(name="link2", inertial=Inertial(mass=0.5))

    robot.add_link(base)
    robot.add_link(link1)
    robot.add_link(link2)

    # Add duplicate joints by manipulating list directly
    robot.joints.append(
        Joint(name="duplicate", type=JointType.FIXED, parent="base", child="link1")
    )
    robot.joints.append(
        Joint(name="duplicate", type=JointType.FIXED, parent="base", child="link2")
    )

    validator = RobotValidator(robot)
    result = validator.validate()

    assert not result.is_valid
    errors = [e for e in result.errors if e.title == "Duplicate joint name"]
    assert len(errors) == 1


def test_missing_parent_link():
    """Test validation of joint with missing parent link."""
    robot = Robot(name="missing_parent")

    link1 = Link(name="link1", inertial=Inertial(mass=1.0))
    robot.add_link(link1)

    # Add joint with invalid parent (bypass add_joint validation)
    robot.joints.append(
        Joint(name="joint1", type=JointType.FIXED, parent="nonexistent", child="link1")
    )

    validator = RobotValidator(robot)
    result = validator.validate()

    assert not result.is_valid
    errors = [e for e in result.errors if e.title == "Missing parent link"]
    assert len(errors) == 1
    assert "nonexistent" in errors[0].message


def test_missing_child_link():
    """Test validation of joint with missing child link."""
    robot = Robot(name="missing_child")

    base = Link(name="base", inertial=Inertial(mass=1.0))
    robot.add_link(base)

    # Add joint with invalid child (bypass add_joint validation)
    robot.joints.append(
        Joint(name="joint1", type=JointType.FIXED, parent="base", child="nonexistent")
    )

    validator = RobotValidator(robot)
    result = validator.validate()

    assert not result.is_valid
    errors = [e for e in result.errors if e.title == "Missing child link"]
    assert len(errors) == 1


def test_disconnected_link():
    """Test validation of disconnected link (shows as multiple root links)."""
    robot = Robot(name="disconnected")

    base = Link(name="base", inertial=Inertial(mass=1.0))
    link1 = Link(name="link1", inertial=Inertial(mass=0.5))
    link2 = Link(name="link2", inertial=Inertial(mass=0.5))  # Disconnected

    robot.add_link(base)
    robot.add_link(link1)
    robot.add_link(link2)

    # Only connect link1 to base
    robot.add_joint(
        Joint(name="joint1", type=JointType.FIXED, parent="base", child="link1")
    )

    validator = RobotValidator(robot)
    result = validator.validate()

    assert not result.is_valid
    # Disconnected link creates multiple root links
    errors = [e for e in result.errors if e.title == "Multiple root links"]
    assert len(errors) == 1
    assert "link2" in errors[0].message or "base" in errors[0].message


def test_multiple_parent_joints():
    """Test validation of link with multiple parent joints."""
    robot = Robot(name="multiple_parents")

    base = Link(name="base", inertial=Inertial(mass=1.0))
    link1 = Link(name="link1", inertial=Inertial(mass=0.5))
    link2 = Link(name="link2", inertial=Inertial(mass=0.5))

    robot.add_link(base)
    robot.add_link(link1)
    robot.add_link(link2)

    # Add first joint normally
    robot.add_joint(
        Joint(name="joint1", type=JointType.FIXED, parent="base", child="link1")
    )

    # Bypass add_joint validation to create invalid structure
    # Both joints have link2 as child (link2 has multiple parents)
    robot.joints.append(
        Joint(name="joint2", type=JointType.FIXED, parent="base", child="link2")
    )
    robot.joints.append(
        Joint(name="joint3", type=JointType.FIXED, parent="link1", child="link2")
    )

    validator = RobotValidator(robot)
    result = validator.validate()

    assert not result.is_valid
    errors = [e for e in result.errors if e.title == "Multiple parent joints"]
    assert len(errors) == 1


def test_low_mass_warning():
    """Test warning for very low mass link."""
    robot = Robot(name="low_mass")

    # Link with very low mass
    base = Link(name="base", inertial=Inertial(mass=0.001))  # 1 gram
    robot.add_link(base)

    validator = RobotValidator(robot)
    result = validator.validate()

    assert result.is_valid  # Not an error
    assert result.has_warnings
    warnings = [w for w in result.warnings if w.title == "Very low mass"]
    assert len(warnings) == 1
    assert "0.001" in warnings[0].message or "base" in warnings[0].message


def test_missing_inertia_warning():
    """Test warning for missing inertia."""
    robot = Robot(name="no_inertia")

    # Link without inertia
    base = Link(name="base", inertial=None)
    robot.add_link(base)

    validator = RobotValidator(robot)
    result = validator.validate()

    assert result.is_valid
    assert result.has_warnings
    warnings = [w for w in result.warnings if w.title == "Missing inertia"]
    assert len(warnings) == 1


def test_missing_geometry_warnings():
    """Test warnings for missing visual and collision geometry."""
    robot = Robot(name="no_geometry")

    # Link with no geometry
    base = Link(name="base", inertial=Inertial(mass=1.0))
    robot.add_link(base)

    validator = RobotValidator(robot)
    result = validator.validate()

    assert result.is_valid
    assert result.has_warnings

    # Should have warnings for both visual and collision
    visual_warnings = [w for w in result.warnings if w.title == "No visual geometry"]
    collision_warnings = [w for w in result.warnings if w.title == "No collision geometry"]

    assert len(visual_warnings) == 1
    assert len(collision_warnings) == 1


def test_validation_result_string_representation():
    """Test string representation of validation results."""
    result = ValidationResult(robot_name="my_robot")

    # Valid with no warnings
    assert "✓" in str(result)
    assert "valid" in str(result).lower()

    # Valid with warnings
    result.add_warning("Test", "Test warning")
    assert "⚠" in str(result)
    assert "warning" in str(result).lower()

    # Invalid with errors
    result.add_error("Test", "Test error")
    assert "✗" in str(result)
    assert "error" in str(result).lower()
