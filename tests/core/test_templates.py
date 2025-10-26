"""Tests for robot templates."""

from __future__ import annotations

import pytest

from linkforge.core.models import Robot
from linkforge.templates import RobotTemplate, TemplateCategory
from linkforge.templates.loader import (
    clear_registry,
    get_all_templates,
    get_template,
    get_template_categories,
    get_templates_by_category,
    register_template,
)


@pytest.fixture(autouse=True)
def _clear_template_registry():
    """Clear template registry before and after each test."""
    clear_registry()
    yield
    clear_registry()


def test_robot_template_creation():
    """Test creating a RobotTemplate."""

    def simple_factory() -> Robot:
        return Robot(name="test", links=[], joints=[])

    template = RobotTemplate(
        id="test_template",
        name="Test Template",
        description="A test template",
        category=TemplateCategory.OTHER,
        factory=simple_factory,
    )

    assert template.id == "test_template"
    assert template.name == "Test Template"
    assert template.category == TemplateCategory.OTHER
    assert template.create_robot().name == "test"


def test_register_template():
    """Test registering a template."""

    def factory() -> Robot:
        return Robot(name="test", links=[], joints=[])

    template = RobotTemplate(
        id="test",
        name="Test",
        description="Test template",
        category=TemplateCategory.OTHER,
        factory=factory,
    )

    register_template(template)
    retrieved = get_template("test")

    assert retrieved is not None
    assert retrieved.id == "test"


def test_register_duplicate_template_raises_error():
    """Test that registering duplicate template ID raises error."""

    def factory() -> Robot:
        return Robot(name="test", links=[], joints=[])

    template1 = RobotTemplate(
        id="duplicate",
        name="First",
        description="First template",
        category=TemplateCategory.OTHER,
        factory=factory,
    )

    template2 = RobotTemplate(
        id="duplicate",
        name="Second",
        description="Second template",
        category=TemplateCategory.OTHER,
        factory=factory,
    )

    register_template(template1)

    with pytest.raises(ValueError, match="already registered"):
        register_template(template2)


def test_get_nonexistent_template():
    """Test getting a template that doesn't exist."""
    result = get_template("nonexistent")
    assert result is None


def test_get_all_templates():
    """Test getting all registered templates."""

    def factory() -> Robot:
        return Robot(name="test", links=[], joints=[])

    template1 = RobotTemplate(
        id="test1",
        name="B Template",  # Intentionally not alphabetical
        description="Test",
        category=TemplateCategory.OTHER,
        factory=factory,
    )

    template2 = RobotTemplate(
        id="test2",
        name="A Template",
        description="Test",
        category=TemplateCategory.OTHER,
        factory=factory,
    )

    register_template(template1)
    register_template(template2)

    templates = get_all_templates()

    assert len(templates) == 2
    assert templates[0].name == "A Template"  # Sorted by name
    assert templates[1].name == "B Template"


def test_get_templates_by_category():
    """Test getting templates filtered by category."""

    def factory() -> Robot:
        return Robot(name="test", links=[], joints=[])

    arm_template = RobotTemplate(
        id="arm",
        name="Arm",
        description="Arm template",
        category=TemplateCategory.ARM,
        factory=factory,
    )

    mobile_template = RobotTemplate(
        id="mobile",
        name="Mobile",
        description="Mobile template",
        category=TemplateCategory.MOBILE,
        factory=factory,
    )

    register_template(arm_template)
    register_template(mobile_template)

    arm_templates = get_templates_by_category(TemplateCategory.ARM)
    mobile_templates = get_templates_by_category(TemplateCategory.MOBILE)

    assert len(arm_templates) == 1
    assert arm_templates[0].id == "arm"
    assert len(mobile_templates) == 1
    assert mobile_templates[0].id == "mobile"


def test_get_template_categories():
    """Test getting all unique categories."""

    def factory() -> Robot:
        return Robot(name="test", links=[], joints=[])

    register_template(
        RobotTemplate(
            id="arm1",
            name="Arm 1",
            description="Arm",
            category=TemplateCategory.ARM,
            factory=factory,
        )
    )

    register_template(
        RobotTemplate(
            id="arm2",
            name="Arm 2",
            description="Arm",
            category=TemplateCategory.ARM,
            factory=factory,
        )
    )

    register_template(
        RobotTemplate(
            id="mobile1",
            name="Mobile",
            description="Mobile",
            category=TemplateCategory.MOBILE,
            factory=factory,
        )
    )

    categories = get_template_categories()

    assert len(categories) == 2
    assert TemplateCategory.ARM in categories
    assert TemplateCategory.MOBILE in categories


def test_built_in_templates_are_registered():
    """Test that built-in templates are automatically registered."""
    # Import library to trigger registration
    from linkforge.templates import library  # noqa: F401

    templates = get_all_templates()

    # Should have at least the 3 built-in templates
    assert len(templates) >= 3

    # Check specific templates exist
    arm_template = get_template("arm_2dof")
    mobile_template = get_template("mobile_4wheel")
    gripper_template = get_template("gripper_parallel")

    assert arm_template is not None
    assert mobile_template is not None
    assert gripper_template is not None


def test_arm_2dof_template_creates_valid_robot():
    """Test that 2-DOF arm template creates a valid robot."""
    from linkforge.templates.library import ARM_2DOF_TEMPLATE

    robot = ARM_2DOF_TEMPLATE.create_robot()

    assert robot.name == "arm_2dof"
    assert len(robot.links) == 3  # base + 2 arm links
    assert len(robot.joints) == 2  # 2 revolute joints
    root = robot.get_root_link()
    assert root is not None
    assert root.name == "base_link"

    # Validate structure
    errors = robot.validate_tree_structure()
    assert len(errors) == 0, f"Validation errors: {errors}"


def test_mobile_4wheel_template_creates_valid_robot():
    """Test that 4-wheel mobile base template creates a valid robot."""
    from linkforge.templates.library import MOBILE_4WHEEL_TEMPLATE

    robot = MOBILE_4WHEEL_TEMPLATE.create_robot()

    assert robot.name == "mobile_4wheel"
    assert len(robot.links) == 5  # chassis + 4 wheels
    assert len(robot.joints) == 4  # 2 continuous + 2 fixed
    root = robot.get_root_link()
    assert root is not None
    assert root.name == "base_link"

    # Validate structure
    errors = robot.validate_tree_structure()
    assert len(errors) == 0, f"Validation errors: {errors}"


def test_gripper_parallel_template_creates_valid_robot():
    """Test that parallel gripper template creates a valid robot."""
    from linkforge.templates.library import GRIPPER_PARALLEL_TEMPLATE

    robot = GRIPPER_PARALLEL_TEMPLATE.create_robot()

    assert robot.name == "gripper_parallel"
    assert len(robot.links) == 3  # palm + 2 fingers
    assert len(robot.joints) == 2  # 2 prismatic joints
    root = robot.get_root_link()
    assert root is not None
    assert root.name == "palm"

    # Check that right finger mimics left finger
    right_joint = next(j for j in robot.joints if j.name == "right_finger_joint")
    assert right_joint.mimic is not None
    assert right_joint.mimic.joint == "left_finger_joint"

    # Validate structure
    errors = robot.validate_tree_structure()
    assert len(errors) == 0, f"Validation errors: {errors}"


def test_all_templates_export_valid_urdf():
    """Test that all built-in templates can export valid URDF."""
    from linkforge.core.generators import URDFGenerator
    from linkforge.templates import library  # noqa: F401

    templates = get_all_templates()

    for template in templates:
        robot = template.create_robot()
        generator = URDFGenerator(robot)
        urdf_xml = generator.generate()

        # Basic checks
        assert urdf_xml is not None
        assert "<robot" in urdf_xml
        assert "</robot>" in urdf_xml
        assert f'name="{robot.name}"' in urdf_xml
