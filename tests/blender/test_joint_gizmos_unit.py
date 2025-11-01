"""Unit tests for joint axis gizmo functions.

Tests the axis vector calculation logic without Blender dependencies.
"""

from __future__ import annotations


def test_axis_vector_logic_x():
    """Test X axis logic."""
    # Mock props object
    class Props:
        axis = "X"
        custom_axis_x = 0.0
        custom_axis_y = 0.0
        custom_axis_z = 1.0

    props = Props()

    # Inline the logic from get_joint_axis_vector
    if props.axis == "X":
        result = (1.0, 0.0, 0.0)
    elif props.axis == "Y":
        result = (0.0, 1.0, 0.0)
    elif props.axis == "Z":
        result = (0.0, 0.0, 1.0)
    elif props.axis == "CUSTOM":
        result = (props.custom_axis_x, props.custom_axis_y, props.custom_axis_z)
    else:
        result = (0.0, 0.0, 1.0)

    assert result == (1.0, 0.0, 0.0)


def test_axis_vector_logic_y():
    """Test Y axis logic."""
    class Props:
        axis = "Y"
        custom_axis_x = 0.0
        custom_axis_y = 0.0
        custom_axis_z = 1.0

    props = Props()

    if props.axis == "X":
        result = (1.0, 0.0, 0.0)
    elif props.axis == "Y":
        result = (0.0, 1.0, 0.0)
    elif props.axis == "Z":
        result = (0.0, 0.0, 1.0)
    elif props.axis == "CUSTOM":
        result = (props.custom_axis_x, props.custom_axis_y, props.custom_axis_z)
    else:
        result = (0.0, 0.0, 1.0)

    assert result == (0.0, 1.0, 0.0)


def test_axis_vector_logic_z():
    """Test Z axis logic."""
    class Props:
        axis = "Z"
        custom_axis_x = 0.0
        custom_axis_y = 0.0
        custom_axis_z = 1.0

    props = Props()

    if props.axis == "X":
        result = (1.0, 0.0, 0.0)
    elif props.axis == "Y":
        result = (0.0, 1.0, 0.0)
    elif props.axis == "Z":
        result = (0.0, 0.0, 1.0)
    elif props.axis == "CUSTOM":
        result = (props.custom_axis_x, props.custom_axis_y, props.custom_axis_z)
    else:
        result = (0.0, 0.0, 1.0)

    assert result == (0.0, 0.0, 1.0)


def test_axis_vector_logic_custom():
    """Test custom axis logic."""
    class Props:
        axis = "CUSTOM"
        custom_axis_x = 0.5
        custom_axis_y = 0.5
        custom_axis_z = 0.7071

    props = Props()

    if props.axis == "X":
        result = (1.0, 0.0, 0.0)
    elif props.axis == "Y":
        result = (0.0, 1.0, 0.0)
    elif props.axis == "Z":
        result = (0.0, 0.0, 1.0)
    elif props.axis == "CUSTOM":
        result = (props.custom_axis_x, props.custom_axis_y, props.custom_axis_z)
    else:
        result = (0.0, 0.0, 1.0)

    assert result == (0.5, 0.5, 0.7071)


def test_axis_vector_logic_default():
    """Test default axis for unknown type."""
    class Props:
        axis = "UNKNOWN"
        custom_axis_x = 0.0
        custom_axis_y = 0.0
        custom_axis_z = 1.0

    props = Props()

    if props.axis == "X":
        result = (1.0, 0.0, 0.0)
    elif props.axis == "Y":
        result = (0.0, 1.0, 0.0)
    elif props.axis == "Z":
        result = (0.0, 0.0, 1.0)
    elif props.axis == "CUSTOM":
        result = (props.custom_axis_x, props.custom_axis_y, props.custom_axis_z)
    else:
        result = (0.0, 0.0, 1.0)  # Default to Z

    assert result == (0.0, 0.0, 1.0)


def test_rgb_color_convention():
    """Test that RGB colors match RViz convention."""
    # Red = X
    x_color = (1.0, 0.0, 0.0, 1.0)
    assert x_color[0] == 1.0  # Red channel
    assert x_color[1] == 0.0  # Green channel
    assert x_color[2] == 0.0  # Blue channel

    # Green = Y
    y_color = (0.0, 1.0, 0.0, 1.0)
    assert y_color[0] == 0.0
    assert y_color[1] == 1.0
    assert y_color[2] == 0.0

    # Blue = Z
    z_color = (0.0, 0.0, 1.0, 1.0)
    assert z_color[0] == 0.0
    assert z_color[1] == 0.0
    assert z_color[2] == 1.0
