"""Tests for Gazebo URDF extension models."""

from __future__ import annotations

import pytest

from linkforge.core.models import (
    GazeboElement,
    GazeboPlugin,
    create_camera_plugin,
    create_differential_drive_plugin,
    create_imu_plugin,
    create_joint_state_publisher_plugin,
    create_lidar_plugin,
)


class TestGazeboPlugin:
    """Tests for GazeboPlugin model."""

    def test_plugin_creation(self):
        """Test creating a basic plugin."""
        plugin = GazeboPlugin(
            name="test_plugin",
            filename="libtest.so",
        )
        assert plugin.name == "test_plugin"
        assert plugin.filename == "libtest.so"
        assert len(plugin.parameters) == 0

    def test_plugin_with_parameters(self):
        """Test creating a plugin with parameters."""
        plugin = GazeboPlugin(
            name="test_plugin",
            filename="libtest.so",
            parameters={"param1": "value1", "param2": "42"},
        )
        assert plugin.parameters["param1"] == "value1"
        assert plugin.parameters["param2"] == "42"

    def test_get_parameter(self):
        """Test getting plugin parameter."""
        plugin = GazeboPlugin(
            name="test",
            filename="lib.so",
            parameters={"key": "value"},
        )
        assert plugin.get_parameter("key") == "value"
        assert plugin.get_parameter("missing") is None
        assert plugin.get_parameter("missing", "default") == "default"

    def test_with_parameter(self):
        """Test adding parameter to plugin."""
        plugin = GazeboPlugin(name="test", filename="lib.so")
        new_plugin = plugin.with_parameter("key", "value")
        assert new_plugin.get_parameter("key") == "value"
        assert plugin.get_parameter("key") is None  # Original unchanged

    def test_empty_name(self):
        """Test that empty name raises error."""
        with pytest.raises(ValueError, match="Plugin name cannot be empty"):
            GazeboPlugin(name="", filename="lib.so")

    def test_empty_filename(self):
        """Test that empty filename raises error."""
        with pytest.raises(ValueError, match="Plugin filename cannot be empty"):
            GazeboPlugin(name="test", filename="")


class TestGazeboElement:
    """Tests for GazeboElement model."""

    def test_robot_level_element(self):
        """Test creating a robot-level Gazebo element (no reference)."""
        element = GazeboElement(
            reference=None,
            properties={"gravity": "true"},
            static=True,
        )
        assert element.reference is None
        assert element.properties["gravity"] == "true"
        assert element.static is True

    def test_link_element(self):
        """Test creating a link-level Gazebo element."""
        element = GazeboElement(
            reference="base_link",
            material="Gazebo/Red",
            self_collide=True,
            mu1=0.8,
            mu2=0.8,
            kp=1000.0,
            kd=100.0,
        )
        assert element.reference == "base_link"
        assert element.material == "Gazebo/Red"
        assert element.self_collide is True
        assert element.mu1 == pytest.approx(0.8)
        assert element.mu2 == pytest.approx(0.8)
        assert element.kp == pytest.approx(1000.0)
        assert element.kd == pytest.approx(100.0)

    def test_joint_element(self):
        """Test creating a joint-level Gazebo element."""
        element = GazeboElement(
            reference="joint1",
            stop_cfm=0.0,
            stop_erp=0.2,
            provide_feedback=True,
            implicit_spring_damper=True,
        )
        assert element.reference == "joint1"
        assert element.stop_cfm == pytest.approx(0.0)
        assert element.stop_erp == pytest.approx(0.2)
        assert element.provide_feedback is True
        assert element.implicit_spring_damper is True

    def test_element_with_plugin(self):
        """Test Gazebo element with plugin."""
        plugin = GazeboPlugin(name="test", filename="lib.so")
        element = GazeboElement(
            reference=None,
            plugins=[plugin],
        )
        assert len(element.plugins) == 1
        assert element.plugins[0].name == "test"

    def test_element_with_properties(self):
        """Test Gazebo element with custom properties."""
        element = GazeboElement(
            reference="link1",
            properties={"custom_prop": "custom_value"},
        )
        assert element.properties["custom_prop"] == "custom_value"

    def test_empty_reference_string(self):
        """Test that empty string reference raises error."""
        with pytest.raises(ValueError, match="Gazebo reference cannot be empty string"):
            GazeboElement(reference="")


class TestGazeboPluginFactories:
    """Tests for Gazebo plugin factory functions."""

    def test_create_differential_drive_plugin(self):
        """Test creating a differential drive plugin."""
        plugin = create_differential_drive_plugin(
            name="diff_drive",
            left_joint="left_wheel_joint",
            right_joint="right_wheel_joint",
            wheel_separation=0.5,
            wheel_diameter=0.2,
        )
        assert plugin.name == "diff_drive"
        assert plugin.filename == "libgazebo_ros_diff_drive.so"
        assert plugin.get_parameter("left_joint") == "left_wheel_joint"
        assert plugin.get_parameter("right_joint") == "right_wheel_joint"
        assert plugin.get_parameter("wheel_separation") == "0.5"
        assert plugin.get_parameter("wheel_diameter") == "0.2"
        assert plugin.get_parameter("command_topic") == "cmd_vel"
        assert plugin.get_parameter("odometry_topic") == "odom"
        assert plugin.get_parameter("robot_base_frame") == "base_link"

    def test_create_differential_drive_custom_topics(self):
        """Test differential drive plugin with custom topics."""
        plugin = create_differential_drive_plugin(
            command_topic="robot/cmd_vel",
            odometry_topic="robot/odom",
            robot_base_frame="robot_base",
        )
        assert plugin.get_parameter("command_topic") == "robot/cmd_vel"
        assert plugin.get_parameter("odometry_topic") == "robot/odom"
        assert plugin.get_parameter("robot_base_frame") == "robot_base"

    def test_create_joint_state_publisher_plugin(self):
        """Test creating a joint state publisher plugin."""
        plugin = create_joint_state_publisher_plugin(
            name="joint_states",
            update_rate=100.0,
        )
        assert plugin.name == "joint_states"
        assert plugin.filename == "libgazebo_ros_joint_state_publisher.so"
        assert plugin.get_parameter("update_rate") == "100.0"

    def test_create_camera_plugin(self):
        """Test creating a camera plugin."""
        plugin = create_camera_plugin(
            name="camera_controller",
            camera_name="front_camera",
            image_topic="camera/image",
            camera_info_topic="camera/info",
            frame_name="camera_optical_frame",
            update_rate=30.0,
        )
        assert plugin.name == "camera_controller"
        assert plugin.filename == "libgazebo_ros_camera.so"
        assert plugin.get_parameter("camera_name") == "front_camera"
        assert plugin.get_parameter("image_topic_name") == "camera/image"
        assert plugin.get_parameter("camera_info_topic_name") == "camera/info"
        assert plugin.get_parameter("frame_name") == "camera_optical_frame"
        assert plugin.get_parameter("update_rate") == "30.0"

    def test_create_imu_plugin(self):
        """Test creating an IMU plugin."""
        plugin = create_imu_plugin(
            name="imu_controller",
            topic="imu/data",
            frame_name="imu_link",
            update_rate=200.0,
        )
        assert plugin.name == "imu_controller"
        assert plugin.filename == "libgazebo_ros_imu_sensor.so"
        assert plugin.get_parameter("topicName") == "imu/data"
        assert plugin.get_parameter("frameName") == "imu_link"
        assert plugin.get_parameter("update_rate") == "200.0"
        assert plugin.get_parameter("initial_orientation_as_reference") == "false"

    def test_create_lidar_plugin(self):
        """Test creating a LIDAR plugin."""
        plugin = create_lidar_plugin(
            name="lidar_controller",
            topic="scan",
            frame_name="lidar_link",
        )
        assert plugin.name == "lidar_controller"
        assert plugin.filename == "libgazebo_ros_ray_sensor.so"
        assert plugin.get_parameter("topicName") == "scan"
        assert plugin.get_parameter("frameName") == "lidar_link"

    def test_create_lidar_plugin_custom_topic(self):
        """Test LIDAR plugin with custom topic."""
        plugin = create_lidar_plugin(
            name="laser",
            topic="robot/scan",
            frame_name="robot/laser",
        )
        assert plugin.get_parameter("topicName") == "robot/scan"
        assert plugin.get_parameter("frameName") == "robot/laser"
