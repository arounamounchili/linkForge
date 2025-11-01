"""Tests for Robot model."""

from __future__ import annotations

import math

import pytest

from linkforge.core.models import (
    CameraInfo,
    GazeboElement,
    GazeboPlugin,
    IMUInfo,
    Inertial,
    InertiaTensor,
    Joint,
    JointLimits,
    JointType,
    Link,
    Robot,
    Sensor,
    SensorType,
    Transmission,
)


class TestRobot:
    """Tests for Robot model."""

    def test_creation(self):
        """Test creating a robot."""
        robot = Robot(name="test_robot")
        assert robot.name == "test_robot"
        assert len(robot.links) == 0
        assert len(robot.joints) == 0

    def test_invalid_name(self):
        """Test that invalid names raise error."""
        with pytest.raises(ValueError, match="invalid characters"):
            Robot(name="robot with spaces!")

    def test_add_link(self):
        """Test adding a link."""
        robot = Robot(name="test_robot")
        link = Link(name="link1")
        robot.add_link(link)

        assert len(robot.links) == 1
        assert robot.get_link("link1") == link

    def test_add_duplicate_link(self):
        """Test that adding duplicate link raises error."""
        robot = Robot(name="test_robot")
        link = Link(name="link1")
        robot.add_link(link)

        with pytest.raises(ValueError, match="already exists"):
            robot.add_link(link)

    def test_add_joint(self):
        """Test adding a joint."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))

        joint = Joint(
            name="joint1",
            type=JointType.FIXED,
            parent="link1",
            child="link2",
        )
        robot.add_joint(joint)

        assert len(robot.joints) == 1
        assert robot.get_joint("joint1") == joint

    def test_add_joint_missing_parent(self):
        """Test that adding joint with missing parent raises error."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link2"))

        joint = Joint(
            name="joint1",
            type=JointType.FIXED,
            parent="link1",  # Does not exist
            child="link2",
        )

        with pytest.raises(ValueError, match="not found"):
            robot.add_joint(joint)

    def test_get_root_link(self):
        """Test getting root link."""
        robot = Robot(name="test_robot")
        link1 = Link(name="link1")
        link2 = Link(name="link2")

        robot.add_link(link1)
        robot.add_link(link2)

        joint = Joint(
            name="joint1",
            type=JointType.FIXED,
            parent="link1",
            child="link2",
        )
        robot.add_joint(joint)

        root = robot.get_root_link()
        assert root == link1

    def test_total_mass(self):
        """Test calculating total mass."""
        robot = Robot(name="test_robot")

        inertia = InertiaTensor(ixx=1.0, ixy=0.0, ixz=0.0, iyy=1.0, iyz=0.0, izz=1.0)
        link1 = Link(name="link1", inertial=Inertial(mass=5.0, inertia=inertia))
        link2 = Link(name="link2", inertial=Inertial(mass=3.0, inertia=inertia))

        robot.add_link(link1)
        robot.add_link(link2)

        assert robot.total_mass == 8.0

    def test_degrees_of_freedom(self):
        """Test calculating degrees of freedom."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))
        robot.add_link(Link(name="link3"))

        # Add fixed joint (0 DOF)
        robot.add_joint(
            Joint(
                name="joint1",
                type=JointType.FIXED,
                parent="link1",
                child="link2",
            )
        )

        # Add revolute joint (1 DOF)
        robot.add_joint(
            Joint(
                name="joint2",
                type=JointType.REVOLUTE,
                parent="link2",
                child="link3",
                limits=JointLimits(lower=-math.pi, upper=math.pi),
            )
        )

        assert robot.degrees_of_freedom == 1


class TestRobotValidation:
    """Tests for robot validation."""

    def test_valid_robot(self):
        """Test that valid robot passes validation."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))

        robot.add_joint(
            Joint(
                name="joint1",
                type=JointType.FIXED,
                parent="link1",
                child="link2",
            )
        )

        errors = robot.validate_tree_structure()
        assert len(errors) == 0

    def test_empty_robot(self):
        """Test that empty robot fails validation."""
        robot = Robot(name="test_robot")
        errors = robot.validate_tree_structure()
        assert len(errors) > 0
        assert any("at least one link" in err for err in errors)

    def test_duplicate_link_names(self):
        """Test that duplicate link names fail validation."""
        robot = Robot(name="test_robot")
        robot.links.append(Link(name="link1"))
        robot.links.append(Link(name="link1"))  # Bypass add_link validation

        errors = robot.validate_tree_structure()
        assert any("Duplicate link names" in err for err in errors)

    def test_duplicate_joint_names(self):
        """Test that duplicate joint names fail validation."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))
        robot.add_link(Link(name="link3"))

        joint1 = Joint(name="joint1", type=JointType.FIXED, parent="link1", child="link2")
        joint2 = Joint(name="joint1", type=JointType.FIXED, parent="link2", child="link3")

        robot.joints.append(joint1)
        robot.joints.append(joint2)

        errors = robot.validate_tree_structure()
        assert any("Duplicate joint names" in err for err in errors)

    def test_missing_parent_link(self):
        """Test that missing parent link fails validation."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link2"))

        # Manually add joint to bypass add_joint validation
        robot.joints.append(
            Joint(
                name="joint1",
                type=JointType.FIXED,
                parent="link1",  # Does not exist
                child="link2",
            )
        )

        errors = robot.validate_tree_structure()
        assert any("parent link 'link1' not found" in err for err in errors)

    def test_cycle_detection(self):
        """Test that cycles are detected."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))

        # Create a cycle
        robot.joints.append(
            Joint(name="joint1", type=JointType.FIXED, parent="link1", child="link2")
        )
        robot.joints.append(
            Joint(name="joint2", type=JointType.FIXED, parent="link2", child="link1")
        )

        errors = robot.validate_tree_structure()
        assert any("cycle" in err.lower() for err in errors)

    def test_multiple_roots(self):
        """Test that multiple roots fail validation."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))
        robot.add_link(Link(name="link3"))

        # Only connect link2 and link3, leaving link1 as orphan root
        robot.add_joint(Joint(name="joint1", type=JointType.FIXED, parent="link2", child="link3"))

        errors = robot.validate_tree_structure()
        assert any("Multiple root links" in err for err in errors)

    def test_disconnected_link(self):
        """Test that disconnected links fail validation."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))
        robot.add_link(Link(name="link3"))

        # Connect link1 and link2, leaving link3 disconnected
        robot.add_joint(Joint(name="joint1", type=JointType.FIXED, parent="link1", child="link2"))

        errors = robot.validate_tree_structure()
        # link3 is disconnected and will be detected as a multiple root issue
        assert any("Multiple root links" in err for err in errors)


class TestRobotWithSensors:
    """Tests for Robot model with sensors."""

    def test_add_sensor(self):
        """Test adding a sensor to a robot."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="camera_link"))

        sensor = Sensor(
            name="camera",
            type=SensorType.CAMERA,
            link_name="camera_link",
            camera_info=CameraInfo(),
        )
        robot.add_sensor(sensor)

        assert len(robot.sensors) == 1
        assert robot.get_sensor("camera") == sensor

    def test_add_sensor_nonexistent_link(self):
        """Test that adding sensor to nonexistent link raises error."""
        robot = Robot(name="test_robot")

        sensor = Sensor(
            name="camera",
            type=SensorType.CAMERA,
            link_name="camera_link",  # Link doesn't exist
            camera_info=CameraInfo(),
        )

        with pytest.raises(ValueError, match="link .* not found"):
            robot.add_sensor(sensor)

    def test_add_duplicate_sensor(self):
        """Test that adding duplicate sensor raises error."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="camera_link"))

        sensor = Sensor(
            name="camera",
            type=SensorType.CAMERA,
            link_name="camera_link",
            camera_info=CameraInfo(),
        )
        robot.add_sensor(sensor)

        with pytest.raises(ValueError, match="already exists"):
            robot.add_sensor(sensor)

    def test_get_sensors_for_link(self):
        """Test getting all sensors for a link."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="sensor_link"))
        robot.add_link(Link(name="other_link"))

        camera = Sensor(
            name="camera",
            type=SensorType.CAMERA,
            link_name="sensor_link",
            camera_info=CameraInfo(),
        )
        imu = Sensor(
            name="imu",
            type=SensorType.IMU,
            link_name="sensor_link",
            imu_info=IMUInfo(),
        )
        other_sensor = Sensor(
            name="other_camera",
            type=SensorType.CAMERA,
            link_name="other_link",
            camera_info=CameraInfo(),
        )

        robot.add_sensor(camera)
        robot.add_sensor(imu)
        robot.add_sensor(other_sensor)

        sensors = robot.get_sensors_for_link("sensor_link")
        assert len(sensors) == 2
        assert camera in sensors
        assert imu in sensors
        assert other_sensor not in sensors

    def test_robot_str_with_sensors(self):
        """Test robot string representation includes sensors."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_sensor(
            Sensor(
                name="camera",
                type=SensorType.CAMERA,
                link_name="link1",
                camera_info=CameraInfo(),
            )
        )

        robot_str = str(robot)
        assert "sensors=1" in robot_str


class TestRobotWithTransmissions:
    """Tests for Robot model with transmissions."""

    def test_add_transmission(self):
        """Test adding a transmission to a robot."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))
        robot.add_joint(
            Joint(
                name="joint1",
                type=JointType.REVOLUTE,
                parent="link1",
                child="link2",
                limits=JointLimits(lower=-math.pi, upper=math.pi),
            )
        )

        trans = Transmission.create_simple(
            name="trans1",
            joint_name="joint1",
        )
        robot.add_transmission(trans)

        assert len(robot.transmissions) == 1
        assert robot.get_transmission("trans1") == trans

    def test_add_transmission_nonexistent_joint(self):
        """Test that adding transmission with nonexistent joint raises error."""
        robot = Robot(name="test_robot")

        trans = Transmission.create_simple(
            name="trans1",
            joint_name="joint1",  # Joint doesn't exist
        )

        with pytest.raises(ValueError, match="joint .* not found"):
            robot.add_transmission(trans)

    def test_add_duplicate_transmission(self):
        """Test that adding duplicate transmission raises error."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))
        robot.add_joint(
            Joint(
                name="joint1",
                type=JointType.REVOLUTE,
                parent="link1",
                child="link2",
                limits=JointLimits(lower=-math.pi, upper=math.pi),
            )
        )

        trans = Transmission.create_simple(
            name="trans1",
            joint_name="joint1",
        )
        robot.add_transmission(trans)

        with pytest.raises(ValueError, match="already exists"):
            robot.add_transmission(trans)

    def test_get_transmissions_for_joint(self):
        """Test getting all transmissions for a joint."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))
        robot.add_link(Link(name="link3"))

        robot.add_joint(
            Joint(
                name="joint1",
                type=JointType.REVOLUTE,
                parent="link1",
                child="link2",
                limits=JointLimits(lower=-math.pi, upper=math.pi),
            )
        )
        robot.add_joint(
            Joint(
                name="joint2",
                type=JointType.REVOLUTE,
                parent="link2",
                child="link3",
                limits=JointLimits(lower=-math.pi, upper=math.pi),
            )
        )

        trans1 = Transmission.create_simple(name="trans1", joint_name="joint1")
        trans2 = Transmission.create_simple(name="trans2", joint_name="joint2")

        robot.add_transmission(trans1)
        robot.add_transmission(trans2)

        transmissions = robot.get_transmissions_for_joint("joint1")
        assert len(transmissions) == 1
        assert trans1 in transmissions
        assert trans2 not in transmissions

    def test_robot_str_with_transmissions(self):
        """Test robot string representation includes transmissions."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))
        robot.add_joint(
            Joint(
                name="joint1",
                type=JointType.REVOLUTE,
                parent="link1",
                child="link2",
                limits=JointLimits(lower=-math.pi, upper=math.pi),
            )
        )

        robot.add_transmission(Transmission.create_simple("trans1", "joint1"))

        robot_str = str(robot)
        assert "transmissions=1" in robot_str


class TestRobotWithGazeboElements:
    """Tests for Robot model with Gazebo elements."""

    def test_add_robot_level_gazebo_element(self):
        """Test adding a robot-level Gazebo element."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))

        element = GazeboElement(
            reference=None,
            static=True,
        )
        robot.add_gazebo_element(element)

        assert len(robot.gazebo_elements) == 1
        robot_level = robot.get_robot_level_gazebo_elements()
        assert len(robot_level) == 1
        assert element in robot_level

    def test_add_link_gazebo_element(self):
        """Test adding a link-level Gazebo element."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="base_link"))

        element = GazeboElement(
            reference="base_link",
            material="Gazebo/Red",
        )
        robot.add_gazebo_element(element)

        assert len(robot.gazebo_elements) == 1
        link_elements = robot.get_gazebo_elements_for_link("base_link")
        assert len(link_elements) == 1
        assert element in link_elements

    def test_add_joint_gazebo_element(self):
        """Test adding a joint-level Gazebo element."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_link(Link(name="link2"))
        robot.add_joint(
            Joint(
                name="joint1",
                type=JointType.REVOLUTE,
                parent="link1",
                child="link2",
                limits=JointLimits(lower=-math.pi, upper=math.pi),
            )
        )

        element = GazeboElement(
            reference="joint1",
            provide_feedback=True,
        )
        robot.add_gazebo_element(element)

        joint_elements = robot.get_gazebo_elements_for_joint("joint1")
        assert len(joint_elements) == 1
        assert element in joint_elements

    def test_add_gazebo_element_invalid_reference(self):
        """Test that adding Gazebo element with invalid reference raises error."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))

        element = GazeboElement(
            reference="nonexistent_link",
            material="Gazebo/Red",
        )

        with pytest.raises(ValueError, match="does not match any link or joint"):
            robot.add_gazebo_element(element)

    def test_add_gazebo_element_with_plugin(self):
        """Test adding Gazebo element with plugin."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))

        plugin = GazeboPlugin(name="test_plugin", filename="lib.so")
        element = GazeboElement(
            reference=None,
            plugins=[plugin],
        )
        robot.add_gazebo_element(element)

        robot_level = robot.get_robot_level_gazebo_elements()
        assert len(robot_level[0].plugins) == 1

    def test_robot_str_with_gazebo_elements(self):
        """Test robot string representation includes Gazebo elements."""
        robot = Robot(name="test_robot")
        robot.add_link(Link(name="link1"))
        robot.add_gazebo_element(GazeboElement(reference=None, static=True))

        robot_str = str(robot)
        assert "gazebo_elements=1" in robot_str
