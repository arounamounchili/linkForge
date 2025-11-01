"""Tests for preset system."""

from __future__ import annotations

from pathlib import Path
from tempfile import TemporaryDirectory

import pytest

from linkforge.core.presets import JointPreset, MaterialPreset, PresetManager, SensorPreset


class TestJointPreset:
    """Tests for JointPreset model."""

    def test_default_joint_preset(self):
        """Test creating a joint preset with defaults."""
        preset = JointPreset(name="test_joint")
        assert preset.name == "test_joint"
        assert preset.joint_type == "REVOLUTE"
        assert preset.use_limits is True

    def test_custom_joint_preset(self):
        """Test creating a joint preset with custom values."""
        preset = JointPreset(
            name="wheel_joint",
            description="Continuous wheel joint",
            joint_type="CONTINUOUS",
            use_limits=False,
            limit_effort=20.0,
        )
        assert preset.name == "wheel_joint"
        assert preset.joint_type == "CONTINUOUS"
        assert preset.use_limits is False
        assert preset.limit_effort == 20.0

    def test_joint_preset_to_dict(self):
        """Test converting preset to dictionary."""
        preset = JointPreset(name="test", joint_type="FIXED")
        data = preset.to_dict()
        assert data["name"] == "test"
        assert data["joint_type"] == "FIXED"
        assert "limit_lower" in data

    def test_joint_preset_from_dict(self):
        """Test creating preset from dictionary."""
        data = {
            "name": "test",
            "description": "Test preset",
            "joint_type": "PRISMATIC",
            "axis": "X",
            "use_limits": True,
            "limit_lower": 0.0,
            "limit_upper": 1.0,
            "limit_effort": 100.0,
            "limit_velocity": 0.5,
            "use_dynamics": False,
            "dynamics_damping": 0.0,
            "dynamics_friction": 0.0,
        }
        preset = JointPreset.from_dict(data)
        assert preset.name == "test"
        assert preset.joint_type == "PRISMATIC"
        assert preset.axis == "X"


class TestMaterialPreset:
    """Tests for MaterialPreset model."""

    def test_default_material_preset(self):
        """Test creating a material preset with defaults."""
        preset = MaterialPreset(name="aluminum")
        assert preset.name == "aluminum"
        assert len(preset.color) == 4
        assert preset.color[3] == 1.0  # Alpha

    def test_custom_material_preset(self):
        """Test creating a material preset with custom values."""
        preset = MaterialPreset(
            name="red_plastic",
            description="Bright red plastic",
            color=[0.8, 0.1, 0.1, 1.0],
            material_name="red",
        )
        assert preset.name == "red_plastic"
        assert preset.color[0] == pytest.approx(0.8)
        assert preset.material_name == "red"

    def test_material_preset_to_dict(self):
        """Test converting preset to dictionary."""
        preset = MaterialPreset(name="test", color=[1.0, 0.0, 0.0, 1.0])
        data = preset.to_dict()
        assert data["name"] == "test"
        assert data["color"] == [1.0, 0.0, 0.0, 1.0]

    def test_material_preset_from_dict(self):
        """Test creating preset from dictionary."""
        data = {
            "name": "steel",
            "description": "Gray metallic",
            "color": [0.5, 0.5, 0.5, 1.0],
            "material_name": "steel",
        }
        preset = MaterialPreset.from_dict(data)
        assert preset.name == "steel"
        assert preset.color == [0.5, 0.5, 0.5, 1.0]


class TestSensorPreset:
    """Tests for SensorPreset model."""

    def test_default_sensor_preset(self):
        """Test creating a sensor preset with defaults."""
        preset = SensorPreset(name="default_camera")
        assert preset.name == "default_camera"
        assert preset.sensor_type == "CAMERA"
        assert preset.camera_width == 640

    def test_camera_preset(self):
        """Test creating a camera preset."""
        preset = SensorPreset(
            name="hd_camera",
            description="HD camera",
            sensor_type="CAMERA",
            camera_width=1920,
            camera_height=1080,
        )
        assert preset.camera_width == 1920
        assert preset.camera_height == 1080

    def test_lidar_preset(self):
        """Test creating a LIDAR preset."""
        preset = SensorPreset(
            name="lidar_3d",
            sensor_type="LIDAR",
            lidar_horizontal_samples=1024,
            lidar_vertical_samples=64,
        )
        assert preset.sensor_type == "LIDAR"
        assert preset.lidar_vertical_samples == 64

    def test_sensor_preset_to_dict(self):
        """Test converting preset to dictionary."""
        preset = SensorPreset(name="test", sensor_type="IMU")
        data = preset.to_dict()
        assert data["name"] == "test"
        assert data["sensor_type"] == "IMU"

    def test_sensor_preset_from_dict(self):
        """Test creating preset from dictionary."""
        data = {
            "name": "gps",
            "description": "GPS sensor",
            "sensor_type": "GPS",
            "update_rate": 1.0,
            "camera_horizontal_fov": 1.047,
            "camera_width": 640,
            "camera_height": 480,
            "camera_near_clip": 0.1,
            "camera_far_clip": 100.0,
            "lidar_horizontal_samples": 640,
            "lidar_horizontal_min_angle": -1.570796,
            "lidar_horizontal_max_angle": 1.570796,
            "lidar_vertical_samples": 1,
            "lidar_range_min": 0.1,
            "lidar_range_max": 10.0,
            "imu_gravity_magnitude": 9.80665,
            "use_noise": False,
            "noise_type": "gaussian",
            "noise_mean": 0.0,
            "noise_stddev": 0.0,
        }
        preset = SensorPreset.from_dict(data)
        assert preset.name == "gps"
        assert preset.sensor_type == "GPS"


class TestPresetManager:
    """Tests for PresetManager."""

    def test_create_manager_with_temp_dir(self):
        """Test creating preset manager with temporary directory."""
        with TemporaryDirectory() as tmpdir:
            manager = PresetManager(presets_dir=Path(tmpdir))
            assert manager.presets_dir.exists()
            assert manager.joint_presets_dir.exists()
            assert manager.material_presets_dir.exists()
            assert manager.sensor_presets_dir.exists()

    def test_default_presets_created(self):
        """Test that default presets are created on initialization."""
        with TemporaryDirectory() as tmpdir:
            manager = PresetManager(presets_dir=Path(tmpdir))

            # Check joint presets
            joint_presets = manager.list_joint_presets()
            assert len(joint_presets) > 0
            assert "Fixed Joint" in joint_presets

            # Check material presets
            material_presets = manager.list_material_presets()
            assert len(material_presets) > 0
            assert "Aluminum" in material_presets

            # Check sensor presets
            sensor_presets = manager.list_sensor_presets()
            assert len(sensor_presets) > 0
            assert "Standard IMU" in sensor_presets

    def test_save_and_load_joint_preset(self):
        """Test saving and loading joint preset."""
        with TemporaryDirectory() as tmpdir:
            manager = PresetManager(presets_dir=Path(tmpdir))

            # Create and save preset
            preset = JointPreset(
                name="test_joint",
                description="Test joint preset",
                joint_type="REVOLUTE",
                limit_lower=-1.5,
                limit_upper=1.5,
            )
            manager.save_joint_preset(preset)

            # Load preset
            loaded = manager.load_joint_preset("test_joint")
            assert loaded is not None
            assert loaded.name == "test_joint"
            assert loaded.joint_type == "REVOLUTE"
            assert loaded.limit_lower == pytest.approx(-1.5)

    def test_save_and_load_material_preset(self):
        """Test saving and loading material preset."""
        with TemporaryDirectory() as tmpdir:
            manager = PresetManager(presets_dir=Path(tmpdir))

            # Create and save preset
            preset = MaterialPreset(
                name="test_material", color=[0.5, 0.5, 0.5, 1.0], material_name="gray"
            )
            manager.save_material_preset(preset)

            # Load preset
            loaded = manager.load_material_preset("test_material")
            assert loaded is not None
            assert loaded.name == "test_material"
            assert loaded.color == [0.5, 0.5, 0.5, 1.0]

    def test_save_and_load_sensor_preset(self):
        """Test saving and loading sensor preset."""
        with TemporaryDirectory() as tmpdir:
            manager = PresetManager(presets_dir=Path(tmpdir))

            # Create and save preset
            preset = SensorPreset(
                name="test_sensor", sensor_type="CAMERA", camera_width=1920, camera_height=1080
            )
            manager.save_sensor_preset(preset)

            # Load preset
            loaded = manager.load_sensor_preset("test_sensor")
            assert loaded is not None
            assert loaded.name == "test_sensor"
            assert loaded.camera_width == 1920

    def test_list_presets(self):
        """Test listing presets."""
        with TemporaryDirectory() as tmpdir:
            manager = PresetManager(presets_dir=Path(tmpdir))

            # Save multiple presets
            manager.save_joint_preset(JointPreset(name="joint1"))
            manager.save_joint_preset(JointPreset(name="joint2"))

            # List presets
            presets = manager.list_joint_presets()
            assert "joint1" in presets
            assert "joint2" in presets

    def test_delete_preset(self):
        """Test deleting presets."""
        with TemporaryDirectory() as tmpdir:
            manager = PresetManager(presets_dir=Path(tmpdir))

            # Save and delete
            manager.save_joint_preset(JointPreset(name="to_delete"))
            assert "to_delete" in manager.list_joint_presets()

            result = manager.delete_joint_preset("to_delete")
            assert result is True
            assert "to_delete" not in manager.list_joint_presets()

            # Try to delete non-existent
            result = manager.delete_joint_preset("nonexistent")
            assert result is False

    def test_load_nonexistent_preset(self):
        """Test loading a preset that doesn't exist."""
        with TemporaryDirectory() as tmpdir:
            manager = PresetManager(presets_dir=Path(tmpdir))

            loaded = manager.load_joint_preset("nonexistent")
            assert loaded is None

    def test_preset_persistence(self):
        """Test that presets persist across manager instances."""
        with TemporaryDirectory() as tmpdir:
            preset_dir = Path(tmpdir)

            # Create manager and save preset
            manager1 = PresetManager(presets_dir=preset_dir)
            manager1.save_joint_preset(JointPreset(name="persistent", joint_type="FIXED"))

            # Create new manager instance and load preset
            manager2 = PresetManager(presets_dir=preset_dir)
            loaded = manager2.load_joint_preset("persistent")

            assert loaded is not None
            assert loaded.name == "persistent"
            assert loaded.joint_type == "FIXED"
