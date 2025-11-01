# LinkForge Development Roadmap

This document outlines the planned features and improvements for LinkForge.

## Version 0.4.0 - Quality & Stability Release

**Target:** Blender Extensions Platform submission
**Status:** In Development
**Timeline:** 2-3 weeks

### Goals

Prepare LinkForge for production use with improved quality, stability, and professional visualizations suitable for Blender Extensions Platform submission.

### Features

#### 1. RViz-Style Joint Axis Visualization ⭐

**Priority:** HIGH
**Status:** ✅ Complete
**Effort:** 1-2 days

Replace plain joint axis visualization with professional RGB-colored axes matching RViz convention:

- Red = X axis
- Green = Y axis
- Blue = Z axis

**Implementation:**

- Create custom 3D gizmo/overlay for joint empties using Blender's `gpu` module
- Add toggle in UI to show/hide axes
- Make axis length configurable
- Render axes with proper depth testing

**Files:**

- New: `linkforge/blender/utils/joint_gizmos.py`
- Update: `linkforge/blender/handlers.py`
- Update: `linkforge/blender/panels/joint_panel.py`

**Tests:**

- Visual verification in Blender
- Unit tests for gizmo data generation

---

#### 2. Mesh Inertia Calculation Improvement

**Priority:** HIGH
**Status:** ✅ Complete
**Effort:** 2-3 days

Replace bounding box approximation with accurate mesh inertia tensor calculation.

**Implementation:**

- Add triangle-based or voxelization-based mesh inertia calculation
- Integrate into physics calculation pipeline
- Remove UI warnings about mesh approximation
- Maintain backward compatibility for primitive shapes

**Files:**

- Update: `linkforge/core/physics/inertia.py` - Add `calculate_mesh_inertia()`
- Update: `linkforge/blender/utils/converters.py` - Use accurate mesh inertia
- Update: `linkforge/blender/panels/link_panel.py` - Remove warning

**Tests:**

- `tests/core/test_inertia.py` - Add mesh inertia tests
- Verify against known mesh inertia values
- Integration tests with sample meshes

---

#### 3. Blender Layer Test Coverage

**Priority:** MEDIUM
**Status:** ✅ Complete
**Effort:** 3-4 days

Increase test coverage from 0% to 60%+ on Blender integration layer.

**Implementation:**

- ✅ Mock `bpy` module for unit testing
- ✅ Test property groups (LinkProps, JointProps)
- ✅ Test converters (Blender ↔ Core model conversion) - 38 tests, 47% coverage
- Operators/Panels require Blender runtime environment for full testing

**New Files:**

- ✅ `tests/blender/__init__.py`
- ✅ `tests/blender/test_converters.py` - 38 comprehensive converter tests
- ✅ `tests/blender/test_joint_gizmos_unit.py` - 6 gizmo tests
- ✅ `tests/blender/conftest.py` - Comprehensive bpy/mathutils mocks

**Results:**

- Overall test coverage increased from 23% → 52%
- Blender layer converters: 47% coverage (60+ functions tested)
- All 252 tests passing
- Code quality: Ruff + formatting checks pass

---

#### 4. Advanced URDF Elements

**Priority:** MEDIUM
**Status:** ✅ Core complete, parser/generator complete, Blender integration pending
**Effort:** 5-7 days (3 days completed)

Add support for advanced URDF/Gazebo elements for simulation and control.

**Implementation - Core Layer (✅ Complete):**

- ✅ Sensor models (Camera, LIDAR, IMU, GPS) - `linkforge/core/models/sensor.py`
- ✅ Transmission models (Simple, Differential, Custom) - `linkforge/core/models/transmission.py`
- ✅ Gazebo plugin models and factory functions - `linkforge/core/models/gazebo.py`
- ✅ Extended Robot model with sensors, transmissions, gazebo_elements
- ✅ Comprehensive unit tests (109 tests, 97-100% coverage)

**Implementation - Parser/Generator (✅ Complete):**

- ✅ URDF parser: Parse `<transmission>` elements
- ✅ URDF parser: Parse `<gazebo>` extension tags (robot/link/joint level)
- ✅ URDF parser: Parse Gazebo plugins with parameters
- ✅ URDF generator: Export transmissions to URDF
- ✅ URDF generator: Export Gazebo elements and plugins
- ✅ Parser coverage: 66% (up from 7%)
- ✅ Generator coverage: 50% (up from 17%)

**Sensor Support:**

- Camera (with configurable FOV, resolution, distortion)
- Depth camera
- LIDAR/laser scanner (2D and 3D)
- IMU (with noise models)
- GPS
- Generic sensor type for custom sensors

**Transmission Support:**

- Simple transmission (1 joint, 1 actuator)
- Differential transmission (2 joints, 2 actuators)
- Four-bar linkage transmission
- Custom transmission types
- Hardware interface specifications (position, velocity, effort)
- Mechanical reduction and offset parameters

**Gazebo Plugin Support:**

- Robot-level, link-level, and joint-level plugins
- Pre-configured factory functions for common plugins:
  - Differential drive controller
  - Joint state publisher
  - Camera sensor
  - IMU sensor
  - LIDAR sensor
- Custom Gazebo properties (materials, friction, damping)

**Blender Integration (✅ Complete):**

- ✅ Blender PropertyGroups for sensors (SensorProps)
- ✅ Blender PropertyGroups for transmissions (TransmissionProps)
- ✅ Converters: Blender ↔ Core models (blender_sensor_to_core, blender_transmission_to_core)
- ✅ Operators: Add/edit/delete sensors and transmissions
- ✅ UI panels for sensors and transmissions in LinkForge tab
- ✅ Integration with URDF importer/exporter via scene_to_robot()
- ⏳ Integration tests for round-trip workflow with new elements (optional)

**Files Created/Modified:**

- **Core Models:**
  - `linkforge/core/models/sensor.py` (139 lines, 97% coverage)
  - `linkforge/core/models/transmission.py` (88 lines, 100% coverage)
  - `linkforge/core/models/gazebo.py` (48 lines, 100% coverage)
  - `linkforge/core/models/robot.py` - Added sensors, transmissions, gazebo_elements

- **Parser/Generator:**
  - `linkforge/core/parsers/urdf_parser.py` - Added 195 lines for parsing transmissions & Gazebo
  - `linkforge/core/generators/urdf.py` - Added 133 lines for generating transmissions & Gazebo

- **Blender PropertyGroups:**
  - `linkforge/blender/properties/sensor_props.py` (72 lines) - Sensor configuration UI
  - `linkforge/blender/properties/transmission_props.py` (62 lines) - Transmission configuration UI

- **Blender Operators:**
  - `linkforge/blender/operators/sensor_ops.py` (75 lines) - Create/delete sensor operations
  - `linkforge/blender/operators/transmission_ops.py` (71 lines) - Create/delete transmission operations

- **Blender Panels:**
  - `linkforge/blender/panels/sensor_panel.py` (82 lines) - Sensor UI panel in LinkForge tab
  - `linkforge/blender/panels/transmission_panel.py` (66 lines) - Transmission UI panel in LinkForge tab

- **Converters:**
  - `linkforge/blender/utils/converters.py` - Added sensor & transmission conversion functions (+230 lines)

- **Tests:**
  - `tests/core/test_sensor.py` (33 tests)
  - `tests/core/test_transmission.py` (24 tests)
  - `tests/core/test_gazebo.py` (19 tests)
  - `tests/core/test_advanced_urdf_roundtrip.py` (6 roundtrip tests)
  - Extended `tests/core/test_robot.py` with 33 additional tests
  - Total: **115 new tests**, all 350 tests passing

**Benefits:**

- Full simulation support for Gazebo
- ros2_control integration via transmissions
- Sensor simulation for cameras, LIDARs, IMUs
- Professional robot descriptions ready for ROS2/Gazebo

---

#### 5. Workflow Enhancements

- Advanced XACRO GUI (macros, properties, includes)
- Preset library (joint configs, materials)
- Batch operations (edit multiple joints)
- Undo/redo for LinkForge operations
- Robot templates (common structures)

---

#### 6. Cross-Platform Testing

**Priority:** MEDIUM
**Status:** Planned
**Effort:** 1-2 days

Verify LinkForge works correctly on Windows, Linux, and macOS.

**Implementation:**

- Set up GitHub Actions CI matrix for multi-platform testing
- Test on: Windows 11, Ubuntu 24.04 LTS, macOS 13+
- Document platform-specific installation steps
- Fix any path handling or platform-specific bugs

**Files:**

- New: `.github/workflows/test-multiplatform.yml`
- Update: `README.md` - Platform-specific notes
- Update: Build scripts for cross-platform compatibility

**Deliverables:**

- All tests pass on all three platforms
- Platform compatibility documented

---

#### 7. Documentation & Polish

**Priority:** MEDIUM
**Status:** Planned
**Effort:** 2-3 days

Professional documentation and examples for Blender Extensions submission.

**Documentation:**

- User guide with screenshots (workflow walkthrough)
- 2-3 minute demo video
- Installation troubleshooting guide
- API documentation for developers

**UI Polish:**

- Review and improve all tooltips
- Consistent terminology across panels
- Add "About" panel with version/license info
- Error messages more user-friendly

**Examples:**

- Add 2-3 new example robots (e.g., gripper, wheeled robot, drone)
- Improve existing examples with better documentation
- Add validation test cases

**Files:**

- New: `docs/USER_GUIDE.md`
- New: `docs/TROUBLESHOOTING.md`
- New: `docs/API.md`
- Update: `examples/` - New robot models
- Update: All panel tooltips

---

### Success Criteria

- ✅ Joint axes display with RGB colors (RViz-style)
- ✅ Mesh inertia calculated accurately (no warnings)
- ✅ 57% overall test coverage (47% on Blender converters)
- ✅ Advanced URDF elements: Core models complete (sensors, transmissions, Gazebo)
- ⏳ Tests pass on Windows, Linux, macOS
- ✅ All 344 tests passing
- ⏳ Professional documentation with video
- ✅ Code quality: Ruff passes, formatting passes
- ⏳ Ready for Blender Extensions platform submission

**Last Updated:** 2025-11-01
**Current Version:** 0.4.0-dev
