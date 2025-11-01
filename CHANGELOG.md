# Changelog

All notable changes to LinkForge will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.4.0] - Unreleased

### Status: In Development

**Goal:** Quality & Stability release for Blender Extensions Platform submission

### Planned Features

#### Added
- **RViz-Style Joint Axis Visualization**: RGB colored axes (Red=X, Green=Y, Blue=Z)
- **Accurate Mesh Inertia Calculation**: Replace bounding box approximation
- **Blender Layer Tests**: 60%+ coverage on integration layer
- **Cross-Platform Support**: Verified on Windows, Linux, macOS
- **Professional Documentation**: User guide, video tutorials, API docs

#### Changed
- Improved UI tooltips and error messages
- Enhanced example robots with better documentation

#### Fixed
- Platform-specific path handling issues
- Mesh inertia warnings removed

### Technical Details
- **Target test coverage**: 210+ tests, 65%+ overall coverage
- **Platform testing**: GitHub Actions CI for Windows/Linux/macOS
- **Code quality**: Ruff + mypy compliance maintained

---

## [0.3.0] - 2025-11-01

### Added

#### Complete XACRO Support

**Phase 1: Detection & Conversion**
- **XACRO File Detection**: Automatically detects XACRO files and provides helpful error messages
- **Standalone Converter Tool**: `tools/convert_xacro.py` for converting XACRO to URDF without ROS
- Detects `.xacro` extensions, `xmlns:xacro` namespace, XACRO elements, and variable substitutions
- Three conversion options: ROS xacro command, standalone converter, or manual xacrodoc

**Phase 2: Native XACRO Import (NEW!) - "Just Works" Approach**
- **Unified "Import Robot" Button**: Single button automatically detects URDF or XACRO files
- **Smart Format Detection**: Auto-detects file format from extension (`.urdf`, `.xacro`, `.urdf.xacro`)
- **Bundled xacrodoc Dependency**: XACRO support built-in, no installation required! 🎉
- **Zero Configuration**: Works immediately after installing the extension
- **parse_urdf_string()** function for parsing URDF from memory
- **Enhanced Import Operator**: `LINKFORGE_OT_import_urdf` now handles both formats seamlessly

#### Documentation
- **README.md**: Updated "Working with XACRO Files" section with native import instructions
- **XACRO_SUPPORT.md**: Technical rationale and implementation details
- **tools/README.md**: Detailed converter documentation

#### Test Coverage
- **Phase 1 Tests**: 7 tests for XACRO detection (in `test_urdf_parser.py`)
- **Phase 2 Tests**: 8 new tests for string parsing (in `test_urdf_string_parser.py`)
- **Total**: 201 tests passing (up from 193)
- **Coverage**: 99% for urdf_parser.py

### Changed
- **Build System**: Migrated from setuptools to Hatchling (PEP 621)
- **Formatting**: Unified on Ruff for both linting and formatting (removed Black)
- **Package Manager**: Recommended UV instead of pip (10-100x faster)

### Technical Details
- **201 tests passing** (8 new tests for Phase 2)
- **Ruff**: 100% compliant (linting + formatting)
- **Coverage**: 99% for urdf_parser.py
- **Bundled dependencies**: xacrodoc + rospkg included in extension package
- **Package size increase**: ~310KB (negligible for better UX)

---

## [0.2.0] - 2025-10-31

### Added

#### UI/UX Improvements
- **Panel State Persistence Fix**: Panels now correctly show "No object selected" when nothing is selected
- **Create Joint at Selection**: New operator to create joints at selected link's location and orientation
- **Kinematic Tree Default OFF**: Show Structure feature now defaults to OFF for cleaner UI

#### Architecture Improvements
- **Link Architecture Refactor** (MAJOR): Link properties now stored on visual mesh objects instead of hidden empties
  - More intuitive selection behavior
  - No more hidden objects cluttering the scene
  - Direct property access on visible meshes

#### New Examples
- **Humanoid Torso**: 13-link upper body robot with 12 revolute joints
- **Quadruped Leg**: 8-link robot leg with 6 joints demonstrating leg kinematics

### Fixed

#### Critical Import Fixes
- **Joint Axis Import**: Properly detects standard axes (X/Y/Z) and sets custom axis components
- **Joint Dynamics Import**: Correctly sets `dynamics_damping` and `dynamics_friction` properties
- **Wheel Orientation**: Fixed example files with correct RPY values and inertia tensors

#### Blender Integration
- Fixed "Remove Joint" operator to actually DELETE the joint Empty
- Removed redundant registration messages on startup
- Improved console feedback during export

### Changed
- Link properties moved from hidden empties to visual mesh objects
- "Remove Joint" now deletes instead of just unmarking
- Updated examples with correct orientations and inertia

---

## [0.1.0] - 2025-01-XX

### Added
- Initial release of LinkForge
- Basic URDF export functionality
- Support for all URDF joint types (revolute, continuous, prismatic, fixed, floating, planar)
- Support for all geometry types (box, cylinder, sphere, capsule, mesh)
- Material and visual properties export
- Collision geometry support
- Inertial properties calculation
- Basic XACRO export with namespace support
- Blender panel UI for marking links and creating joints
- Robot validation system
- Mesh export (STL, OBJ, DAE)
- URDF import capability
- Mass and inertia calculations for primitive geometries
- Documentation and examples

### Known Limitations
- Mesh inertia calculation uses bounding box approximation
- Cross-platform testing limited to macOS (Windows and Linux not yet tested)
- No GUI for advanced XACRO options (controlled via code)

---

## Release Philosophy

LinkForge follows semantic versioning:
- **MAJOR**: Breaking API changes
- **MINOR**: New features, backward compatible
- **PATCH**: Bug fixes, backward compatible

Each release is thoroughly tested with:
- Unit tests for core functionality
- Integration tests for URDF/XACRO generation
- Manual testing in Blender
- Code quality checks (linting, formatting, type checking)
