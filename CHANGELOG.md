# Changelog

All notable changes to LinkForge will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.2.0] - 2025-01-XX

### Added

#### Advanced XACRO Features
- **Material Property Extraction**: Automatically extracts unique materials as XACRO properties with `${variable}` references
- **Dimension Property Extraction**: Detects common dimensions (radii, lengths) used across multiple links
- **Macro Generation**: Automatically generates XACRO macros for repeated patterns (e.g., identical wheels)
- **File Splitting**: Split XACRO output into organized files (materials.xacro, macros.xacro, robot.xacro)
- Advanced XACRO mode can be toggled on/off with backward compatibility

#### Enhanced Validation System
- Comprehensive robot validation with detailed error messages
- Warning system for non-blocking issues (low mass, missing collision, unusual inertia)
- Auto-fix suggestions for common problems
- Validation panel in Blender UI showing:
  - Validation status (✅ Valid, ⚠️ Warnings, ❌ Errors)
  - Expandable list of issues with severity indicators
  - Object selection from validation results
  - Suggested fixes for each issue

#### Test Coverage Improvements
- Added comprehensive unit tests for XACRO generator (19 tests, 99% coverage)
- Added complete test suite for mass calculations (35 tests, 100% coverage)
- Added full URDF parser tests (43 tests, 100% coverage)
- Enhanced inertia calculation tests (8 new tests, 100% coverage)
- Overall test count increased from 100 to 186 tests
- Core module coverage improved from 24% to 38%

#### Type Safety Improvements
- Added type annotations to core generators and utilities
- Implemented TYPE_CHECKING for Blender API imports
- Reduced mypy errors from 77 to 70
- Fixed multiple type mismatches in core modules

### Fixed

#### Origin and Transform Calculations
- Fixed link origin export - root link now at origin following URDF convention
- Fixed visual and collision origin calculations to be relative to link frames
- Fixed joint origin calculations using proper relative transforms
- Fixed frozen dataclass error when updating joint origins
- Correctly export object transforms to visual/collision origins

#### XACRO and URDF Generation
- Fixed generator variable type mismatch in export operator
- Fixed mesh inertia calculation TypeError (line 173 in inertia.py)
- Improved URDF coordinate system handling

#### User Experience
- Fixed joint enum property warnings in Blender
- Implemented bidirectional link name synchronization
- Added console messages for better export feedback
- Multiple UX improvements based on user feedback

### Changed

- Updated mypy CI configuration comment to reflect current status (70 errors, advisory mode)
- Improved linting compliance across all modules
- Applied black formatting to all Python files
- Organized imports following ruff standards

### Technical Details

#### Code Quality
- All 186 tests passing
- Ruff linting: 100% compliant
- Black formatting: 100% compliant
- Core modules with 100% test coverage:
  - `parsers/urdf_parser.py`
  - `physics/mass.py`
  - `physics/inertia.py`
- High coverage modules (90%+):
  - `generators/xacro.py` (99%)
  - `models/geometry.py` (97%)
  - `validation/validator.py` (96%)
  - `validation/result.py` (95%)

#### Commits
This release includes 19 commits with comprehensive improvements across the codebase.

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
