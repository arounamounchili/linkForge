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
**Status:** Planned
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
**Status:** Planned
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
**Status:** Planned
**Effort:** 3-4 days

Increase test coverage from 0% to 60%+ on Blender integration layer.

**Implementation:**
- Mock `bpy` module for unit testing
- Test property groups (LinkProps, JointProps)
- Test converters (Blender ↔ Core model conversion)
- Test basic operator functionality with mocked context

**New Files:**
- `tests/blender/__init__.py`
- `tests/blender/test_properties.py`
- `tests/blender/test_converters.py`
- `tests/blender/test_operators.py`
- `tests/blender/conftest.py` (bpy mocks)

**Coverage Target:** 60%+ on `linkforge/blender/` layer

---

#### 4. Cross-Platform Testing
**Priority:** MEDIUM
**Status:** Planned
**Effort:** 1-2 days

Verify LinkForge works correctly on Windows, Linux, and macOS.

**Implementation:**
- Set up GitHub Actions CI matrix for multi-platform testing
- Test on: Windows 11, Ubuntu 22.04 LTS, macOS 13+
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

#### 5. Documentation & Polish
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
- ✅ 60%+ test coverage on Blender layer
- ✅ Tests pass on Windows, Linux, macOS
- ✅ All 210+ tests passing
- ✅ Professional documentation with video
- ✅ Code quality: Ruff passes, mypy passes
- ✅ Ready for Blender Extensions platform submission

---

### Task Order

```
Week 1:
  Day 1-2: RViz-style joint axes (HIGH priority, user-facing)
  Day 3-5: Mesh inertia improvement (HIGH priority, removes limitation)

Week 2:
  Day 1-4: Blender layer test coverage (MEDIUM priority, quality)
  Day 5:   Cross-platform testing setup (MEDIUM priority)

Week 3:
  Day 1-3: Documentation & polish (MEDIUM priority, presentation)
  Day 4-5: Final testing, bug fixes, release prep
```

---

## Version 0.5.0 - Advanced Features (Future)

**Status:** Planning
**Timeline:** TBD

### Potential Features

#### Option A: Advanced URDF Elements
- Sensor support (cameras, LIDARs, IMUs)
- Transmission elements (ros_control/ros2_control)
- Gazebo-specific plugins and tags
- SDF export support

#### Option B: Workflow Enhancements
- Advanced XACRO GUI (macros, properties, includes)
- Preset library (joint configs, materials)
- Batch operations (edit multiple joints)
- Undo/redo for LinkForge operations
- Robot templates (common structures)

#### Option C: CAD Integration
- SolidWorks assembly import
- Fusion 360 integration
- MJCF (MuJoCo) export
- Improved mesh optimization

**Decision:** TBD based on user feedback after v0.4.0 release

---

## Version 1.0.0 - Production Release (Future)

**Status:** Vision
**Timeline:** TBD

### Goals
- Feature-complete for common robotics workflows
- Published on Blender Extensions Platform
- Active user community
- Comprehensive documentation and tutorials
- 80%+ test coverage across all layers
- Multi-language support (if community demand)

---

## Completed Versions

### Version 0.3.0 - XACRO Support ✅
**Released:** 2025-11-01

- Native XACRO import/export
- Bundled xacrodoc dependency
- Unified "Import Robot" button
- Standalone XACRO converter tool
- 201 tests passing, 99% coverage on parser

### Version 0.2.0 - Architecture & Import ✅
**Released:** 2025-10-31

- Link architecture refactor (properties on visual meshes)
- Fixed joint axis/dynamics import
- UI/UX improvements
- New examples (humanoid torso, quadruped leg)

### Version 0.1.0 - Initial Release ✅
**Released:** 2025-01-XX

- Basic URDF export
- All joint types and geometry primitives
- Material and collision support
- Validation system
- Mesh export (STL/DAE)

---

## Contributing

Want to help with the roadmap? Please:
1. Check [GitHub Issues](https://github.com/arounamounchili/linkforge/issues) for open tasks
2. Comment on features you'd like to see
3. Submit PRs targeting the current development version
4. Test pre-release versions and report bugs

---

**Last Updated:** 2025-11-01
**Current Version:** 0.4.0-dev
