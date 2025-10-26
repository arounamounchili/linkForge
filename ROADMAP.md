# LinkForge Roadmap

This document outlines the planned features and improvements for LinkForge.

## v0.1.0 - Initial Release ✅ **RELEASED**

**Release Date**: October 2025
**Status**: Stable, Production Ready

### Features
- URDF/XACRO export and import
- Automatic physics calculations (inertia tensors)
- Robot structure validation
- Support for 6 joint types and 5 geometry types
- Three-layer architecture (core/adapter/UI)
- 83% test coverage on core library

---

## v0.2.0 - Robustness & Usability 🔄 **IN PLANNING**

**Target Date**: Q1 2026
**Theme**: Making LinkForge more robust and easier to use

### Planned Features

#### 1. Robot Templates Library ⭐ **HIGH PRIORITY**
**Goal**: Provide pre-built robot templates for quick starting

**Implementation**:
- Create templates module (`linkforge/templates/`)
- Pre-built robots:
  - 2-DOF robot arm
  - 4-wheel differential drive mobile base
  - Parallel gripper
  - 6-DOF manipulator arm
- UI panel for template selection
- One-click template instantiation in Blender

**User Benefit**: New users can start with working examples and modify them

**Estimated Effort**: 2-3 weeks

---

#### 2. Enhanced Validation UI ⭐ **HIGH PRIORITY**
**Goal**: Better error reporting and user guidance

**Implementation**:
- Validation result panel with expandable error list
- Specific fix suggestions (e.g., "Link 'base' has no parent joint")
- Visual highlighting of problem objects in viewport
- Warning system (not just errors):
  - Low mass links
  - Missing collision geometry
  - Unusual inertia values
- "Fix" buttons for common issues

**User Benefit**: Users understand and fix problems faster

**Estimated Effort**: 2-3 weeks

---

#### 3. Type Safety Cleanup ⭐ **MEDIUM PRIORITY**
**Goal**: Achieve 100% mypy compliance

**Implementation**:
- Fix all mypy errors in core library:
  - `geometry.py` - Add missing type annotations
  - `inertia.py` - Fix tuple/Vector3 type issues
  - `robot.py` - Annotate dict types
  - `urdf.py` - Fix SubElement type hints
- Add type annotations to Blender integration code
- Enable strict mypy in CI (remove `continue-on-error`)

**Developer Benefit**: Better IDE support, catch bugs at type-check time

**Estimated Effort**: 1-2 weeks

---

#### 4. Cross-Platform Testing ⭐ **MEDIUM PRIORITY**
**Goal**: Ensure LinkForge works on all platforms

**Implementation**:
- Test on Windows 11
- Test on Linux (Ubuntu 22.04)
- Test on macOS (Intel and Apple Silicon)
- Test Blender versions: 4.2, 4.3, 4.4, 4.5
- Document platform-specific issues
- Add platform test matrix to CI (if feasible)

**User Benefit**: Confidence it works everywhere

**Estimated Effort**: 1 week

---

### Technical Debt

Items identified but deferred:
- Blender UI test coverage (requires headless Blender testing setup)
- API documentation generation (Sphinx)
- Performance profiling for large robots (100+ links)

---

## v0.3.0 - Advanced Visualization 🔮 **FUTURE**

**Target Date**: Q2 2026
**Theme**: Interactive manipulation and visualization

### Planned Features

#### 1. Interactive Joint Manipulation Gizmos
- 3D gizmos for rotating/sliding joints
- Real-time joint position updates
- Joint limit visualization
- Snap to limit feature

#### 2. Convex Hull Collision Generation
- Auto-generate convex hull collision meshes
- Simplification options (vertex count reduction)
- Preview before applying

#### 3. Advanced XACRO Features
- Parameterized robot macros
- Property inheritance
- Include file management

---

## v0.4.0 - Automation & Integration 🔮 **FUTURE**

**Target Date**: Q3 2026
**Theme**: Workflow automation and ecosystem integration

### Planned Features

#### 1. CLI Tool
- Batch URDF export without GUI
- Automated builds in CI/CD
- Scripting support

#### 2. MoveIt Configuration Generation
- Generate SRDF files
- Planning group definitions
- Collision matrix generation

#### 3. Gazebo World Export
- Export robot with environment
- Spawn scripts generation
- Plugin configuration

---

## Community Requests 💬

Features requested by users will be tracked here as they come in via:
- GitHub Issues
- GitHub Discussions
- ROS Discourse

**Current requests**: _None yet - v0.1.0 just released!_

---

## Contributing to the Roadmap

Have ideas for LinkForge? We'd love to hear them!

1. **Check existing issues**: https://github.com/arounamounchili/linkforge/issues
2. **Start a discussion**: https://github.com/arounamounchili/linkforge/discussions
3. **Propose features**: Open an issue with the "enhancement" label

**Contribution welcome!** If you'd like to implement a roadmap feature, comment on the relevant issue or discussion.

---

## Version History

- **v0.1.0** - October 2025 - Initial release
- **v0.2.0** - Q1 2026 (planned) - Robustness & Usability
- **v0.3.0** - Q2 2026 (planned) - Advanced Visualization
- **v0.4.0** - Q3 2026 (planned) - Automation & Integration

---

**Last Updated**: October 2025
**Maintainer**: Arouna Patouossa Mounchili
