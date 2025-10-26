# LinkForge v0.1.0 - Release Notes

## 🎉 First Release - Production Ready!

LinkForge is a professional Blender Extension for converting 3D robot models to URDF/XACRO for ROS2 and Gazebo.

---

## ✅ What's Included

### Core Features

**Import/Export**
- ✅ Import URDF files with full fidelity
- ✅ Export to URDF or XACRO formats
- ✅ Mesh export (OBJ with materials, STL)
- ✅ Material/color preservation
- ✅ Round-trip editing (import → modify → export)

**Robot Modeling**
- ✅ Mark objects as robot links
- ✅ Define joints between links (6 types supported)
- ✅ Auto-calculate inertia tensors
- ✅ Support all URDF geometry types
- ✅ Set joint limits, dynamics, and mimic constraints

**Validation & Visualization**
- ✅ Validate robot structure before export
- ✅ Kinematic tree visualization
- ✅ Joint axes display (RViz-style RGB 3-axis)
- ✅ Real-time property editing

---

## 📦 What's New in v0.1.0

### Import Functionality (New!)
- Complete URDF parser implementation
- Imports all geometry types (Box, Cylinder, Sphere, Capsule, Mesh)
- Preserves materials, colors, inertia, and dynamics
- Creates proper kinematic hierarchy in Blender
- Matches RViz visualization exactly

### UI/UX Improvements
- Clean, production-ready interface
- Only functional features (no unused options)
- Toggleable joint axes visualization
- Kinematic tree display
- Import panel for easy URDF loading

### Examples & Documentation
- 3 example URDFs (arm, gripper, mobile base)
- Complete Quick Start guide
- Examples documentation with learning path
- Round-trip integration tests

---

## 🚀 Supported Features

### Joint Types
- ✅ Revolute (hinge with limits)
- ✅ Continuous (360° rotation)
- ✅ Prismatic (linear slider)
- ✅ Fixed (rigid attachment)
- ✅ Floating (6-DOF)
- ✅ Planar (2D movement)

### Geometry Types
- ✅ Box (rectangular prism)
- ✅ Cylinder
- ✅ Sphere
- ✅ Capsule
- ✅ Mesh (OBJ, STL, DAE)

### Physics Properties
- ✅ Mass
- ✅ Inertia tensor (auto or manual)
- ✅ Center of mass offset
- ✅ Joint limits (position, effort, velocity)
- ✅ Joint dynamics (damping, friction)
- ✅ Mimic constraints

---

## 📊 Testing Status

**Test Coverage:**
- Core library: 83% coverage
- Integration tests: 4/4 passing
- Round-trip import/export: ✅ Verified
- Example URDFs: ✅ All valid

**Platforms Tested:**
- ✅ macOS (Blender 4.5)
- ✅ Python 3.11+

---

## 📚 Documentation

**Included Docs:**
- `README.md` - Project overview
- `QUICKSTART.md` - Get started in 5 minutes
- `examples/README.md` - Example documentation
- `RELEASE_NOTES.md` - This file

**Examples:**
- `simple_arm.urdf` - 2-DOF robot arm
- `simple_gripper.urdf` - Parallel gripper with mimic
- `mobile_base.urdf` - Wheeled mobile robot

---

## 🎯 Use Cases

**Perfect For:**
- ROS2 developers creating robot descriptions
- Robotics engineers prototyping new designs
- Researchers testing robot configurations
- Students learning URDF/ROS

**Typical Workflow:**
1. Model robot in Blender (or import CAD)
2. Mark links and define joints with LinkForge
3. Export to URDF
4. Test in RViz/Gazebo
5. Iterate and refine

---

## 🔧 System Requirements

**Required:**
- Blender 4.2 or newer
- Python 3.11+ (bundled with Blender)

**Optional:**
- ROS2 (Humble, Iron, or Rolling) for testing
- RViz2 for visualization
- Gazebo for simulation

---

## 📦 Installation

**Method 1: Drag and Drop**
1. Download `linkforge-0.1.0.zip`
2. Drag into Blender window
3. Done!

**Method 2: Extensions Manager**
1. Edit → Preferences → Get Extensions
2. Dropdown → Install from Disk
3. Select `linkforge-0.1.0.zip`

---

## 🐛 Known Limitations

- Visual gizmos not implemented (planned for v0.2.0)
- Convex hull collision not implemented (planned for v0.2.0)
- Windows/Linux not extensively tested (should work)
- XACRO macros are basic (advanced macros planned for future)

---

## 🔮 Future Plans (v0.2.0+)

**Planned Features:**
- Interactive joint manipulation gizmos
- Convex hull collision generation
- Robot templates (common arm/base types)
- CLI tool for batch export
- Advanced XACRO macro generation
- MoveIt config generation

---

## 📞 Support

**Bug Reports & Feature Requests:**
- GitHub Issues: [github.com/arounamounchili/linkforge/issues](https://github.com/arounamounchili/linkforge/issues)

**Questions & Discussion:**
- GitHub Discussions: [github.com/arounamounchili/linkforge/discussions](https://github.com/arounamounchili/linkforge/discussions)
- ROS Discourse: Tag with `linkforge`

**Documentation:**
- Quick Start: `QUICKSTART.md`
- Examples: `examples/README.md`
- README: `README.md`

---

## 📝 License

GPL-3.0-or-later

LinkForge is licensed under the GNU General Public License v3.0 or later to comply with Blender Extensions platform requirements.

---

## 🙏 Acknowledgments

Built for the robotics community by robotics engineers. Inspired by:
- MoveIt Setup Assistant
- ROS ecosystem tools
- Blender's amazing extensibility

Special thanks to all contributors and testers!

---

## 🎊 Get Started

```bash
# 1. Install extension in Blender
# 2. Press N in 3D Viewport → LinkForge tab
# 3. Import example: examples/simple_arm.urdf
# 4. Explore the UI
# 5. Read QUICKSTART.md for detailed guide
```

**Welcome to LinkForge - Get from 3D model to working URDF in minutes!** 🤖

---

**Version:** 0.1.0
**Release Date:** 2025
**Build:** Production
**Status:** ✅ Stable
