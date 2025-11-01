# LinkForge

**Professional Blender Extension for converting 3D robot models to URDF/XACRO for ROS2 and Gazebo**

[![CI](https://github.com/arounamounchili/linkforge/workflows/CI/badge.svg)](https://github.com/arounamounchili/linkforge/actions)
[![License: GPL-3.0](https://img.shields.io/badge/License-GPL%203.0-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Python 3.11+](https://img.shields.io/badge/python-3.11+-blue.svg)](https://www.python.org/downloads/)
[![Blender 4.2+](https://img.shields.io/badge/blender-4.2+-orange.svg)](https://www.blender.org/)
[![Extensions](https://img.shields.io/badge/Blender-Extensions-blue.svg)](https://extensions.blender.org)

## Overview

LinkForge enables robotics engineers to convert 3D robot models into valid URDF/XACRO files in minutes, not hours. Designed for ROS users who need a professional workflow for creating robot descriptions from Blender models.

## Features

### Import/Export
- **Import URDF/XACRO files** - automatic format detection
- **Export to URDF or XACRO** with meshes included
- **Round-trip editing**: Import → Modify → Export workflow
- **Material preservation**: Colors and materials maintained

### Robot Modeling
- **Mark objects as robot links** with automatic inertia calculation
- **Define joints** between links (revolute, prismatic, fixed, continuous, floating, planar)
- **Support for all URDF geometry types**: box, cylinder, sphere, capsule, and mesh
- **Auto-calculate inertia tensors** from object geometry and mass
- **Joint limits, dynamics, and mimic constraints**

### Visualization & Validation
- **Visual feedback**: joint axes (RViz-style RGB 3-axis), kinematic tree
- **Validation**: catch errors before export
- **Non-destructive workflow**: preserves existing Blender projects
- **Fully tested**: comprehensive unit and integration tests (83% coverage)

## Installation

> **Note**: LinkForge is a Blender Extension (not a legacy add-on). It requires **Blender 4.2 or newer** to access the new Extensions platform.

### Method 1: From Blender Extensions Platform (Recommended)

**Coming Soon!** LinkForge will be available on https://extensions.blender.org

1. Open Blender 4.2+
2. Go to **Edit → Preferences → Get Extensions**
3. Search for **"LinkForge"**
4. Click **Install**

### Method 2: Install from GitHub Release

1. Download the latest `.zip` from [Releases](https://github.com/arounamounchili/linkforge/releases)
2. Open Blender 4.2+
3. Go to **Edit → Preferences → Get Extensions**
4. Click the dropdown menu (⌄) → **Install from Disk**
5. Select the downloaded `linkforge-{version}.zip`
6. Enable the extension

### Method 3: Drag and Drop

1. Download the latest `.zip` from [Releases](https://github.com/arounamounchili/linkforge/releases)
2. Open Blender 4.2+
3. **Drag and drop** the `.zip` file directly into Blender
4. The extension will install automatically

### From Source (Development)

```bash
# Clone repository
git clone https://github.com/arounamounchili/linkforge.git
cd linkforge

# Install UV (if not already installed)
# macOS/Linux
curl -LsSf https://astral.sh/uv/install.sh | sh

# Windows
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"

# Create virtual environment and install dependencies
uv venv
source .venv/bin/activate  # Windows: .venv\Scripts\activate
uv pip install -e ".[dev]"

# Build the extension package
python build_extension.py

# Install in Blender
# 1. Open Blender 4.2+
# 2. Edit → Preferences → Get Extensions
# 3. Dropdown (⌄) → Install from Disk
# 4. Select dist/linkforge-{version}.zip
```

**Legacy pip workflow** (if you prefer pip over uv):
```bash
python3 -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate
pip install -e ".[dev]"
```

**Alternative - Symlink for Development** (Advanced):

```bash
# Copy manifest to Blender extensions directory
cp blender_manifest.toml ~/.config/blender/4.2/extensions/user_default/

# Symlink the package
# macOS/Linux
ln -s /path/to/linkforge/linkforge \
  ~/.config/blender/4.2/extensions/user_default/linkforge

# Windows
mklink /D "%APPDATA%\Blender Foundation\Blender\4.2\extensions\user_default\linkforge" \
  "C:\path\to\linkforge\linkforge"
```

## Quick Start

### Option 1: Import Existing URDF/XACRO

1. Press `N` → **LinkForge** tab
2. Click **"Import Robot"**
3. Select your `.urdf`, `.xacro`, or `.urdf.xacro` file
4. Robot appears with full hierarchy, materials, and properties
5. Modify as needed and re-export

**XACRO support is built-in!** No conversion needed - just select your XACRO file directly.

### Option 2: Create from Scratch

### 1. Mark Links

Select a Blender object and mark it as a robot link:

1. Open the **3D Viewport** sidebar (press `N`)
2. Navigate to the **LinkForge** tab
3. With your base object selected, click **"Mark as Link"**
4. Set the link name (e.g., `base_link`)
5. Set mass and enable **Auto-Calculate Inertia**
6. Repeat for all robot links

### 2. Create Joints

Define joints between links:

1. In the **Joints** panel, click **"Create Joint"**
2. Select joint type (Revolute, Prismatic, Fixed, etc.)
3. Choose parent and child links from dropdowns
4. Set joint axis (X, Y, or Z)
5. Define joint limits (for revolute/prismatic)
6. Position the joint Empty at the desired origin

### 3. Validate

Check your robot for errors:

1. Click **"Validate"** in the Robot panel
2. Review any warnings or errors
3. Fix issues before export

### 4. Export

Export to URDF or XACRO:

1. In the **Export** panel, choose format (URDF or XACRO)
2. Enable **"Export Meshes"** to include STL/DAE files
3. Choose output directory
4. Click **"Export URDF/XACRO"**

Your robot is now ready for ROS2, RViz, and Gazebo!

## XACRO Support

LinkForge provides **native XACRO support** out of the box - no installation or conversion required!

**Import**: Click "Import Robot" and select any `.urdf`, `.xacro`, or `.urdf.xacro` file
**Export**: Choose "XACRO" format with automatic properties and macro generation

For advanced command-line conversion, use the included tool: `python tools/convert_xacro.py robot.urdf.xacro`

## Documentation

- **[CHANGELOG.md](CHANGELOG.md)** - Version history and changes
- **[examples/](examples/)** - Example URDF files:
  - `robot_arm.urdf` - Robot arm with revolute joints
  - `mobile_robot.urdf` - Mobile base with continuous and fixed joints

## Target Audience

**Robotics engineers** familiar with ROS/URDF who want a faster workflow than hand-writing XML files. No Blender expertise required - the UI is designed to feel familiar to ROS users.

## Requirements

- **Blender**: 4.2 or newer
- **Python**: 3.11+ (bundled with Blender)
- **ROS2** (optional, for testing exported URDFs): Humble, Iron, or Rolling

## Development

### Running Tests

```bash
# Install development dependencies
pip install -e ".[dev]"

# Run all tests
pytest

# Run with coverage
pytest --cov=linkforge --cov-report=html

# Run specific test file
pytest tests/core/test_inertia.py
```

### Code Quality

```bash
# Format code
ruff format linkforge tests

# Lint
ruff check linkforge tests

# Type checking
mypy linkforge
```

## Architecture

LinkForge uses a **three-layer architecture**:

1. **Core Layer**: Pure Python dataclasses for robot models, URDF generation, inertia calculations (100% Blender-independent, fully testable)
2. **Adapter Layer**: Bridges Blender's property system with core models
3. **UI Layer**: Panels, operators, and visual gizmos for user interaction

This design allows the core logic to be thoroughly tested without Blender and potentially reused in other tools.

## Contributing

Contributions are welcome! Please:
1. Fork the repository
2. Create a feature branch from `develop`
3. Run tests and quality checks before submitting
4. Open a pull request targeting `develop`

## License

GPL-3.0-or-later - see [LICENSE](LICENSE) for details.

LinkForge is licensed under the GNU General Public License v3.0 or later to comply with Blender Extensions platform requirements. This ensures the extension can be published on https://extensions.blender.org.

## Support

- **Issues**: [GitHub Issues](https://github.com/arounamounchili/linkforge/issues)
- **Discussions**: [GitHub Discussions](https://github.com/arounamounchili/linkforge/discussions)
- **ROS Discourse**: Tag posts with `linkforge`

## Acknowledgments

Built for the robotics community by robotics engineers. Inspired by MoveIt Setup Assistant and other ROS tooling.

---

**Get from 3D model to working URDF in 5 minutes. Start building robots faster with LinkForge.**
