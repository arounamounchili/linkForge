# LinkForge Tools

Utility scripts for working with LinkForge and robot description files.

## XACRO to URDF Converter

`convert_xacro.py` - Standalone command-line tool for converting XACRO files to URDF.

> **Note**: As of LinkForge v0.3.0, **XACRO import is built-in**! You can directly import XACRO files in Blender without conversion. This tool is provided for command-line workflows and automation.

### Quick Start

**No installation needed** - uses LinkForge's bundled dependencies:

```bash
# Convert XACRO to URDF
python tools/convert_xacro.py robot.urdf.xacro

# Specify output file
python tools/convert_xacro.py robot.urdf.xacro -o my_robot.urdf

# Compact output (no pretty printing)
python tools/convert_xacro.py robot.urdf.xacro --no-pretty
```

### When to Use This Tool

**Use the converter for:**
- ✅ Command-line automation/scripting
- ✅ CI/CD pipelines
- ✅ Batch conversion of multiple files
- ✅ Debugging XACRO files

**Use Blender import for:**
- ✅ Interactive editing in Blender
- ✅ Visual inspection of robots
- ✅ One-off imports

### Alternative: ROS xacro Command

If you have ROS2 installed:

```bash
ros2 run xacro xacro robot.urdf.xacro > robot.urdf
```

### About XACRO

XACRO (XML Macros) extends URDF with:
- Variables/properties: `<xacro:property name="width" value="0.2"/>`
- Macros: `<xacro:macro name="wheel" params="prefix">`
- Math expressions: `${pi/2}`, `${width*2}`
- File includes: `<xacro:include filename="common.xacro"/>`

### Learn More

- **xacrodoc**: https://github.com/adamheins/xacrodoc
- **XACRO Tutorial**: https://docs.ros.org/en/foxy/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html
