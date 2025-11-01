# LinkForge Tools

Utility scripts for working with LinkForge and robot description files.

## XACRO to URDF Converter

`convert_xacro.py` - Convert XACRO files to URDF without requiring ROS installation.

### Installation

The converter requires the `xacrodoc` Python package:

```bash
pip install xacrodoc
```

### Usage

**Basic conversion** (creates `robot.urdf` from `robot.urdf.xacro`):
```bash
python tools/convert_xacro.py robot.urdf.xacro
```

**Specify output file:**
```bash
python tools/convert_xacro.py robot.urdf.xacro -o my_robot.urdf
```

**Compact output (no pretty printing):**
```bash
python tools/convert_xacro.py robot.urdf.xacro --no-pretty
```

**Check if xacrodoc is installed:**
```bash
python tools/convert_xacro.py --check
```

### Why This Tool?

LinkForge imports **URDF files only**. If you have XACRO files, you need to convert them first.

**Options for conversion:**

1. **Use this tool** (recommended for non-ROS users):
   - No ROS installation required
   - Simple Python script
   - Uses industry-standard `xacrodoc` library

2. **Use ROS xacro** (if you have ROS installed):
   ```bash
   xacro robot.urdf.xacro > robot.urdf
   ```

3. **Manual xacrodoc usage:**
   ```bash
   pip install xacrodoc
   python -c "from xacrodoc import XacroDoc; print(XacroDoc.from_file('robot.urdf.xacro').to_urdf_string())" > robot.urdf
   ```

### About XACRO

XACRO (XML Macros) is an extension of URDF that supports:
- Variables and properties: `<xacro:property name="width" value="0.2"/>`
- Macros for repeated elements: `<xacro:macro name="wheel" params="prefix">`
- Mathematical expressions: `${pi/2}`, `${width*2}`
- File includes: `<xacro:include filename="common.xacro"/>`
- Package references: `$(find my_robot_description)/urdf/base.xacro`

XACRO files must be "compiled" to plain URDF before use in most simulators and tools.

### Troubleshooting

**"xacrodoc is not installed"**
```bash
pip install xacrodoc
```

**"Module 'rospkg' not found"**

xacrodoc requires rospkg for package resolution:
```bash
pip install rospkg
```

**"Conversion failed"**

Check that your XACRO file is valid. Common issues:
- Syntax errors in XACRO macros
- Missing included files
- Invalid package references

### Alternative: Full ROS Installation

If you're doing serious ROS development, install ROS2 and use the built-in `xacro` command:

- **ROS2 Humble**: https://docs.ros.org/en/humble/Installation.html
- **ROS2 Jazzy**: https://docs.ros.org/en/jazzy/Installation.html

Then use:
```bash
ros2 run xacro xacro robot.urdf.xacro > robot.urdf
```

### Learn More

- **xacrodoc**: https://github.com/adamheins/xacrodoc
- **XACRO Tutorial**: https://docs.ros.org/en/foxy/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html
- **URDF Specification**: http://wiki.ros.org/urdf/XML
