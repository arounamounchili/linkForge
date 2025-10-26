# LinkForge Quick Start Guide

Get from 3D model to working URDF in 5 minutes!

## Installation

1. **Download** the latest `linkforge-{version}.zip` from releases
2. **Open Blender 4.2+**
3. Go to **Edit → Preferences → Get Extensions**
4. Click dropdown (⌄) → **Install from Disk**
5. Select the `.zip` file
6. **Done!** LinkForge appears in the 3D Viewport sidebar (press `N`)

## Workflow Overview

```
Create/Import Model → Mark Links → Define Joints → Validate → Export URDF
```

---

## Method 1: Create Robot from Scratch

### Step 1: Create Geometry

1. Add mesh objects in Blender (Cube, Cylinder, etc.)
2. Model your robot links
3. Position them correctly

### Step 2: Mark as Links

1. Select a mesh object (e.g., base)
2. Press `N` → **LinkForge** tab → **Link** panel
3. Click **"Mark as Link"**
4. Set properties:
   - **Link Name**: `base_link` (required)
   - **Mass**: e.g., `1.0` kg
   - **Visual Geometry**: Choose type (Box, Cylinder, Sphere, Mesh)
   - **Collision Geometry**: Usually same as visual
   - ☑ **Auto-Calculate Inertia** (recommended)

5. Repeat for all robot parts

**Tips:**
- Name your first link `base_link` (ROS convention)
- Use descriptive names: `arm_link`, `gripper_palm`, etc.
- Set realistic masses for accurate physics

### Step 3: Create Joints

1. In **Joint** panel, click **"Create Joint"**
2. An Empty object appears - position it at the joint location
3. Set properties:
   - **Joint Name**: e.g., `shoulder_joint`
   - **Type**: Revolute, Prismatic, Fixed, Continuous, etc.
   - **Parent Link**: Select from dropdown
   - **Child Link**: Select from dropdown
   - **Axis**: X, Y, or Z (rotation/movement axis)
   - **Limits**: Set min/max angles (for revolute) or distances (for prismatic)

4. Repeat for all joints

**Joint Types:**
- **Revolute**: Hinge (rotates with limits) - elbows, knees
- **Continuous**: Wheel (rotates 360°) - wheels, turrets
- **Prismatic**: Slider (linear movement) - grippers, lifts
- **Fixed**: Rigid connection - sensors, mounts

### Step 4: Validate

1. In **Robot** panel:
   - Check **Links**: Shows count
   - Check **Joints**: Shows count
   - Check **Root**: Should show base link name
2. Click **"Validate Robot"**
3. Fix any errors shown

### Step 5: Export

1. Go to **Export** panel
2. Set options:
   - **Format**: URDF or XACRO
   - ☑ **Export Meshes** (if using mesh geometry)
   - **Mesh Format**: OBJ (with materials) or STL
3. Click **"Export URDF/XACRO"**
4. Choose save location
5. **Done!** Your URDF is ready for ROS

---

## Method 2: Import Existing URDF

### Import from URDF File

1. Press `N` → **LinkForge** tab → **Import** panel
2. Click **"Import URDF/XACRO"**
3. Select your `.urdf` file
4. Robot appears in viewport with correct hierarchy

**What Gets Imported:**
- ✓ All links (with geometry, colors, inertia)
- ✓ All joints (with types, limits, dynamics)
- ✓ Materials and colors
- ✓ Proper kinematic tree structure

### Modify Imported Robot

1. Select any link object
2. Modify in Link panel:
   - Change mass, geometry
   - Edit colors/materials
   - Update inertia
3. Select any joint Empty
4. Modify in Joint panel:
   - Change limits
   - Adjust dynamics
   - Reposition joint origin
5. **Export** to save changes back to URDF

---

## Visualization Features

### Show/Hide Joint Axes

- In **Robot** panel → **Visualization**
- ☑ **Show Joint Axes**: Toggle RGB 3-axis markers
  - **Red** = X-axis
  - **Green** = Y-axis
  - **Blue** = Z-axis

### Kinematic Tree

- In **Robot** panel:
- ☑ **Show Structure**: Displays robot hierarchy tree
- Shows parent-child relationships

---

## Common Workflows

### Mobile Robot (Wheeled Base)

```
base_link
├─ left_wheel (continuous joint, Y-axis)
├─ right_wheel (continuous joint, Y-axis)
└─ caster (fixed joint)
```

**Example:** See `examples/mobile_base.urdf`

### Robot Arm

```
base_link
└─ link1 (revolute joint, Z-axis)
   └─ link2 (revolute joint, Y-axis)
      └─ link3 (revolute joint, Y-axis)
```

**Example:** See `examples/simple_arm.urdf`

### Gripper

```
palm
├─ left_finger (prismatic joint, X-axis)
└─ right_finger (prismatic joint, -X-axis, mimic left)
```

**Example:** See `examples/simple_gripper.urdf`

---

## Tips & Best Practices

### Naming Conventions

- ✓ Use `base_link` for root link (ROS standard)
- ✓ Use descriptive names: `wheel_left`, `arm_joint_1`
- ✓ Avoid spaces and special characters
- ✓ Use lowercase with underscores: `robot_arm` not `RobotArm`

### Mass & Inertia

- ✓ Use realistic masses (kg)
- ✓ Enable **Auto-Calculate Inertia** for simple geometry
- ✓ For complex meshes, use simplified collision geometry
- ✗ Don't use zero mass (causes physics errors)

### Joint Limits

- ✓ Set realistic limits for revolute joints (radians)
- ✓ Set effort and velocity limits
- ✓ For continuous joints, no limits needed
- ✓ Test in RViz to verify range of motion

### Coordinate Systems

- LinkForge uses **ROS conventions** (Z-up, X-forward)
- Blender uses Z-up, Y-forward
- Export handles conversion automatically

### Materials

- ✓ Use Blender materials (Principled BSDF)
- ✓ Set Base Color for visual appearance
- ✓ Materials export to URDF color tags

---

## Testing Your URDF

### In RViz

```bash
# Launch RViz
ros2 run rviz2 rviz2

# Add → RobotModel
# Set Fixed Frame: base_link
# Set Robot Description: /robot_description

# Publish URDF
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(cat your_robot.urdf)"
```

### In Gazebo

```bash
# Add robot to Gazebo world
ros2 run gazebo_ros spawn_entity.py \
  -entity my_robot \
  -file your_robot.urdf
```

---

## Troubleshooting

### "Root: Not found" Error

**Cause:** No link is marked as root (not a child of any joint)

**Fix:**
1. Check Robot panel → Structure
2. Ensure one link has no parent joint
3. Name it `base_link`

### "Validation errors"

**Cause:** Missing properties or invalid structure

**Fix:**
1. Click Validate to see specific errors
2. Common issues:
   - Missing parent/child in joints
   - Duplicate link/joint names
   - Missing mass/inertia
   - Invalid joint limits

### Imported Robot Looks Wrong

**Cause:** Coordinate system or scale issues

**Fix:**
1. Check if URDF uses meters (Blender default)
2. Verify joint origins match expected positions
3. Toggle **Show Joint Axes** to visualize joints

### Export Button Grayed Out

**Cause:** No links marked in scene

**Fix:**
1. Mark at least one object as a link
2. Check Link panel shows `is_robot_link = True`

### Mesh Files Not Exporting

**Cause:** Export Meshes option disabled or unsupported geometry

**Fix:**
1. In Export panel: ☑ **Export Meshes**
2. Set **Visual Geometry Type** to MESH for links
3. Use OBJ format (recommended)

---

## Examples

LinkForge includes example URDFs:

| Example | Description | Features |
|---------|-------------|----------|
| `simple_arm.urdf` | 2-DOF robot arm | Revolute joints, colors, inertia |
| `simple_gripper.urdf` | Parallel gripper | Prismatic joints, mimic constraint |
| `mobile_base.urdf` | Wheeled robot | Continuous joints, casters |

**Import Examples:**
1. LinkForge → Import panel
2. Select example URDF from `examples/` folder
3. Explore the structure
4. Modify and re-export

---

## Next Steps

- Read the [full documentation](docs/) for advanced features
- Join [GitHub Discussions](https://github.com/arounamounchili/linkforge/discussions)
- Report issues on [GitHub](https://github.com/arounamounchili/linkforge/issues)

**Happy robot building! 🤖**
