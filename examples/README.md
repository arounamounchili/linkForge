# LinkForge Examples

This directory contains example URDF files demonstrating different robot types and features.

## Examples

### 1. simple_arm.urdf

**Type:** 2-DOF Robot Arm

**Features Demonstrated:**
- Revolute joints with limits
- Multiple links in series
- Different geometry types (cylinder, box)
- Material colors (gray, blue, orange)
- Proper inertial properties
- Joint dynamics (damping, friction)

**Structure:**
```
base_link (cylinder)
└─ joint1 (revolute, Z-axis, ±180°)
   └─ link1 (box)
      └─ joint2 (revolute, Y-axis, ±90°)
         └─ link2 (box)
```

**Use Case:** Simple vertical robot arm, good for learning joint configuration

---

### 2. simple_gripper.urdf

**Type:** Parallel Jaw Gripper

**Features Demonstrated:**
- Prismatic (linear) joints
- Mimic constraint (right finger follows left)
- Symmetric structure
- Small-scale precision parts

**Structure:**
```
palm (box)
├─ left_finger_joint (prismatic, X-axis, 0-30mm)
│  └─ left_finger (box)
└─ right_finger_joint (prismatic, -X-axis, 0-30mm, mimics left)
   └─ right_finger (box)
```

**Use Case:** End-effector for robot arms, demonstrates prismatic joints and mimic

---

### 3. mobile_base.urdf

**Type:** Differential Drive Mobile Robot

**Features Demonstrated:**
- Continuous rotation joints (wheels)
- Fixed joints (casters)
- Spherical geometry (caster balls)
- Multiple wheels with symmetry
- Realistic mobile robot kinematics

**Structure:**
```
base_link (box)
├─ left_wheel_joint (continuous, Y-axis)
│  └─ left_wheel (cylinder)
├─ right_wheel_joint (continuous, Y-axis)
│  └─ right_wheel (cylinder)
├─ front_caster_joint (fixed)
│  └─ front_caster (sphere)
└─ rear_caster_joint (fixed)
   └─ rear_caster (sphere)
```

**Use Case:** Mobile robot base, demonstrates continuous and fixed joints

---

## How to Use Examples

### Import into Blender

1. Open Blender 4.2+
2. Press `N` → **LinkForge** tab
3. Go to **Import** panel
4. Click **"Import URDF/XACRO"**
5. Select an example URDF file
6. Robot appears in viewport

### Test in RViz

```bash
# Launch RViz
ros2 run rviz2 rviz2

# In another terminal, publish the URDF
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(cat simple_arm.urdf)"

# In RViz:
# - Add → RobotModel
# - Set Fixed Frame: base_link
# - Adjust Joint State Publisher to move joints
```

### Modify and Export

1. Import example URDF into Blender
2. Select any link or joint
3. Modify properties in LinkForge panels:
   - Change masses
   - Adjust joint limits
   - Modify colors
   - Add/remove links
4. Export back to URDF
5. Test changes in RViz/Gazebo

---

## Learning Path

**Beginner:**
1. Start with `simple_arm.urdf`
   - Learn link/joint structure
   - Understand revolute joints
   - Practice modifying properties

**Intermediate:**
2. Explore `simple_gripper.urdf`
   - Learn prismatic joints
   - Understand mimic constraints
   - See symmetrical designs

**Advanced:**
3. Study `mobile_base.urdf`
   - Continuous joints for wheels
   - Fixed joints for attachments
   - Complex multi-link robots

---

## Creating Your Own

Use these examples as templates:

### Robot Arm Template
- Base: `simple_arm.urdf`
- Modify: Add more links/joints
- Use for: Manipulators, legs

### Gripper Template
- Base: `simple_gripper.urdf`
- Modify: Finger count, jaw type
- Use for: End-effectors, hands

### Mobile Base Template
- Base: `mobile_base.urdf`
- Modify: Wheel count, shape
- Use for: Ground vehicles, platforms

---

## Technical Details

### Joint Type Reference

| Type | Movement | Limits | Example Use |
|------|----------|--------|-------------|
| `revolute` | Rotation | Yes (min/max angle) | Elbows, knees |
| `continuous` | Rotation | No | Wheels, turrets |
| `prismatic` | Linear | Yes (min/max distance) | Sliders, lifts |
| `fixed` | None | N/A | Sensors, mounts |
| `floating` | 6-DOF | N/A | Floating bases |
| `planar` | 2D plane | N/A | Planar mechanisms |

### Geometry Types

- **Box**: `<box size="x y z"/>`
- **Cylinder**: `<cylinder radius="r" length="l"/>`
- **Sphere**: `<sphere radius="r"/>`
- **Capsule**: `<capsule radius="r" length="l"/>`
- **Mesh**: `<mesh filename="path/to/mesh.obj"/>`

### Coordinate System

- **X-axis (Red)**: Forward
- **Y-axis (Green)**: Left
- **Z-axis (Blue)**: Up

All examples use ROS conventions (Z-up, X-forward).

---

## Validation

All examples pass LinkForge validation:
- ✓ Valid kinematic tree structure
- ✓ No duplicate names
- ✓ Proper parent-child relationships
- ✓ Realistic inertial properties
- ✓ Valid joint limits

Test with:
```python
from linkforge.core.parsers.urdf_parser import parse_urdf

robot = parse_urdf("simple_arm.urdf")
errors = robot.validate_tree_structure()
print(f"Validation: {'PASS' if not errors else 'FAIL'}")
```

---

## Contributing Examples

Want to contribute an example? Please:
1. Create a well-structured URDF
2. Include proper comments
3. Use realistic masses/inertias
4. Test in RViz
5. Submit PR with description

Example types we'd love to see:
- Humanoid robot
- Quadruped (4-legged)
- Robotic hand
- Industrial manipulator
- Flying drone

---

**Questions?** Open an issue on [GitHub](https://github.com/arounamounchili/linkforge/issues)
