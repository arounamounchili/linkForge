# LinkForge Examples

Simple, functional URDF examples for robotics.

## Examples

### 1. mobile_robot.urdf

**Type:** 4-Wheel Drive Mobile Robot with LiDAR

**Description:**
Basic mobile robot platform with four driven wheels and top-mounted LiDAR sensor. Simple and functional design for navigation and SLAM applications.

**Components:**
- Base platform (400mm × 300mm × 120mm)
- 4× driven wheels (continuous rotation)
- LiDAR sensor (fixed mount)

**Structure:**
- 6 links total (base + 4 wheels + lidar)
- 5 joints (4 continuous wheel joints + 1 fixed lidar mount)

**Use Cases:**
- ROS2 Navigation
- SLAM mapping
- Autonomous navigation
- Mobile robotics education

---

### 2. robot_arm.urdf

**Type:** 4-DOF Robot Arm

**Description:**
Simple serial manipulator with 4 revolute joints. Clean design without unnecessary complexity, suitable for pick-and-place and basic manipulation tasks.

**Components:**
- Fixed base (cylinder)
- Shoulder link (vertical box)
- Upper arm link (box)
- Forearm link (box)
- Wrist link (end effector mount)

**Structure:**
- 5 links total
- 4 revolute joints (properly positioned at link endpoints)

**Joint Configuration:**
1. base_to_shoulder (Z-axis rotation): ±180°
2. shoulder_to_upper_arm (Y-axis pitch): ±90°
3. upper_arm_to_forearm (Y-axis pitch): ±135°
4. forearm_to_wrist (Z-axis roll): ±180°

**Use Cases:**
- Pick-and-place operations
- Basic manipulation
- Robot arm education
- MoveIt! motion planning

---

## Quick Start

### Import into Blender

1. Open Blender 4.2+
2. Press `N` → **LinkForge** tab
3. Click **"Import Robot"**
4. Select example file (`.urdf`, `.xacro`, or `.urdf.xacro`)
5. Robot appears in viewport with full hierarchy

### Visualize in RViz2

```bash
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(cat mobile_robot.urdf)"

ros2 run rviz2 rviz2
# Add → RobotModel, Set Fixed Frame: base_link
```

### Test in Gazebo

```bash
ros2 run gazebo_ros spawn_entity.py -entity my_robot -file mobile_robot.urdf
```

---

## Modification Tips

### Mobile Robot
- **Change wheel size**: Modify cylinder radius for different terrains
- **Add sensors**: Attach camera, IMU, or ultrasonic sensors to base_link
- **Convert to differential drive**: Remove 2 wheels, keep front pair only

### Robot Arm
- **Add more joints**: Duplicate link/joint pattern for 6-DOF or 7-DOF arm
- **Change dimensions**: Modify box dimensions for different workspace/reach
- **Add gripper**: Attach end-effector to wrist_link
- **Change geometry**: Replace boxes with cylinders for industrial look

---

## Validation

Validate before export in Blender:
1. Click **"Validate"** in LinkForge panel
2. Review any warnings or errors
3. Fix issues and re-validate

All examples include valid URDF syntax, proper inertial properties, realistic dimensions, and correct joint limits.

---

## ROS2 Integration

**Mobile Robot**: Use with Nav2 for autonomous navigation
**Robot Arm**: Use with MoveIt! for motion planning

See ROS2 documentation for setup details.

---

**Questions?** https://github.com/arounamounchili/linkforge/issues
