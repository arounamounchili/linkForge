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
3. Click **"Import URDF/XACRO"**
4. Select example file
5. Robot appears in viewport

### Visualize in RViz2

```bash
# Publish robot description
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(cat mobile_robot.urdf)"

# Launch RViz
ros2 run rviz2 rviz2

# In RViz:
# - Add → RobotModel
# - Set Fixed Frame: base_link
```

### Test in Gazebo

```bash
gazebo --verbose
ros2 run gazebo_ros spawn_entity.py \
  -entity my_robot \
  -file mobile_robot.urdf
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

Test with LinkForge:

```python
from linkforge.core.parsers.urdf_parser import parse_urdf

robot = parse_urdf("mobile_robot.urdf")
print(f"Robot: {robot.name}")
print(f"Links: {len(robot.links)}")
print(f"Joints: {len(robot.joints)}")
```

All examples include:
- Valid URDF syntax
- Proper inertial properties
- Realistic dimensions
- Correct joint limits

---

## ROS2 Integration

### Navigation (Mobile Robot)

```bash
# Launch Nav2
ros2 launch nav2_bringup navigation_launch.py

# Send goal
ros2 topic pub /goal_pose geometry_msgs/PoseStamped ...
```

### MoveIt! (Robot Arm)

```bash
# Setup MoveIt config
ros2 run moveit_setup_assistant moveit_setup_assistant

# Launch MoveIt
ros2 launch <robot>_moveit_config demo.launch.py
```

---

## License

GPL-3.0-or-later (same as LinkForge)

Free for research, education, and commercial use.

---

**Questions?** https://github.com/arounamounchili/linkforge/issues
