# LinkForge - Comprehensive Manual Testing Guide

**Version:** 0.5.0
**Purpose:** Complete A-to-Z testing of all features in Blender
**Estimated Time:** 2-3 hours

**Testing Status:** 🟢 IN PROGRESS
**Last Updated:** 2025-11-02
**Tester:** Arouna Patouossa Mounchili

---

## ✅ Completed Tests

- [x] **Pre-Testing Setup** - Extension built and installed
- [x] **Test 1: Basic UI & Preferences** - All panels visible, order correct, UX improved

## 🔄 Current Test

- [ ] **Test 2: Link Creation & Properties** - STARTING NOW

---

## Pre-Testing Setup

### 1. Build and Install Extension

```bash
# Activate virtual environment
source .venv/bin/activate  # or: source venv/bin/activate

# Run tests to ensure code is working
./venv/bin/pytest tests/ -q --tb=short

# Build the extension package
python build_extension.py

# Note the output file location (usually dist/linkforge-0.4.0.zip)
```

### 2. Install in Blender

1. Open **Blender 4.2+**
2. Go to **Edit → Preferences → Get Extensions**
3. Click dropdown **(⌄)** → **Install from Disk**
4. Select `dist/linkforge-{version}.zip`
5. Enable the extension if not auto-enabled
6. **Restart Blender** to ensure clean installation

### 3. Verify Installation

1. Open Blender
2. Press **N** in 3D Viewport to open sidebar
3. Look for **LinkForge** tab
4. Check panels are visible:
   - Robot
   - Links
   - Joints
   - Sensors
   - Transmissions
   - Validation
   - Export

---

## Test Suite Overview

We'll test in this order:

1. ✅ **Basic UI & Preferences**
2. ✅ **Link Creation & Properties**
3. ✅ **Joint Creation & Types**
4. ✅ **Sensors (Camera, LIDAR, IMU, GPS)**
5. ✅ **Transmissions (Simple & Differential)**
6. ✅ **Presets System**
7. ✅ **Validation System**
8. ✅ **Export (URDF & XACRO)**
9. ✅ **Import (URDF & XACRO)**
10. ✅ **Round-trip Testing**
11. ✅ **Visual Features (Joint Axes)**

---

## Test 1: Basic UI & Preferences

### 1.1 Check Extension Preferences

**Steps:**
1. Edit → Preferences → Get Extensions
2. Find **LinkForge** in installed extensions
3. Click to expand extension details
4. Check information displays correctly:
   - Name: LinkForge
   - Version: 0.4.0 or higher
   - Author: Arouna Patouossa Mounchili
   - Description visible

**Expected:** All metadata displays correctly

**Status:** [✅] Pass [ ] Fail
**Notes:** All information displays correctly

### 1.2 Check Joint Visualization Preferences

**Steps:**
1. Open extension preferences (same location)
2. Look for LinkForge preferences section
3. Check for joint axis display settings

**Expected:** Preferences section visible with joint display options

**Status:** [✅] Pass [ ] Fail
**Notes:** Preferences accessible

### 1.3 Check All Panels Visible

**Steps:**
1. In 3D Viewport, press **N**
2. Navigate to **LinkForge** tab
3. Verify all panels present in correct order:
   - Import (collapsed)
   - Links (EXPANDED)
   - Joints (EXPANDED)
   - Sensors (collapsed)
   - Transmissions (collapsed)
   - Validation (collapsed)
   - Robot (collapsed)
   - Export (collapsed)

**Expected:** All 8 panels visible in workflow order, Links/Joints expanded, others collapsed

**Status:** [✅] Pass [ ] Fail
**Notes:** Panel ordering improved! Sensors and Transmissions now collapsed by default. Better UX.

---

## Test 2: Link Creation & Properties

### 2.1 Create Base Link (Cube)

**Steps:**
1. Delete default cube if present: X → Delete
2. Add new cube: Shift+A → Mesh → Cube
3. Rename to "base_link" (F2 or double-click in outliner)
4. In LinkForge → Links panel, click **Mark as Link**
5. Set properties:
   - Link Name: `base_link`
   - Mass: `5.0`
   - Enable **Auto-Calculate Inertia**
   - Visual Geometry Type: Box
   - Collision Geometry Type: Box

**Expected:**
- Object properties show LinkForge link data
- Inertia values auto-calculated
- No errors in console

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 2.2 Test All Geometry Types

**Test each primitive geometry:**

#### Box (already tested above)
**Status:** [ ] Pass [ ] Fail

#### Cylinder
**Steps:**
1. Shift+A → Mesh → Cylinder
2. Rename to "cylinder_link"
3. Mark as Link
4. Set Visual Geometry Type: Cylinder
5. Mass: 2.0
6. Auto-Calculate Inertia: ON

**Expected:** Inertia calculated for cylinder
**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

#### Sphere
**Steps:**
1. Shift+A → Mesh → UV Sphere
2. Rename to "sphere_link"
3. Mark as Link
4. Set Visual Geometry Type: Sphere
5. Mass: 1.0
6. Auto-Calculate Inertia: ON

**Expected:** Inertia calculated for sphere
**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

#### Capsule
**Steps:**
1. Shift+A → Mesh → Cylinder
2. Add modifier: Subdivision Surface (to approximate capsule)
3. Rename to "capsule_link"
4. Mark as Link
5. Set Visual Geometry Type: Capsule
6. Mass: 1.5
7. Auto-Calculate Inertia: ON

**Expected:** Inertia calculated for capsule
**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 2.3 Test Mesh Geometry (Custom Shape)

**Steps:**
1. Shift+A → Mesh → Monkey (Suzanne)
2. Rename to "mesh_link"
3. Mark as Link
4. Set Visual Geometry Type: Mesh
5. Mass: 3.0
6. Auto-Calculate Inertia: ON

**Expected:**
- Mesh-based inertia calculation (slower)
- Console may show calculation progress
- Inertia tensor populated

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 2.4 Test Material Colors

**Steps:**
1. Select base_link
2. In Links panel, expand **Material** section
3. Set color: Red (1.0, 0.0, 0.0, 1.0)
4. Apply material
5. Select cylinder_link
6. Set color: Blue (0.0, 0.0, 1.0, 1.0)
7. Apply material

**Expected:**
- Links display with assigned colors in viewport
- Material appears in Shading workspace

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

---

## Test 3: Joint Creation & Types

### 3.1 Create Revolute Joint

**Steps:**
1. In Joints panel, click **Create Joint**
2. Set properties:
   - Joint Name: `base_to_cylinder`
   - Joint Type: **Revolute**
   - Parent Link: `base_link`
   - Child Link: `cylinder_link`
   - Axis: Z
   - Lower Limit: `-1.57` (radians, ~-90°)
   - Upper Limit: `1.57` (radians, ~90°)
3. Click **Create Joint**

**Expected:**
- Empty object created at origin named "base_to_cylinder"
- Joint axes visible (RGB arrows)
- No errors

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 3.2 Create Prismatic Joint

**Steps:**
1. Add new cube: Shift+A → Mesh → Cube
2. Rename to "prismatic_link"
3. Mark as Link (mass: 1.0)
4. Create Joint:
   - Joint Name: `cylinder_to_prismatic`
   - Joint Type: **Prismatic**
   - Parent Link: `cylinder_link`
   - Child Link: `prismatic_link`
   - Axis: X
   - Lower Limit: `0.0`
   - Upper Limit: `0.5` (meters)

**Expected:**
- Prismatic joint created
- Linear axis limits set correctly

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 3.3 Create Fixed Joint

**Steps:**
1. Add new sphere: Shift+A → Mesh → UV Sphere
2. Rename to "fixed_link"
3. Mark as Link (mass: 0.5)
4. Create Joint:
   - Joint Name: `prismatic_to_fixed`
   - Joint Type: **Fixed**
   - Parent Link: `prismatic_link`
   - Child Link: `fixed_link`

**Expected:**
- Fixed joint created
- No limit properties (fixed joints don't move)

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 3.4 Create Continuous Joint

**Steps:**
1. Add new cylinder: Shift+A → Mesh → Cylinder
2. Rename to "continuous_link"
3. Mark as Link (mass: 1.0)
4. Create Joint:
   - Joint Name: `fixed_to_continuous`
   - Joint Type: **Continuous**
   - Parent Link: `fixed_link`
   - Child Link: `continuous_link`
   - Axis: Z

**Expected:**
- Continuous joint created
- No limit properties (unlimited rotation)

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 3.5 Create Floating Joint

**Steps:**
1. Add new cube: Shift+A → Mesh → Cube
2. Rename to "floating_link"
3. Mark as Link (mass: 0.5)
4. Create Joint:
   - Joint Name: `base_to_floating`
   - Joint Type: **Floating**
   - Parent Link: `base_link`
   - Child Link: `floating_link`

**Expected:**
- Floating joint created (6-DOF)
- No axis or limit properties

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 3.6 Create Planar Joint

**Steps:**
1. Add new cube: Shift+A → Mesh → Cube
2. Rename to "planar_link"
3. Mark as Link (mass: 0.5)
4. Create Joint:
   - Joint Name: `base_to_planar`
   - Joint Type: **Planar**
   - Parent Link: `base_link`
   - Child Link: `planar_link`
   - Axis: Z (normal to plane)

**Expected:**
- Planar joint created (2D movement)
- Axis defines plane normal

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 3.7 Test Joint Dynamics

**Steps:**
1. Select `base_to_cylinder` joint (Empty object)
2. In Joint panel, expand **Dynamics** section
3. Set:
   - Damping: `0.5`
   - Friction: `0.1`

**Expected:**
- Dynamics properties saved
- Values persist when selecting other objects

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 3.8 Test Joint Mimic

**Steps:**
1. Select `cylinder_to_prismatic` joint
2. Expand **Mimic** section
3. Set:
   - Mimic Joint: `base_to_cylinder`
   - Multiplier: `2.0`
   - Offset: `0.0`

**Expected:**
- Mimic relationship configured
- Joint dropdown shows available joints

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

---

## Test 4: Sensors

### 4.1 Create Camera Sensor

**Steps:**
1. Select `cylinder_link`
2. In Sensors panel, click **Add Sensor**
3. Set properties:
   - Sensor Name: `front_camera`
   - Sensor Type: **Camera**
   - Update Rate: `30`
   - Horizontal FOV: `1.57` (90°)
   - Image Width: `640`
   - Image Height: `480`
   - Camera Format: `R8G8B8`

**Expected:**
- Sensor created and attached to link
- Blender camera object created (optional visualization)
- Properties saved correctly

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 4.2 Create Depth Camera Sensor

**Steps:**
1. Select `sphere_link`
2. Add Sensor:
   - Sensor Name: `depth_camera`
   - Sensor Type: **Depth Camera**
   - Update Rate: `20`
   - Horizontal FOV: `1.0`
   - Image Width: `320`
   - Image Height: `240`
   - Camera Format: `L8`

**Expected:**
- Depth camera sensor created
- Additional depth properties visible

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 4.3 Create LIDAR Sensor

**Steps:**
1. Select `base_link`
2. Add Sensor:
   - Sensor Name: `lidar_sensor`
   - Sensor Type: **Ray (LIDAR)**
   - Update Rate: `10`
   - Samples: `360`
   - Resolution: `1`
   - Min Angle: `-1.57` (-90°)
   - Max Angle: `1.57` (90°)
   - Min Range: `0.1`
   - Max Range: `10.0`

**Expected:**
- LIDAR sensor created
- Ray/scan properties configured

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 4.4 Create IMU Sensor

**Steps:**
1. Select `base_link`
2. Add Sensor:
   - Sensor Name: `imu_sensor`
   - Sensor Type: **IMU**
   - Update Rate: `100`
   - Noise: Standard Deviation: `0.01`

**Expected:**
- IMU sensor created
- Noise properties available

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 4.5 Create GPS Sensor

**Steps:**
1. Select `base_link`
2. Add Sensor:
   - Sensor Name: `gps_sensor`
   - Sensor Type: **GPS**
   - Update Rate: `1`
   - Noise: Standard Deviation: `1.0`

**Expected:**
- GPS sensor created
- Position noise configured

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 4.6 Delete Sensor

**Steps:**
1. Select `base_link`
2. In Sensors panel, find `gps_sensor` in list
3. Click **Delete** button next to sensor

**Expected:**
- Sensor removed from list
- No errors

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

---

## Test 5: Transmissions

### 5.1 Create Simple Transmission

**Steps:**
1. In Transmissions panel, click **Add Transmission**
2. Set properties:
   - Transmission Name: `base_transmission`
   - Type: **Simple**
   - Joint: `base_to_cylinder`
   - Actuator Name: `base_motor`
   - Hardware Interface: **Position (ROS2 Control)**
   - Mechanical Reduction: `100.0`

**Expected:**
- Transmission created
- Empty object created in scene
- Properties saved

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 5.2 Create Differential Transmission

**Steps:**
1. Add Transmission:
   - Transmission Name: `diff_transmission`
   - Type: **Differential**
   - Joint 1: `cylinder_to_prismatic`
   - Joint 2: `base_to_cylinder`
   - Actuator 1 Name: `left_motor`
   - Actuator 2 Name: `right_motor`
   - Hardware Interface: **Velocity (ROS2 Control)**
   - Mechanical Reduction: `50.0`

**Expected:**
- Differential transmission created
- Two joints referenced
- Two actuators configured

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 5.3 Test All Hardware Interfaces

**Test each interface type:**

1. **Position (ROS2 Control)** - Already tested
2. **Velocity (ROS2 Control)** - Already tested
3. **Effort (ROS2 Control)**
4. **Position Interface (ROS1)**
5. **Velocity Interface (ROS1)**
6. **Effort Interface (ROS1)**

**Steps:**
1. Select transmission Empty object
2. Change Hardware Interface dropdown
3. Save and verify property updates

**Expected:** All interface types selectable and saved
**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 5.4 Delete Transmission

**Steps:**
1. In Transmissions panel, find `diff_transmission`
2. Click **Delete** button

**Expected:**
- Transmission removed
- Empty object deleted from scene

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

---

## Test 6: Presets System

### 6.1 Save Joint Preset

**Steps:**
1. Select `base_to_cylinder` joint (Empty object)
2. In Joint panel, ensure properties are set:
   - Damping: 0.5
   - Friction: 0.1
   - Limits: -1.57 to 1.57
3. Click **Save as Preset** button
4. Enter preset name: `servo_motor_joint`
5. Save

**Expected:**
- Preset saved successfully
- Confirmation message or no error

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 6.2 Apply Joint Preset

**Steps:**
1. Create new joint: `test_preset_joint`
   - Type: Revolute
   - Parent: base_link
   - Child: cylinder_link (create new dummy link if needed)
2. In Joint panel, find **Presets** dropdown
3. Select `servo_motor_joint`
4. Click **Apply Preset**

**Expected:**
- Joint properties updated to match preset
- Damping: 0.5, Friction: 0.1, Limits: -1.57 to 1.57

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 6.3 Save Material Preset

**Steps:**
1. Select `base_link`
2. Set material color: Green (0.0, 1.0, 0.0, 1.0)
3. Click **Save Material Preset**
4. Enter name: `green_metal`

**Expected:**
- Material preset saved
- Available in preset list

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 6.4 Apply Material Preset

**Steps:**
1. Select different link (e.g., `cylinder_link`)
2. In Links panel, Presets dropdown
3. Select `green_metal`
4. Apply preset

**Expected:**
- Link material changes to green
- Viewport updates

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 6.5 Save Sensor Preset

**Steps:**
1. Select link with camera sensor (`cylinder_link`)
2. Select camera sensor in Sensors panel
3. Configure camera:
   - Resolution: 1920x1080
   - FOV: 1.57
   - Format: R8G8B8
4. Click **Save Sensor Preset**
5. Enter name: `hd_camera`

**Expected:**
- Sensor preset saved
- Available for reuse

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 6.6 Apply Sensor Preset

**Steps:**
1. Select different link
2. Add new sensor
3. Select `hd_camera` from presets
4. Apply

**Expected:**
- Sensor properties match preset
- Resolution: 1920x1080

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

---

## Test 7: Validation System

### 7.1 Validate Valid Robot

**Steps:**
1. With current robot scene (should be valid)
2. In Validation panel, click **Validate Robot**
3. Review results

**Expected:**
- Validation passes
- Green checkmark or "Robot is valid" message
- No errors listed

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 7.2 Test Cycle Detection

**Steps:**
1. Create invalid cycle:
   - Create joint: parent=`continuous_link`, child=`base_link`
   - This creates circular dependency
2. Click **Validate Robot**

**Expected:**
- Validation FAILS
- Error: "Cycle detected in kinematic tree"
- Lists joints involved in cycle

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

**Cleanup:** Delete the invalid joint before continuing

### 7.3 Test Orphaned Link Detection

**Steps:**
1. Create new link without joint: `orphan_link`
2. Mark as link but don't create joint to it
3. Click **Validate Robot**

**Expected:**
- Warning: "Orphaned link detected: orphan_link"
- Validation may pass with warnings

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

**Cleanup:** Delete orphan_link or connect it properly

### 7.4 Test Missing Parent/Child

**Steps:**
1. Select a joint Empty object in outliner
2. In Joint panel, set Child Link to `(none)` or invalid link
3. Validate

**Expected:**
- Error: "Joint has invalid child link"
- Validation fails

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

**Cleanup:** Fix the joint or delete it

---

## Test 8: Export (URDF & XACRO)

### 8.1 Export to URDF

**Steps:**
1. Clean up scene: Remove any invalid joints/links from tests
2. Ensure robot is valid (run validation)
3. In Export panel:
   - Format: **URDF**
   - Robot Name: `test_robot`
   - Export Meshes: **ON**
   - Mesh Format: **STL**
4. Choose output directory (e.g., Desktop or Documents)
5. Click **Export URDF**

**Expected:**
- Export completes without errors
- Output directory contains:
  - `test_robot.urdf`
  - `meshes/` folder with STL files
- Console shows export progress
- Success message

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

**Save the export location for later testing!**

### 8.2 Verify URDF Content

**Steps:**
1. Open `test_robot.urdf` in text editor
2. Check XML structure:
   - `<robot name="test_robot">` root element
   - Multiple `<link>` elements with names matching Blender
   - Multiple `<joint>` elements
   - `<sensor>` elements (if sensors added)
   - `<transmission>` elements (if transmissions added)
   - Material colors present
   - Inertia tensors present

**Expected:**
- Valid XML structure
- All robot components exported
- Mesh paths reference `meshes/` folder

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 8.3 Export to XACRO

**Steps:**
1. In Export panel:
   - Format: **XACRO**
   - Robot Name: `test_robot_xacro`
   - Export Meshes: **ON**
   - Mesh Format: **DAE (COLLADA)**
2. Choose output directory
3. Click **Export XACRO**

**Expected:**
- Export completes
- Output directory contains:
  - `test_robot_xacro.urdf.xacro`
  - `meshes/` folder with DAE files
- Success message

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 8.4 Verify XACRO Content

**Steps:**
1. Open `test_robot_xacro.urdf.xacro` in text editor
2. Check XACRO structure:
   - `<?xml version="1.0"?>` header
   - `<robot xmlns:xacro="...">` root with xacro namespace
   - XACRO properties (e.g., `<xacro:property name="...">`)
   - Potential XACRO macros
   - Links and joints

**Expected:**
- Valid XACRO file
- XACRO-specific elements present
- Can be processed by xacro tool

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 8.5 Test Export Without Meshes

**Steps:**
1. In Export panel:
   - Format: URDF
   - Export Meshes: **OFF**
2. Export to new directory
3. Check output

**Expected:**
- Only URDF file exported
- No meshes/ folder
- Geometry uses primitives (box, cylinder, sphere) instead of mesh references

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

---

## Test 9: Import (URDF & XACRO)

### 9.1 Import URDF File

**Steps:**
1. **Save current work** (File → Save As)
2. Start **new Blender file** (File → New → General)
3. Press N → LinkForge tab
4. In Import section, click **Import Robot**
5. Navigate to previously exported `test_robot.urdf`
6. Select and import

**Expected:**
- Robot imported into Blender
- All links appear as objects
- All joints appear as Empty objects with axes
- Materials applied (colors)
- Kinematic hierarchy preserved
- No errors in console

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 9.2 Verify Imported Properties

**Steps:**
1. Select `base_link` object
2. Check Links panel:
   - Link name matches
   - Mass matches original (5.0)
   - Inertia values present
   - Geometry type correct
3. Select `base_to_cylinder` joint
4. Check Joint panel:
   - Joint type: Revolute
   - Limits match (-1.57 to 1.57)
   - Dynamics match (if exported)

**Expected:**
- All properties correctly imported
- Values match original scene

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 9.3 Verify Sensors Imported

**Steps:**
1. Select link that had camera sensor
2. In Sensors panel, check sensor list
3. Verify sensor properties

**Expected:**
- All sensors imported
- Sensor properties match original

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 9.4 Verify Transmissions Imported

**Steps:**
1. In Transmissions panel, check transmission list
2. Verify transmission properties

**Expected:**
- All transmissions imported
- Properties match original

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 9.5 Import XACRO File

**Steps:**
1. Start new Blender file
2. Import `test_robot_xacro.urdf.xacro`
3. Verify import succeeds

**Expected:**
- XACRO file processed automatically
- Robot imported successfully
- All components present

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 9.6 Import Example URDF Files

**Test with provided examples:**

#### Import robot_arm.urdf

**Steps:**
1. New Blender file
2. Import `examples/robot_arm.urdf`
3. Inspect robot

**Expected:**
- Robot arm imported
- Multiple revolute joints
- Kinematic chain visible

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

#### Import mobile_robot.urdf

**Steps:**
1. New Blender file
2. Import `examples/mobile_robot.urdf`
3. Inspect robot

**Expected:**
- Mobile robot imported
- Continuous joints (wheels)
- Fixed joints (sensors)

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

---

## Test 10: Round-Trip Testing

**Critical test: Export → Import → Export → Compare**

### 10.1 Round-Trip URDF

**Steps:**
1. Open original robot scene (with all features)
2. Export as `roundtrip_v1.urdf` with meshes
3. New Blender file
4. Import `roundtrip_v1.urdf`
5. Export as `roundtrip_v2.urdf` with meshes
6. Compare files in text editor or diff tool

**Expected:**
- Files nearly identical (minor formatting differences OK)
- All link names, joint names match
- All properties preserved
- No data loss

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 10.2 Round-Trip XACRO

**Steps:**
1. Same test but with XACRO format
2. Export → Import → Export
3. Compare files

**Expected:**
- XACRO properties preserved
- No data loss
- Structure consistent

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

---

## Test 11: Visual Features

### 11.1 Joint Axes Visualization

**Steps:**
1. In scene with multiple joints
2. Select a joint Empty object
3. Observe RGB axes in viewport:
   - **Red arrow**: X-axis
   - **Green arrow**: Y-axis
   - **Blue arrow**: Z-axis (RViz convention)

**Expected:**
- All three axes visible
- Colors correct (RGB = XYZ)
- Arrows point in correct directions
- Axes scale appropriately

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 11.2 Toggle Joint Axes

**Steps:**
1. In extension preferences (Edit → Preferences → Get Extensions → LinkForge)
2. Find joint axis display settings
3. Toggle visibility or adjust size
4. Observe changes in viewport

**Expected:**
- Joint axes respond to preference changes
- Can hide/show axes
- Can adjust axis size

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 11.3 Joint Highlighting

**Steps:**
1. Select different joints in outliner
2. Observe viewport highlighting

**Expected:**
- Selected joint highlighted
- Easy to identify which joint is selected
- Axes remain visible

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

---

## Test 12: Edge Cases & Error Handling

### 12.1 Empty Robot Name

**Steps:**
1. In Export panel, leave Robot Name blank
2. Try to export

**Expected:**
- Uses default name or scene name
- OR shows error message
- No crash

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 12.2 Invalid File Path

**Steps:**
1. Try to import from non-existent file path
2. Or provide invalid URDF file

**Expected:**
- Clear error message
- No crash
- Helpful error description

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 12.3 Duplicate Link Names

**Steps:**
1. Create two links with same name
2. Try to export or validate

**Expected:**
- Validation catches duplicate names
- Error message displayed
- Export prevented or names auto-renamed

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 12.4 Joint Without Parent or Child

**Steps:**
1. Create joint
2. Leave parent or child as (none)
3. Validate or export

**Expected:**
- Validation error
- Clear message about missing parent/child
- Export prevented

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 12.5 Negative Mass

**Steps:**
1. Try to set link mass to negative value
2. Observe behavior

**Expected:**
- Property validation prevents negative mass
- OR warning message shown
- No crash

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 12.6 Invalid Joint Limits

**Steps:**
1. Set joint lower limit > upper limit
2. Validate or export

**Expected:**
- Validation warning or error
- Helpful message
- Export may proceed with warning

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

---

## Test 13: Performance & Large Scenes

### 13.1 Many Links (20+)

**Steps:**
1. Create robot with 20+ links
2. Mark all as links with auto-inertia
3. Create joints between them
4. Validate
5. Export

**Expected:**
- No significant slowdown
- Validation completes in reasonable time (<5 seconds)
- Export succeeds
- UI remains responsive

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 13.2 Complex Mesh Geometry

**Steps:**
1. Import high-poly mesh (100k+ vertices)
2. Mark as link with mesh geometry
3. Enable auto-calculate inertia

**Expected:**
- Inertia calculation may be slow but completes
- Progress indication or message
- No crash
- Reasonable performance (<30 seconds)

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

---

## Test 14: Blender Integration

### 14.1 Undo/Redo

**Steps:**
1. Create link
2. Press Ctrl+Z (undo)
3. Press Ctrl+Shift+Z (redo)

**Expected:**
- Undo removes link properties
- Redo restores them
- No corruption

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 14.2 Save and Load Blender File

**Steps:**
1. Create complete robot
2. Save as .blend file
3. Close Blender
4. Reopen Blender
5. Open saved file
6. Verify all LinkForge properties intact

**Expected:**
- All link properties preserved
- All joint properties preserved
- Sensors and transmissions intact
- No data loss

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 14.3 Multiple 3D Viewports

**Steps:**
1. Split viewport (drag corner)
2. Have LinkForge panel open in both
3. Select objects in different viewports
4. Modify properties

**Expected:**
- Both panels update correctly
- No synchronization issues
- Properties consistent

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

---

## Test 15: Console & Logging

### 15.1 Check Console Output

**Steps:**
1. On Windows: Window → Toggle System Console
2. On Mac/Linux: Launch Blender from terminal
3. Perform various operations
4. Monitor console for errors/warnings

**Expected:**
- No Python errors or tracebacks
- Informational messages only
- No unexpected warnings

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

### 15.2 Check Blender Info Window

**Steps:**
1. Change one area to Info editor
2. Perform operations
3. Check info messages

**Expected:**
- Success messages for operations
- Clear error messages if problems
- Useful information logged

**Status:** [ ] Pass [ ] Fail [ ] Notes: _______________

---

## Testing Summary

### Overall Results

**Total Tests:** ~75+ individual test cases
**Passed:** ___ / ___
**Failed:** ___ / ___
**Skipped:** ___ / ___

### Critical Issues Found

1. _______________________________________________
2. _______________________________________________
3. _______________________________________________

### Minor Issues Found

1. _______________________________________________
2. _______________________________________________
3. _______________________________________________

### Performance Notes

- Validation speed: _______________
- Export speed: _______________
- Import speed: _______________
- UI responsiveness: _______________

### Recommendations

1. _______________________________________________
2. _______________________________________________
3. _______________________________________________

---

## Post-Testing Actions

### If All Tests Pass:

1. ✅ Bump version to 0.5.0
2. ✅ Update CHANGELOG.md
3. ✅ Create GitHub release
4. ✅ Consider Blender Extensions submission

### If Issues Found:

1. 🐛 Document all bugs in GitHub Issues
2. 🔧 Prioritize fixes
3. 🔄 Re-test after fixes
4. 📝 Update documentation if needed

---

## Quick Reference: Common Issues

| Issue | Likely Cause | Solution |
|-------|-------------|----------|
| Import fails | Invalid URDF XML | Check XML syntax, validate with xmllint |
| Export crash | Invalid robot structure | Run validation first |
| Missing textures | Mesh export path issue | Check mesh export directory |
| Properties not saved | Blender file not saved | Save .blend file |
| Axes not visible | Display settings | Check preferences |
| Slow inertia calc | High-poly mesh | Use primitive geometry or simplify mesh |

---

**Document Version:** 1.0
**Last Updated:** 2025-11-01
**Testing Date:** _______________
**Tested By:** _______________
**Blender Version:** _______________
**LinkForge Version:** _______________
