"""Converters between Blender properties and Core models.

These functions bridge the gap between Blender's property system
and LinkForge's core data models.
"""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    # Type stubs for Blender types when type checking
    bpy: Any
    Matrix: Any
    Vector: Any
else:
    try:
        import bpy
        from mathutils import Matrix, Vector
    except ImportError:
        # Allow importing without Blender
        bpy = None  # type: ignore
        Matrix = Vector = None  # type: ignore

from ...core.models import (
    Box,
    CameraInfo,
    Capsule,
    Collision,
    Color,
    Cylinder,
    GazeboPlugin,
    Geometry,
    GPSInfo,
    HardwareInterface,
    IMUInfo,
    Inertial,
    InertiaTensor,
    Joint,
    JointDynamics,
    JointLimits,
    JointMimic,
    JointType,
    LidarInfo,
    Link,
    Material,
    Mesh,
    Robot,
    Sensor,
    SensorNoise,
    SensorType,
    Sphere,
    Transform,
    Transmission,
    TransmissionActuator,
    TransmissionJoint,
    TransmissionType,
    Vector3,
    Visual,
)
from ...core.physics import calculate_inertia, calculate_mesh_inertia_from_triangles


def blender_to_vector3(vec: Any) -> Vector3:
    """Convert Blender Vector to Vector3.

    Args:
        vec: Blender mathutils.Vector

    Returns:
        Core Vector3
    """
    return Vector3(vec.x, vec.y, vec.z)


def clean_float(value: float, epsilon: float = 1e-10) -> float:
    """Clean up floating point values to avoid -0.0 and very small numbers.

    Args:
        value: Float value to clean
        epsilon: Threshold below which values become 0.0

    Returns:
        Cleaned float value
    """
    if abs(value) < epsilon:
        return 0.0
    return value


def matrix_to_transform(matrix: Any) -> Transform:
    """Convert Blender 4x4 matrix to Transform.

    Args:
        matrix: Blender mathutils.Matrix (4x4)

    Returns:
        Core Transform with XYZ position and RPY rotation
    """
    if matrix is None or Matrix is None:
        return Transform.identity()

    # Extract translation
    translation = matrix.to_translation()
    xyz = Vector3(
        clean_float(translation.x),
        clean_float(translation.y),
        clean_float(translation.z),
    )

    # Extract rotation (Euler angles in radians)
    rotation = matrix.to_euler("XYZ")
    rpy = Vector3(
        clean_float(rotation.x),
        clean_float(rotation.y),
        clean_float(rotation.z),
    )

    return Transform(xyz=xyz, rpy=rpy)


def get_object_geometry(
    obj: Any,
    geometry_type: str = "MESH",
    link_name: str | None = None,
    geom_purpose: str = "visual",
    meshes_dir: Path | None = None,
    mesh_format: str = "STL",
    simplify: bool = False,
    decimation_ratio: float = 0.5,
) -> Geometry | None:
    """Extract geometry from Blender object.

    Args:
        obj: Blender Object
        geometry_type: Type of geometry to extract (MESH, BOX, CYLINDER, SPHERE, CAPSULE)
        link_name: Name of the link (for mesh filename)
        geom_purpose: "visual" or "collision" (for mesh filename)
        meshes_dir: Directory to export mesh files to
        mesh_format: "STL" or "DAE"
        simplify: Whether to simplify mesh (for collision)
        decimation_ratio: Simplification ratio if simplify=True

    Returns:
        Core Geometry or None
    """
    if bpy is None or obj is None:
        return None

    if geometry_type == "MESH":
        # Export actual mesh file if meshes_dir is provided
        if meshes_dir and link_name and obj.type == "MESH":
            from .mesh_export import export_link_mesh

            mesh_path = export_link_mesh(
                obj=obj,
                link_name=link_name,
                geometry_type=geom_purpose,
                mesh_format=mesh_format,
                meshes_dir=meshes_dir,
                simplify=simplify,
                decimation_ratio=decimation_ratio,
            )

            if mesh_path:
                # Return Mesh geometry with file path
                return Mesh(filepath=mesh_path, scale=Vector3(1.0, 1.0, 1.0))

        # Fallback: approximate with bounding box if export failed or not requested
        geometry_type = "BOX"

    if geometry_type == "BOX":
        # Use bounding box dimensions
        dimensions = obj.dimensions
        return Box(size=Vector3(dimensions.x, dimensions.y, dimensions.z))

    elif geometry_type == "CYLINDER":
        # Approximate with bounding cylinder
        dimensions = obj.dimensions
        radius = max(dimensions.x, dimensions.y) / 2.0
        length = dimensions.z
        return Cylinder(radius=radius, length=length)

    elif geometry_type == "SPHERE":
        # Approximate with bounding sphere
        radius = max(obj.dimensions) / 2.0
        return Sphere(radius=radius)

    elif geometry_type == "CAPSULE":
        # Approximate with bounding capsule
        dimensions = obj.dimensions
        radius = max(dimensions.x, dimensions.y) / 2.0
        length = dimensions.z - 2 * radius  # Cylinder portion length
        return Capsule(radius=radius, length=max(0.0, length))

    return None


def extract_mesh_triangles(
    obj: Any,
) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]]] | None:
    """Extract triangle mesh data from Blender object.

    Args:
        obj: Blender mesh object

    Returns:
        Tuple of (vertices, triangles) or None if not a mesh
        vertices: List of (x, y, z) coordinates
        triangles: List of (v0, v1, v2) triangle vertex indices
    """
    if bpy is None or obj is None or obj.type != "MESH":
        return None

    # Get evaluated mesh (with modifiers applied)
    depsgraph = bpy.context.evaluated_depsgraph_get()
    eval_obj = obj.evaluated_get(depsgraph)
    mesh = eval_obj.to_mesh()

    if mesh is None:
        return None

    # Ensure mesh has triangulated faces
    mesh.calc_loop_triangles()

    # Extract vertices in object space
    vertices = [(v.co.x, v.co.y, v.co.z) for v in mesh.vertices]

    # Extract triangles
    triangles = [(tri.vertices[0], tri.vertices[1], tri.vertices[2]) for tri in mesh.loop_triangles]

    # Clean up temporary mesh
    eval_obj.to_mesh_clear()

    return vertices, triangles


def get_object_material(obj: Any, props: Any) -> Material | None:
    """Extract material from Blender object.

    Args:
        obj: Blender Object
        props: LinkPropertyGroup with material settings

    Returns:
        Core Material or None
    """
    if not props.use_material:
        return None

    # Determine material name
    mat_name = props.material_name if props.material_name else f"{obj.name}_material"

    # Get color based on source
    if props.material_source == "CUSTOM":
        # Use custom color picker
        color = Color(
            r=props.material_color[0],
            g=props.material_color[1],
            b=props.material_color[2],
            a=props.material_color[3],
        )
    else:
        # Extract from Blender material
        if obj.material_slots and obj.material_slots[0].material:
            blender_mat = obj.material_slots[0].material

            # Try to get color from Principled BSDF node (modern Blender)
            color = None
            if blender_mat.use_nodes and blender_mat.node_tree:
                # Find Principled BSDF node
                for node in blender_mat.node_tree.nodes:
                    if node.type == "BSDF_PRINCIPLED":
                        # Get Base Color input (index 0)
                        base_color_input = node.inputs["Base Color"]
                        base_color = base_color_input.default_value
                        color = Color(
                            r=base_color[0],
                            g=base_color[1],
                            b=base_color[2],
                            a=base_color[3] if len(base_color) > 3 else 1.0,
                        )
                        break

            # Fallback to viewport display color if no node found
            if color is None:
                diffuse = blender_mat.diffuse_color
                color = Color(r=diffuse[0], g=diffuse[1], b=diffuse[2], a=diffuse[3])
        else:
            # No Blender material, use default gray
            color = Color(0.8, 0.8, 0.8, 1.0)

    return Material(name=mat_name, color=color)


def blender_link_to_core_with_origin(
    obj: Any,
    visual_origin: Transform,
    meshes_dir: Path | None = None,
    robot_props: Any = None,
) -> Link | None:
    """Convert Blender object with LinkPropertyGroup to Core Link.

    Args:
        obj: Blender Object with linkforge property group
        visual_origin: Pre-calculated visual/collision origin transform
        meshes_dir: Optional directory for exporting mesh files
        robot_props: Robot property group with export settings

    Returns:
        Core Link model or None
    """
    if bpy is None or obj is None:
        return None

    props = obj.linkforge
    if not props.is_robot_link:
        return None

    link_name = props.link_name if props.link_name else obj.name

    # Extract material
    material = get_object_material(obj, props)

    # Get mesh format from robot props
    mesh_format = "STL"
    if robot_props and hasattr(robot_props, "mesh_format"):
        mesh_format = robot_props.mesh_format

    # Visual geometry
    visual = None
    if props.use_visual_geometry:
        visual_geom = get_object_geometry(
            obj=obj,
            geometry_type=props.visual_geometry_type,
            link_name=link_name,
            geom_purpose="visual",
            meshes_dir=meshes_dir,
            mesh_format=mesh_format,
            simplify=False,  # Don't simplify visual meshes
            decimation_ratio=1.0,
        )
        if visual_geom:
            visual = Visual(
                geometry=visual_geom,
                origin=visual_origin,
                material=material,
            )

    # Collision geometry
    collision = None
    collision_geom = None
    if props.export_collision:
        collision_geom = get_object_geometry(
            obj=obj,
            geometry_type=props.collision_geometry_type,
            link_name=link_name,
            geom_purpose="collision",
            meshes_dir=meshes_dir,
            mesh_format="STL",  # Always use STL for collision
            simplify=props.simplify_collision,
            decimation_ratio=props.collision_decimation_ratio,
        )
        if collision_geom:
            collision = Collision(geometry=collision_geom, origin=visual_origin)

    # Inertial properties
    inertial = None
    if props.mass > 0:
        if props.use_auto_inertia and collision_geom:
            # Auto-calculate inertia from geometry
            if isinstance(collision_geom, Mesh) and obj.type == "MESH":
                # Use accurate triangle-based mesh inertia calculation
                mesh_data = extract_mesh_triangles(obj)
                if mesh_data:
                    vertices, triangles = mesh_data
                    inertia_tensor = calculate_mesh_inertia_from_triangles(
                        vertices, triangles, props.mass
                    )
                else:
                    # Fallback to bounding box if mesh extraction fails
                    dimensions = obj.dimensions
                    bbox_geom = Box(size=Vector3(dimensions.x, dimensions.y, dimensions.z))
                    inertia_tensor = calculate_inertia(bbox_geom, props.mass)
            else:
                # Calculate from primitive geometry
                inertia_tensor = calculate_inertia(collision_geom, props.mass)
        else:
            # Use manual inertia
            inertia_tensor = InertiaTensor(
                ixx=props.inertia_ixx,
                ixy=props.inertia_ixy,
                ixz=props.inertia_ixz,
                iyy=props.inertia_iyy,
                iyz=props.inertia_iyz,
                izz=props.inertia_izz,
            )

        inertial = Inertial(
            mass=props.mass,
            origin=visual_origin,
            inertia=inertia_tensor,
        )

    return Link(
        name=link_name,
        visual=visual,
        collision=collision,
        inertial=inertial,
    )


def blender_joint_to_core(obj: Any, scene: Any) -> Joint | None:
    """Convert Blender Empty with JointPropertyGroup to Core Joint.

    Args:
        obj: Blender Empty object with linkforge_joint property group
        scene: Blender scene to find parent link object

    Returns:
        Core Joint model or None
    """
    if bpy is None or obj is None:
        return None

    props = obj.linkforge_joint
    if not props.is_robot_joint:
        return None

    joint_name = props.joint_name if props.joint_name else obj.name

    # Joint type
    joint_type = JointType(props.joint_type.lower())

    # Joint axis
    if props.axis == "X":
        axis = Vector3(1.0, 0.0, 0.0)
    elif props.axis == "Y":
        axis = Vector3(0.0, 1.0, 0.0)
    elif props.axis == "Z":
        axis = Vector3(0.0, 0.0, 1.0)
    else:  # CUSTOM
        axis = Vector3(props.custom_axis_x, props.custom_axis_y, props.custom_axis_z)

    # Joint origin is already calculated relative to parent in converters.scene_to_robot
    # Just use the joint's world transform here, will be made relative in scene_to_robot
    origin = matrix_to_transform(obj.matrix_world)

    # Joint limits
    limits = None
    if props.use_limits and joint_type in (JointType.REVOLUTE, JointType.PRISMATIC):
        limits = JointLimits(
            lower=props.limit_lower,
            upper=props.limit_upper,
            effort=props.limit_effort,
            velocity=props.limit_velocity,
        )

    # Dynamics
    dynamics = None
    if props.use_dynamics:
        dynamics = JointDynamics(
            damping=props.dynamics_damping,
            friction=props.dynamics_friction,
        )

    # Mimic
    mimic = None
    if props.use_mimic and props.mimic_joint:
        mimic = JointMimic(
            joint=props.mimic_joint,
            multiplier=props.mimic_multiplier,
            offset=props.mimic_offset,
        )

    # Handle "NONE" value for parent/child links (when no link is selected)
    parent = props.parent_link if props.parent_link != "NONE" else ""
    child = props.child_link if props.child_link != "NONE" else ""

    return Joint(
        name=joint_name,
        type=joint_type,
        parent=parent,
        child=child,
        origin=origin,
        axis=axis,
        limits=limits,
        dynamics=dynamics,
        mimic=mimic,
    )


def scene_to_robot(context: Any, meshes_dir: Path | None = None) -> Robot:
    """Convert entire Blender scene to Core Robot.

    Args:
        context: Blender context
        meshes_dir: Optional directory for exporting mesh files

    Returns:
        Core Robot model
    """
    if bpy is None or context is None:
        return Robot(name="empty_robot")

    scene = context.scene
    robot_props = scene.linkforge
    robot_name = robot_props.robot_name if robot_props.robot_name else "robot"

    robot = Robot(name=robot_name)

    # First pass: Find root link and build joint map
    root_link = None
    joints_map = {}  # child_link_name -> (parent_link_name, joint_empty_obj)

    for obj in scene.objects:
        if obj.type == "EMPTY" and obj.linkforge_joint.is_robot_joint:
            props = obj.linkforge_joint
            parent_name = props.parent_link if props.parent_link != "NONE" else ""
            child_name = props.child_link if props.child_link != "NONE" else ""
            if parent_name and child_name:
                joints_map[child_name] = (parent_name, obj)

    # Find root link (link with no parent joint)
    for obj in scene.objects:
        if obj.linkforge.is_robot_link:
            link_name = obj.linkforge.link_name if obj.linkforge.link_name else obj.name
            if link_name not in joints_map:
                root_link = (link_name, obj)
                break

    # Second pass: Calculate link frame positions recursively
    link_frames = {}  # link_name -> world matrix where link frame is

    if root_link and Matrix:
        root_name, root_obj = root_link
        # Root link frame is at origin (standard URDF convention)
        # Visual/collision will be offset by root object's world position
        link_frames[root_name] = Matrix.Identity(4)

        # Calculate child link frames
        def calc_child_frames(parent_name: str) -> None:
            for child_name, (parent, joint_obj) in joints_map.items():
                if parent == parent_name and child_name not in link_frames:
                    # Child frame = parent frame transformed by joint transform
                    parent_frame = link_frames[parent_name]
                    parent_inv = parent_frame.inverted()
                    joint_rel = parent_inv @ joint_obj.matrix_world
                    child_frame = parent_frame @ joint_rel
                    link_frames[child_name] = child_frame
                    calc_child_frames(child_name)

        calc_child_frames(root_name)

    # Third pass: Collect all links with correct visual origins
    for obj in scene.objects:
        if obj.linkforge.is_robot_link:
            link_name = obj.linkforge.link_name if obj.linkforge.link_name else obj.name

            # Calculate visual origin relative to link frame
            if link_name in link_frames and Matrix:
                link_frame = link_frames[link_name]
                link_frame_inv = link_frame.inverted()
                obj_relative = link_frame_inv @ obj.matrix_world
                visual_origin = matrix_to_transform(obj_relative)
            else:
                # Fallback: identity
                visual_origin = Transform.identity()

            # Create link with calculated visual origin
            link = blender_link_to_core_with_origin(obj, visual_origin, meshes_dir, robot_props)
            if link:
                try:
                    robot.add_link(link)
                except ValueError as e:
                    print(f"Warning: Could not add link {link.name}: {e}")

    # Fourth pass: Collect all joints with correct origins
    for obj in scene.objects:
        if obj.type == "EMPTY" and obj.linkforge_joint.is_robot_joint:
            joint = blender_joint_to_core(obj, scene)
            if joint:
                # Calculate joint origin relative to parent link frame
                parent_name = joint.parent
                if parent_name and parent_name in link_frames and Matrix:
                    parent_frame = link_frames[parent_name]
                    parent_frame_inv = parent_frame.inverted()
                    joint_relative = parent_frame_inv @ obj.matrix_world
                    corrected_origin = matrix_to_transform(joint_relative)
                    # Create new joint with corrected origin (Joint is frozen dataclass)
                    joint = replace(joint, origin=corrected_origin)

                try:
                    robot.add_joint(joint)
                except ValueError as e:
                    print(f"Warning: Could not add joint {joint.name}: {e}")

    # Fifth pass: Collect all sensors
    for obj in scene.objects:
        if obj.type == "EMPTY" and hasattr(obj, "linkforge_sensor"):
            sensor = blender_sensor_to_core(obj)
            if sensor:
                try:
                    robot.add_sensor(sensor)
                except ValueError as e:
                    print(f"Warning: Could not add sensor {sensor.name}: {e}")

    # Sixth pass: Collect all transmissions
    for obj in scene.objects:
        if obj.type == "EMPTY" and hasattr(obj, "linkforge_transmission"):
            transmission = blender_transmission_to_core(obj)
            if transmission:
                try:
                    robot.add_transmission(transmission)
                except ValueError as e:
                    print(f"Warning: Could not add transmission {transmission.name}: {e}")

    return robot


def blender_sensor_to_core(obj: Any) -> Sensor | None:
    """Convert Blender Empty with SensorPropertyGroup to Core Sensor.

    Args:
        obj: Blender Empty object with linkforge_sensor property group

    Returns:
        Core Sensor model or None
    """
    if bpy is None or obj is None:
        return None

    props = obj.linkforge_sensor
    if not props.is_robot_sensor:
        return None

    sensor_name = props.sensor_name if props.sensor_name else obj.name
    sensor_type = SensorType(props.sensor_type.lower())
    link_name = props.attached_link if props.attached_link != "NONE" else ""

    if not link_name:
        print(f"Warning: Sensor {sensor_name} has no attached link")
        return None

    # Build sensor origin from object transform
    origin = matrix_to_transform(obj.matrix_world)

    # Type-specific info
    camera_info = None
    lidar_info = None
    imu_info = None
    gps_info = None

    # Noise model
    noise = None
    if props.use_noise:
        noise = SensorNoise(
            type=props.noise_type,
            mean=props.noise_mean,
            stddev=props.noise_stddev,
        )

    # Camera info
    if sensor_type in (SensorType.CAMERA, SensorType.DEPTH_CAMERA):
        camera_info = CameraInfo(
            horizontal_fov=props.camera_horizontal_fov,
            width=props.camera_width,
            height=props.camera_height,
            near_clip=props.camera_near_clip,
            far_clip=props.camera_far_clip,
        )

    # LIDAR info
    elif sensor_type in (SensorType.LIDAR, SensorType.GPU_LIDAR):
        lidar_info = LidarInfo(
            horizontal_samples=props.lidar_horizontal_samples,
            horizontal_min_angle=props.lidar_horizontal_min_angle,
            horizontal_max_angle=props.lidar_horizontal_max_angle,
            vertical_samples=props.lidar_vertical_samples,
            range_min=props.lidar_range_min,
            range_max=props.lidar_range_max,
            noise=noise,
        )

    # IMU info
    elif sensor_type == SensorType.IMU:
        imu_info = IMUInfo(
            gravity_magnitude=props.imu_gravity_magnitude,
            angular_velocity_noise=noise,
            linear_acceleration_noise=noise,
        )

    # GPS info
    elif sensor_type == SensorType.GPS:
        gps_info = GPSInfo(
            position_sensing_horizontal_noise=noise,
            velocity_sensing_horizontal_noise=noise,
        )

    # Gazebo plugin
    plugin = None
    if props.use_gazebo_plugin and props.plugin_filename:
        plugin = GazeboPlugin(
            name=f"{sensor_name}_plugin",
            filename=props.plugin_filename,
            parameters={},
        )

    # Topic name
    topic = props.topic_name if props.topic_name else f"/{sensor_name}"

    return Sensor(
        name=sensor_name,
        type=sensor_type,
        link_name=link_name,
        origin=origin,
        update_rate=props.update_rate,
        camera_info=camera_info,
        lidar_info=lidar_info,
        imu_info=imu_info,
        gps_info=gps_info,
        plugin=plugin,
        topic=topic,
    )


def blender_transmission_to_core(obj: Any) -> Transmission | None:
    """Convert Blender Empty with TransmissionPropertyGroup to Core Transmission.

    Args:
        obj: Blender Empty object with linkforge_transmission property group

    Returns:
        Core Transmission model or None
    """
    if bpy is None or obj is None:
        return None

    props = obj.linkforge_transmission
    if not props.is_robot_transmission:
        return None

    transmission_name = props.transmission_name if props.transmission_name else obj.name

    # Transmission type mapping
    transmission_type_map = {
        "SIMPLE": TransmissionType.SIMPLE.value,
        "DIFFERENTIAL": TransmissionType.DIFFERENTIAL.value,
        "FOUR_BAR_LINKAGE": TransmissionType.FOUR_BAR_LINKAGE.value,
    }

    # Determine transmission type
    trans_type_str = props.transmission_type
    if trans_type_str == "CUSTOM":
        trans_type = props.custom_type if props.custom_type else "custom"
    else:
        trans_type = transmission_type_map.get(trans_type_str, TransmissionType.SIMPLE.value)

    # Hardware interface mapping
    hardware_interface_map = {
        "POSITION": HardwareInterface.COMMAND_POSITION.value,
        "VELOCITY": HardwareInterface.COMMAND_VELOCITY.value,
        "EFFORT": HardwareInterface.COMMAND_EFFORT.value,
        "POSITION_ROS1": HardwareInterface.POSITION.value,
        "VELOCITY_ROS1": HardwareInterface.VELOCITY.value,
        "EFFORT_ROS1": HardwareInterface.EFFORT.value,
    }

    # Determine hardware interface
    hardware_interface = hardware_interface_map.get(
        props.hardware_interface, HardwareInterface.COMMAND_POSITION.value
    )

    # Build joints and actuators based on type
    joints: list[TransmissionJoint] = []
    actuators: list[TransmissionActuator] = []

    if trans_type_str == "SIMPLE":
        joint_name = props.joint_name if props.joint_name != "NONE" else ""
        if not joint_name:
            print(f"Warning: Transmission {transmission_name} has no joint selected")
            return None

        # Actuator name
        if props.use_custom_actuator_name and props.actuator_name:
            actuator_name = props.actuator_name
        else:
            actuator_name = f"{joint_name}_motor"

        joints.append(
            TransmissionJoint(
                name=joint_name,
                hardware_interfaces=[hardware_interface],
                mechanical_reduction=props.mechanical_reduction,
                offset=props.offset,
            )
        )
        actuators.append(
            TransmissionActuator(
                name=actuator_name,
                hardware_interfaces=[hardware_interface],
            )
        )

    elif trans_type_str == "DIFFERENTIAL":
        joint1_name = props.joint1_name if props.joint1_name != "NONE" else ""
        joint2_name = props.joint2_name if props.joint2_name != "NONE" else ""

        if not joint1_name or not joint2_name:
            print(f"Warning: Differential transmission {transmission_name} needs 2 joints")
            return None

        # Actuator names
        if props.use_custom_actuator_name:
            actuator1_name = (
                props.actuator1_name if props.actuator1_name else f"{joint1_name}_motor"
            )
            actuator2_name = (
                props.actuator2_name if props.actuator2_name else f"{joint2_name}_motor"
            )
        else:
            actuator1_name = f"{joint1_name}_motor"
            actuator2_name = f"{joint2_name}_motor"

        joints.append(
            TransmissionJoint(
                name=joint1_name,
                hardware_interfaces=[hardware_interface],
                mechanical_reduction=props.mechanical_reduction,
                offset=props.offset,
            )
        )
        joints.append(
            TransmissionJoint(
                name=joint2_name,
                hardware_interfaces=[hardware_interface],
                mechanical_reduction=props.mechanical_reduction,
                offset=props.offset,
            )
        )
        actuators.append(
            TransmissionActuator(name=actuator1_name, hardware_interfaces=[hardware_interface])
        )
        actuators.append(
            TransmissionActuator(name=actuator2_name, hardware_interfaces=[hardware_interface])
        )

    return Transmission(
        name=transmission_name,
        type=trans_type,
        joints=joints,
        actuators=actuators,
    )
