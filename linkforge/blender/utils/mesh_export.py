"""Mesh export utilities for LinkForge.

Export Blender mesh objects to STL and DAE files for URDF.
"""

from __future__ import annotations

from pathlib import Path

try:
    import bpy
except ImportError:
    bpy = None


def export_mesh_stl(obj, filepath: Path) -> bool:
    """Export Blender object to STL file.

    Args:
        obj: Blender Object to export
        filepath: Path where STL file should be saved

    Returns:
        True if export succeeded, False otherwise
    """
    if bpy is None or obj is None:
        return False

    # Ensure parent directory exists
    filepath.parent.mkdir(parents=True, exist_ok=True)

    # Deselect all and select only target object
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj

    # Export to STL
    try:
        bpy.ops.wm.stl_export(
            filepath=str(filepath),
            export_selected_objects=True,
            apply_modifiers=True,
            forward_axis='Y',
            up_axis='Z',
        )
        return True
    except Exception as e:
        print(f"STL export failed: {e}")
        return False


def export_mesh_obj(obj, filepath: Path) -> bool:
    """Export Blender object to OBJ file (with MTL material).

    Args:
        obj: Blender Object to export
        filepath: Path where OBJ file should be saved

    Returns:
        True if export succeeded, False otherwise
    """
    if bpy is None or obj is None:
        return False

    # Ensure parent directory exists
    filepath.parent.mkdir(parents=True, exist_ok=True)

    # Deselect all and select only target object
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj

    # Export to OBJ
    try:
        bpy.ops.wm.obj_export(
            filepath=str(filepath),
            export_selected_objects=True,
            apply_modifiers=True,
            export_materials=True,
            forward_axis='Y',
            up_axis='Z',
        )
        return True
    except Exception as e:
        print(f"OBJ export failed: {e}")
        return False


def create_simplified_mesh(obj, decimation_ratio: float):
    """Create a simplified copy of mesh using decimation.

    Args:
        obj: Blender Object to simplify
        decimation_ratio: Target ratio of faces to keep (0.0-1.0)

    Returns:
        Simplified Blender Object or None
    """
    if bpy is None or obj is None or obj.type != 'MESH':
        return None

    # Duplicate the object
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.duplicate()

    # Get the duplicated object
    simplified_obj = bpy.context.active_object

    # Add Decimate modifier
    decimate_mod = simplified_obj.modifiers.new(name="Decimate", type='DECIMATE')
    decimate_mod.ratio = decimation_ratio
    decimate_mod.decimate_type = 'COLLAPSE'

    # Apply the modifier
    bpy.context.view_layer.objects.active = simplified_obj
    bpy.ops.object.modifier_apply(modifier=decimate_mod.name)

    return simplified_obj


def get_mesh_filename(link_name: str, geometry_type: str, mesh_format: str) -> str:
    """Generate mesh filename based on link and geometry type.

    Args:
        link_name: Name of the robot link
        geometry_type: "visual" or "collision"
        mesh_format: "STL" or "DAE"

    Returns:
        Filename string (e.g., "base_link_visual.stl")
    """
    ext = mesh_format.lower()
    return f"{link_name}_{geometry_type}.{ext}"


def export_link_mesh(
    obj,
    link_name: str,
    geometry_type: str,
    mesh_format: str,
    meshes_dir: Path,
    simplify: bool = False,
    decimation_ratio: float = 0.5,
) -> Path | None:
    """Export mesh for a robot link.

    Args:
        obj: Blender Object to export
        link_name: Name of the robot link
        geometry_type: "visual" or "collision"
        mesh_format: "STL" or "DAE"
        meshes_dir: Directory where mesh files should be saved
        simplify: Whether to simplify mesh (for collision)
        decimation_ratio: Simplification ratio if simplify=True

    Returns:
        Path to exported mesh file, or None if export failed
    """
    if bpy is None or obj is None or obj.type != 'MESH':
        return None

    # Generate filename
    filename = get_mesh_filename(link_name, geometry_type, mesh_format)
    filepath = meshes_dir / filename

    # Determine which object to export
    export_obj = obj
    temp_obj = None

    if simplify and geometry_type == "collision":
        # Create simplified mesh for collision
        temp_obj = create_simplified_mesh(obj, decimation_ratio)
        if temp_obj:
            export_obj = temp_obj

    # Export based on format
    success = False
    if mesh_format.upper() == "STL":
        success = export_mesh_stl(export_obj, filepath)
    elif mesh_format.upper() == "OBJ":
        success = export_mesh_obj(export_obj, filepath)
    else:
        # Unknown format, default to OBJ
        print(f"Warning: Unknown mesh format '{mesh_format}', using OBJ")
        filepath = filepath.with_suffix('.obj')
        success = export_mesh_obj(export_obj, filepath)

    # Clean up temporary simplified object
    if temp_obj:
        bpy.data.objects.remove(temp_obj, do_unlink=True)

    return filepath if success else None
