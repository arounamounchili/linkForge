"""Blender event handlers for LinkForge.

Handlers respond to Blender events like object renaming, scene updates, etc.
"""

from __future__ import annotations

try:
    import bpy
    from bpy.app.handlers import persistent
except ImportError:
    # Allow importing without Blender
    bpy = None
    persistent = lambda f: f  # noqa: E731


# Store previous object names to detect changes
_previous_object_names = {}


@persistent
def sync_object_names(scene):
    """Handler to sync object names to link names when objects are renamed in Outliner.

    This runs after every depsgraph update.
    """
    if not bpy or not scene:
        return

    global _previous_object_names

    # Check each object for name changes
    for obj in scene.objects:
        # Skip if not a robot link
        if not hasattr(obj, "linkforge") or not obj.linkforge.is_robot_link:
            continue

        obj_id = obj.as_pointer()
        old_name = _previous_object_names.get(obj_id)
        current_name = obj.name

        # If name changed and it wasn't initiated by LinkForge (check link_name)
        if old_name is not None and old_name != current_name:
            # Only update link_name if it differs (avoid infinite loops)
            if obj.linkforge.link_name != current_name:
                # User renamed in Outliner, sync to link_name
                obj.linkforge.link_name = current_name

        # Update tracking
        _previous_object_names[obj_id] = current_name


def register():
    """Register handlers."""
    if bpy is not None:
        # Register depsgraph update handler
        if sync_object_names not in bpy.app.handlers.depsgraph_update_post:
            bpy.app.handlers.depsgraph_update_post.append(sync_object_names)


def unregister():
    """Unregister handlers."""
    if bpy is not None:
        # Unregister handler
        if sync_object_names in bpy.app.handlers.depsgraph_update_post:
            bpy.app.handlers.depsgraph_update_post.remove(sync_object_names)

        # Clear tracking dict
        global _previous_object_names
        _previous_object_names.clear()


if __name__ == "__main__":
    register()
