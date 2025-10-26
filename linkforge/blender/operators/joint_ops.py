"""Operators for managing robot joints."""

from __future__ import annotations

try:
    import bpy
    from bpy.types import Operator
    from mathutils import Vector
except ImportError:
    # Allow importing without Blender
    bpy = None  # type: ignore
    Operator = object
    Vector = None  # type: ignore


class LINKFORGE_OT_create_joint(Operator):
    """Create a new robot joint at 3D cursor"""

    bl_idname = "linkforge.create_joint"
    bl_label = "Create Joint"
    bl_description = "Create a new robot joint (Empty) at the 3D cursor location"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        """Execute the operator."""
        # Create Empty at cursor location
        bpy.ops.object.empty_add(type="ARROWS", location=context.scene.cursor.location)
        joint_empty = context.active_object
        joint_empty.name = "Joint"

        # Enable joint properties
        joint_empty.linkforge_joint.is_robot_joint = True
        joint_empty.linkforge_joint.joint_name = joint_empty.name.replace(".", "_")

        # Set default joint type
        joint_empty.linkforge_joint.joint_type = "REVOLUTE"

        # Try to auto-detect parent/child from selection
        selected_links = [
            obj
            for obj in context.selected_objects
            if obj != joint_empty and obj.linkforge.is_robot_link
        ]

        if len(selected_links) >= 1:
            joint_empty.linkforge_joint.parent_link = selected_links[0].linkforge.link_name
        if len(selected_links) >= 2:
            joint_empty.linkforge_joint.child_link = selected_links[1].linkforge.link_name

        self.report({"INFO"}, f"Created joint '{joint_empty.name}'")
        return {"FINISHED"}


class LINKFORGE_OT_remove_joint(Operator):
    """Remove robot joint marking from selected Empty (keeps the Empty)"""

    bl_idname = "linkforge.remove_joint"
    bl_label = "Remove Joint Marking"
    bl_description = "Remove robot joint marking (Empty will not be deleted)"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        """Check if operator can run."""
        obj = context.active_object
        return obj is not None and obj.type == "EMPTY" and obj.linkforge_joint.is_robot_joint

    def execute(self, context):
        """Execute the operator."""
        obj = context.active_object

        # Disable joint properties (Empty is not deleted)
        obj.linkforge_joint.is_robot_joint = False

        self.report({"INFO"}, f"Removed joint marking from '{obj.name}' (Empty kept)")
        return {"FINISHED"}


class LINKFORGE_OT_auto_detect_parent_child(Operator):
    """Auto-detect parent and child links based on hierarchy"""

    bl_idname = "linkforge.auto_detect_parent_child"
    bl_label = "Auto-Detect Links"
    bl_description = "Automatically detect parent and child links from object hierarchy"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        """Check if operator can run."""
        obj = context.active_object
        return obj is not None and obj.type == "EMPTY" and obj.linkforge_joint.is_robot_joint

    def execute(self, context):
        """Execute the operator."""
        joint_empty = context.active_object
        props = joint_empty.linkforge_joint

        # Find nearest links based on distance
        joint_loc = joint_empty.location
        links = [
            (obj, (obj.location - joint_loc).length)
            for obj in context.scene.objects
            if obj.linkforge.is_robot_link
        ]

        if not links:
            self.report({"WARNING"}, "No robot links found in scene")
            return {"CANCELLED"}

        # Sort by distance
        links.sort(key=lambda x: x[1])

        # Force property update to refresh enum items
        bpy.context.view_layer.update()

        # Try to set parent and child links
        try:
            if len(links) >= 1:
                parent_name = links[0][0].linkforge.link_name
                props.parent_link = parent_name
                self.report({"INFO"}, f"Set parent: {parent_name}")
        except TypeError:
            # Fallback: enum validation failed, but continue
            self.report({"WARNING"}, "Could not set parent link in dropdown")

        try:
            if len(links) >= 2:
                child_name = links[1][0].linkforge.link_name
                props.child_link = child_name
                self.report({"INFO"}, f"Set child: {child_name}")
        except TypeError:
            # Fallback: enum validation failed, but continue
            self.report({"WARNING"}, "Could not set child link in dropdown")

        return {"FINISHED"}


# Registration
classes = [
    LINKFORGE_OT_create_joint,
    LINKFORGE_OT_remove_joint,
    LINKFORGE_OT_auto_detect_parent_child,
]


def register():
    """Register operators."""
    if bpy is not None:
        for cls in classes:
            bpy.utils.register_class(cls)


def unregister():
    """Unregister operators."""
    if bpy is not None:
        for cls in reversed(classes):
            bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
