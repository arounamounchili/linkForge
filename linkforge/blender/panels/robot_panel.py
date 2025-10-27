"""UI Panel for robot-level properties and validation."""

from __future__ import annotations

try:
    import bpy
    from bpy.types import Panel
except ImportError:
    # Allow importing without Blender
    bpy = None  # type: ignore
    Panel = object


def build_tree_structure(scene):
    """Build robot tree structure from scene objects.

    Returns:
        dict: Tree structure mapping parent links to children
    """
    # Collect all links
    links = {obj.linkforge.link_name: obj for obj in scene.objects if obj.linkforge.is_robot_link}

    # Build parent->children mapping from joints
    tree = {link_name: [] for link_name in links}
    root_link = None

    for obj in scene.objects:
        if obj.type == "EMPTY" and obj.linkforge_joint.is_robot_joint:
            props = obj.linkforge_joint
            parent = props.parent_link
            child = props.child_link

            if parent and child and parent in tree:
                tree[parent].append(child)

    # Find root (link with no parent)
    parents_with_children = set()
    all_children = set()
    for parent, children in tree.items():
        if children:
            parents_with_children.add(parent)
            all_children.update(children)

    # Root is a link that appears as parent but not as child
    for link_name in links:
        if link_name not in all_children:
            root_link = link_name
            break

    return tree, root_link


def draw_tree_recursive(layout, tree, node, depth=0, drawn=None):
    """Recursively draw tree structure.

    Args:
        layout: Blender layout object
        tree: Tree structure dict
        node: Current node name
        depth: Current depth in tree
        drawn: Set of already drawn nodes (to avoid cycles)
    """
    if drawn is None:
        drawn = set()

    if node in drawn:
        return
    drawn.add(node)

    # Draw current node with indentation
    indent = "  " * depth
    if depth == 0:
        icon = "OUTLINER_OB_ARMATURE"
        prefix = "└─"
    else:
        icon = "OUTLINER_OB_MESH"
        prefix = "└─"

    row = layout.row()
    row.label(text=f"{indent}{prefix} {node}", icon=icon)

    # Draw children
    if node in tree:
        for child in tree[node]:
            draw_tree_recursive(layout, tree, child, depth + 1, drawn)


class LINKFORGE_PT_import_panel(Panel):
    """Panel for import settings in 3D Viewport sidebar."""

    bl_label = "Import"
    bl_idname = "LINKFORGE_PT_import_panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "LinkForge"
    bl_order = 0

    def draw(self, context):
        """Draw the panel."""
        layout = self.layout

        # Import section
        box = layout.box()
        box.label(text="Import URDF:", icon="IMPORT")
        box.operator("linkforge.import_urdf", text="Import URDF/XACRO", icon="FILE_FOLDER")

        # Info text
        box.separator()
        col = box.column(align=True)
        col.label(text="Import robot models from", icon="INFO")
        col.label(text="URDF or XACRO files")


class LINKFORGE_PT_robot_panel(Panel):
    """Panel for global robot properties in 3D Viewport sidebar."""

    bl_label = "Robot"
    bl_idname = "LINKFORGE_PT_robot_panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "LinkForge"
    bl_order = 3

    def draw(self, context):
        """Draw the panel."""
        layout = self.layout
        scene = context.scene
        props = scene.linkforge

        # Robot identification
        box = layout.box()
        box.label(text="Robot Properties:", icon="ARMATURE_DATA")
        box.prop(props, "robot_name")

        # Count links and joints
        num_links = sum(1 for obj in scene.objects if obj.linkforge.is_robot_link)
        num_joints = sum(
            1 for obj in scene.objects if obj.type == "EMPTY" and obj.linkforge_joint.is_robot_joint
        )

        # Build tree structure to find root
        tree, root_link = build_tree_structure(scene)

        # Statistics
        box.separator()
        col = box.column(align=True)
        col.label(text="Structure:", icon="OUTLINER")
        row = col.row(align=True)
        row.label(text=f"Links: {num_links}")
        row.label(text=f"Joints: {num_joints}")

        # Show root link
        if root_link:
            row = col.row()
            row.label(text=f"Root: {root_link}", icon="ARMATURE_DATA")
        elif num_links > 0:
            row = col.row()
            row.label(text="Root: Not found", icon="ERROR")

        # Show tree structure if enabled
        box.separator()
        box.prop(props, "show_kinematic_tree", toggle=True, icon="OUTLINER")

        if props.show_kinematic_tree and root_link:
            tree_box = box.box()
            tree_box.label(text="Kinematic Tree:", icon="OUTLINER_OB_ARMATURE")
            draw_tree_recursive(tree_box, tree, root_link)

        # Visual helpers
        box.separator()
        box.label(text="Visualization:", icon="HIDE_OFF")
        box.prop(props, "show_joint_axes")


class LINKFORGE_PT_export_panel(Panel):
    """Panel for export settings in 3D Viewport sidebar."""

    bl_label = "Export"
    bl_idname = "LINKFORGE_PT_export_panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "LinkForge"
    bl_order = 4

    def draw(self, context):
        """Draw the panel."""
        layout = self.layout
        scene = context.scene
        props = scene.linkforge

        # Export format
        box = layout.box()
        box.label(text="Export Settings:", icon="EXPORT")
        box.prop(props, "export_format", expand=True)

        # Mesh export options
        box.separator()
        box.prop(props, "export_meshes")
        if props.export_meshes:
            box.prop(props, "mesh_format")
            box.prop(props, "mesh_directory_name")

        # Validation option
        box.separator()
        box.prop(props, "validate_before_export")

        # Export button
        box.separator()
        box.operator("linkforge.export_urdf", text="Export URDF/XACRO", icon="FILE_TICK")


# Registration
classes = [
    LINKFORGE_PT_import_panel,
    LINKFORGE_PT_robot_panel,
    LINKFORGE_PT_export_panel,
]


def register():
    """Register panels."""
    if bpy is not None:
        for cls in classes:
            bpy.utils.register_class(cls)


def unregister():
    """Unregister panels."""
    if bpy is not None:
        for cls in reversed(classes):
            bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
