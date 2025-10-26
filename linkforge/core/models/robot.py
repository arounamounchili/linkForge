"""Robot model representing a complete robot description."""

from __future__ import annotations

from dataclasses import dataclass, field

from .joint import Joint
from .link import Link


@dataclass
class Robot:
    """Complete robot description.

    A robot consists of links connected by joints, forming a kinematic tree.
    """

    name: str
    links: list[Link] = field(default_factory=list)
    joints: list[Joint] = field(default_factory=list)

    def __post_init__(self) -> None:
        """Validate robot."""
        if not self.name:
            raise ValueError("Robot name cannot be empty")

        # Validate naming convention
        if not all(c.isalnum() or c in ("_", "-") for c in self.name):
            raise ValueError(
                f"Robot name '{self.name}' contains invalid characters. "
                "Use only alphanumeric, underscore, or hyphen."
            )

    def add_link(self, link: Link) -> None:
        """Add a link to the robot."""
        if self.get_link(link.name) is not None:
            raise ValueError(f"Link '{link.name}' already exists")
        self.links.append(link)

    def add_joint(self, joint: Joint) -> None:
        """Add a joint to the robot."""
        if self.get_joint(joint.name) is not None:
            raise ValueError(f"Joint '{joint.name}' already exists")

        # Validate parent and child links exist
        if self.get_link(joint.parent) is None:
            raise ValueError(f"Parent link '{joint.parent}' not found")
        if self.get_link(joint.child) is None:
            raise ValueError(f"Child link '{joint.child}' not found")

        self.joints.append(joint)

    def get_link(self, name: str) -> Link | None:
        """Get link by name."""
        return next((link for link in self.links if link.name == name), None)

    def get_joint(self, name: str) -> Joint | None:
        """Get joint by name."""
        return next((joint for joint in self.joints if joint.name == name), None)

    def get_joints_for_link(self, link_name: str, as_parent: bool = True) -> list[Joint]:
        """Get all joints where the link is parent or child.

        Args:
            link_name: Name of the link
            as_parent: If True, get joints where link is parent; if False, where link is child

        Returns:
            List of matching joints
        """
        if as_parent:
            return [joint for joint in self.joints if joint.parent == link_name]
        else:
            return [joint for joint in self.joints if joint.child == link_name]

    def get_root_link(self) -> Link | None:
        """Get the root link of the kinematic tree.

        The root link is the one that is never a child in any joint.
        """
        if not self.links:
            return None

        child_links = {joint.child for joint in self.joints}
        root_links = [link for link in self.links if link.name not in child_links]

        if len(root_links) == 0:
            raise ValueError("No root link found (circular dependency detected)")
        if len(root_links) > 1:
            raise ValueError(f"Multiple root links found: {[link.name for link in root_links]}")

        return root_links[0]

    def validate_tree_structure(self) -> list[str]:
        """Validate the kinematic tree structure.

        Returns:
            List of error messages (empty if valid)
        """
        errors = []

        # Must have at least one link
        if not self.links:
            errors.append("Robot must have at least one link")
            return errors

        # Check for duplicate link names
        link_names = [link.name for link in self.links]
        if len(link_names) != len(set(link_names)):
            duplicates = [name for name in link_names if link_names.count(name) > 1]
            errors.append(f"Duplicate link names: {set(duplicates)}")

        # Check for duplicate joint names
        joint_names = [joint.name for joint in self.joints]
        if len(joint_names) != len(set(joint_names)):
            duplicates = [name for name in joint_names if joint_names.count(name) > 1]
            errors.append(f"Duplicate joint names: {set(duplicates)}")

        # Check that all joint parent/child links exist
        link_name_set = set(link_names)
        for joint in self.joints:
            if joint.parent not in link_name_set:
                errors.append(f"Joint '{joint.name}': parent link '{joint.parent}' not found")
            if joint.child not in link_name_set:
                errors.append(f"Joint '{joint.name}': child link '{joint.child}' not found")

        # Check for cycles in the kinematic tree
        if self._has_cycle():
            errors.append("Kinematic tree contains a cycle")

        # Check that there's exactly one root link
        root = None
        try:
            root = self.get_root_link()
            if root is None:
                errors.append("No root link found")
        except ValueError as e:
            errors.append(str(e))

        # Check that each link (except root) is a child in exactly one joint
        child_counts = {}
        for joint in self.joints:
            child_counts[joint.child] = child_counts.get(joint.child, 0) + 1

        for link in self.links:
            count = child_counts.get(link.name, 0)
            # Only check connectivity if we have a valid root
            if root is not None and link != root and count == 0:
                errors.append(f"Link '{link.name}' is not connected to the tree")
            elif count > 1:
                errors.append(f"Link '{link.name}' has {count} parent joints (should have 1)")

        return errors

    def _has_cycle(self) -> bool:
        """Check if the kinematic tree has a cycle using DFS."""
        if not self.joints:
            return False

        # Build adjacency list
        graph: dict[str, list[str]] = {}
        for joint in self.joints:
            if joint.parent not in graph:
                graph[joint.parent] = []
            graph[joint.parent].append(joint.child)

        visited = set()
        rec_stack = set()

        def dfs(node: str) -> bool:
            visited.add(node)
            rec_stack.add(node)

            if node in graph:
                for neighbor in graph[node]:
                    if neighbor not in visited:
                        if dfs(neighbor):
                            return True
                    elif neighbor in rec_stack:
                        return True

            rec_stack.remove(node)
            return False

        # Check from all nodes
        return any(link.name not in visited and dfs(link.name) for link in self.links)

    @property
    def total_mass(self) -> float:
        """Calculate total mass of the robot."""
        return sum(link.mass for link in self.links)

    @property
    def degrees_of_freedom(self) -> int:
        """Calculate total degrees of freedom (actuated joints only)."""
        return sum(joint.degrees_of_freedom for joint in self.joints)

    def __str__(self) -> str:
        """String representation."""
        return (
            f"Robot(name={self.name}, "
            f"links={len(self.links)}, "
            f"joints={len(self.joints)}, "
            f"dof={self.degrees_of_freedom})"
        )
