"""Template loader and registry for LinkForge.

This module manages the available robot templates and provides
functions to discover and retrieve them.
"""

from __future__ import annotations

from collections.abc import Sequence

from .template import RobotTemplate

# Global template registry
_TEMPLATE_REGISTRY: dict[str, RobotTemplate] = {}


def register_template(template: RobotTemplate) -> None:
    """Register a template in the global registry.

    Args:
        template: The template to register

    Raises:
        ValueError: If a template with the same ID already exists
    """
    if template.id in _TEMPLATE_REGISTRY:
        raise ValueError(f"Template with id '{template.id}' already registered")
    _TEMPLATE_REGISTRY[template.id] = template


def get_template(template_id: str) -> RobotTemplate | None:
    """Get a template by its ID.

    Args:
        template_id: The template ID to look up

    Returns:
        The template if found, None otherwise
    """
    return _TEMPLATE_REGISTRY.get(template_id)


def get_all_templates() -> Sequence[RobotTemplate]:
    """Get all registered templates.

    Returns:
        List of all registered templates, sorted by name
    """
    return sorted(_TEMPLATE_REGISTRY.values(), key=lambda t: t.name)


def get_templates_by_category(category: str) -> Sequence[RobotTemplate]:
    """Get all templates in a specific category.

    Args:
        category: The category to filter by (e.g., "arm", "mobile")

    Returns:
        List of templates in the specified category, sorted by name
    """
    templates = [t for t in _TEMPLATE_REGISTRY.values() if t.category == category]
    return sorted(templates, key=lambda t: t.name)


def get_template_categories() -> Sequence[str]:
    """Get all unique template categories.

    Returns:
        List of unique categories, sorted alphabetically
    """
    categories = {t.category for t in _TEMPLATE_REGISTRY.values()}
    return sorted(categories)


def clear_registry() -> None:
    """Clear all registered templates.

    This is primarily useful for testing.
    """
    _TEMPLATE_REGISTRY.clear()
