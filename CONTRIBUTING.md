# Contributing to LinkForge

Thank you for your interest in contributing to LinkForge! This guide will help you get started.

## Development Setup

### Prerequisites

- **Python 3.11+**
- **Blender 4.2+** (for testing)
- **Git**

### Quick Start

1. **Fork and clone the repository**:
   ```bash
   git clone https://github.com/YOUR_USERNAME/linkforge.git
   cd linkforge
   ```

2. **Create a virtual environment**:
   ```bash
   python3 -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. **Install development dependencies**:
   ```bash
   pip install -e ".[dev]"
   ```

4. **Run tests to verify setup**:
   ```bash
   pytest
   ```

## Testing Your Changes

### Run Tests

```bash
# Run all tests
pytest

# Run with coverage report
pytest --cov=linkforge

# Run specific test file
pytest tests/core/test_robot.py
```

### Code Quality Checks

Before submitting a PR, ensure your code passes all checks:

```bash
# Linting
ruff check linkforge tests

# Formatting
black linkforge tests

# Type checking
mypy linkforge
```

### Testing in Blender

1. **Build the extension**:
   ```bash
   python build_extension.py
   ```

2. **Install in Blender**:
   - Open Blender 4.2+
   - Edit → Preferences → Get Extensions
   - Dropdown menu (⌄) → Install from Disk
   - Select `dist/linkforge-{version}.zip`

3. **Test your changes** in Blender, then rebuild and reinstall to test updates

## Code Guidelines

### Architecture

LinkForge uses a **three-layer architecture**:

- **Core Layer** (`linkforge/core/`) - Pure Python, Blender-independent, fully testable
- **Adapter Layer** (`linkforge/blender/properties/`, `utils/converters.py`) - Bridges Blender and core
- **UI Layer** (`linkforge/blender/operators/`, `panels/`) - User interaction

When adding features, keep core logic in the `core/` module so it can be unit tested without Blender.

### Code Style

- **Type hints**: Required for all functions
- **Docstrings**: Required for all public functions/classes
- **Formatting**: Use `black` (line length: 100)
- **Imports**: Use `from __future__ import annotations`

Example:
```python
from __future__ import annotations

def calculate_inertia(mass: float, dimensions: Vector3) -> InertiaTensor:
    """Calculate inertia tensor for a box geometry.

    Args:
        mass: Mass in kilograms
        dimensions: Box dimensions (length, width, height)

    Returns:
        Calculated inertia tensor
    """
    # Implementation here
```

### Testing

- Write unit tests for all core logic
- Aim for >80% code coverage
- Use descriptive test names

```python
def test_robot_validates_duplicate_link_names():
    """Test that Robot rejects duplicate link names."""
    link1 = Link(name="base")
    link2 = Link(name="base")

    with pytest.raises(ValidationError, match="Duplicate link name"):
        Robot(name="test", links=[link1, link2])
```

## Submitting Changes

1. **Create a feature branch**:
   ```bash
   git checkout -b feature/my-feature
   ```

2. **Make your changes**:
   - Write code following the guidelines above
   - Add/update tests
   - Update documentation if needed

3. **Run all quality checks**:
   ```bash
   pytest
   ruff check linkforge tests
   black linkforge tests
   mypy linkforge
   ```

4. **Commit your changes**:
   ```bash
   git add .
   git commit -m "Add feature: brief description"
   ```

5. **Push to your fork**:
   ```bash
   git push origin feature/my-feature
   ```

6. **Open a Pull Request**:
   - Go to https://github.com/arounamounchili/linkforge
   - Click "Pull Requests" → "New Pull Request"
   - Select your fork and branch
   - Provide a clear description of your changes
   - Include screenshots/videos for UI changes

## Need Help?

- **Issues**: https://github.com/arounamounchili/linkforge/issues
- **Discussions**: https://github.com/arounamounchili/linkforge/discussions

## License

By contributing to LinkForge, you agree that your contributions will be licensed under the GPL-3.0-or-later license.

---

Thank you for contributing to LinkForge!
