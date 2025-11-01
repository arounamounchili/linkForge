#!/usr/bin/env python3
"""Build script for packaging LinkForge as a Blender Extension.

This script creates a .zip file that can be:
1. Uploaded to https://extensions.blender.org
2. Installed via drag-and-drop into Blender
3. Installed via Blender's Extensions preference panel

Usage:
    python build_extension.py

Output:
    dist/linkforge-{version}.zip
"""

from __future__ import annotations

import shutil
import tomllib
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile


def read_version() -> str:
    """Read version from blender_manifest.toml."""
    manifest_path = Path(__file__).parent / "blender_manifest.toml"
    with open(manifest_path, "rb") as f:
        manifest = tomllib.load(f)
    return manifest["version"]


def build_extension() -> Path:
    """Build the Blender Extension package.

    Returns:
        Path to the created .zip file
    """
    root_dir = Path(__file__).parent
    version = read_version()

    # Output directory and filename
    dist_dir = root_dir / "dist"
    dist_dir.mkdir(exist_ok=True)

    output_file = dist_dir / f"linkforge-{version}.zip"

    print(f"Building LinkForge Extension v{version}")
    print(f"Output: {output_file}")
    print()

    # Files and directories to include
    include_patterns = [
        "blender_manifest.toml",  # Required manifest
        "LICENSE",                 # Required for GPL
        "README.md",              # Documentation
        "linkforge/**/*.py",      # All Python code
    ]

    # Files and directories to exclude
    exclude_patterns = [
        "**/__pycache__/**",
        "**/*.pyc",
        "**/*.pyo",
        "**/.pytest_cache/**",
        "**/venv/**",
        "**/.*",  # Hidden files
        "tests/**",  # Don't include tests in extension
        "examples/**",  # Don't include examples in extension
        "docs/**",  # Don't include docs source in extension
    ]

    # Collect files to include
    files_to_package: list[Path] = []

    # Add manifest and LICENSE (required)
    files_to_package.append(root_dir / "blender_manifest.toml")
    files_to_package.append(root_dir / "LICENSE")
    files_to_package.append(root_dir / "README.md")

    # Add all Python files from linkforge/
    linkforge_dir = root_dir / "linkforge"
    for py_file in linkforge_dir.rglob("*.py"):
        # Check if file should be excluded
        relative_path = py_file.relative_to(root_dir)
        should_exclude = False

        for pattern in exclude_patterns:
            if relative_path.match(pattern):
                should_exclude = True
                break

        if not should_exclude:
            files_to_package.append(py_file)

    # Add wheel files if they exist (bundled dependencies)
    wheels_dir = root_dir / "wheels"
    if wheels_dir.exists():
        for wheel_file in wheels_dir.glob("*.whl"):
            files_to_package.append(wheel_file)

    # Create the zip file
    with ZipFile(output_file, "w", ZIP_DEFLATED) as zipf:
        for file_path in files_to_package:
            # Calculate archive path (relative to root)
            relative_path = file_path.relative_to(root_dir)

            # For Python files in linkforge/, flatten them to root level
            # This prevents double-nesting: extension/linkforge/linkforge/
            if relative_path.parts[0] == "linkforge" and len(relative_path.parts) > 1:
                # Remove the 'linkforge/' prefix
                arcname = Path(*relative_path.parts[1:])
            elif relative_path.parts[0] == "wheels":
                # Keep wheels in wheels/ directory
                arcname = relative_path
            else:
                # Keep manifest, LICENSE, README at root
                arcname = relative_path

            zipf.write(file_path, arcname)
            print(f"  Added: {arcname}")

    print()
    print(f"✅ Extension package created: {output_file}")
    print(f"   Size: {output_file.stat().st_size / 1024:.1f} KB")
    print()
    print("Installation:")
    print("  1. Open Blender 4.2+")
    print("  2. Go to Edit → Preferences → Get Extensions")
    print("  3. Click the dropdown menu (⌄) → Install from Disk")
    print(f"  4. Select: {output_file.absolute()}")
    print()
    print("Or drag and drop the .zip file into Blender!")

    return output_file


def clean_dist() -> None:
    """Remove old build artifacts."""
    dist_dir = Path(__file__).parent / "dist"
    if dist_dir.exists():
        shutil.rmtree(dist_dir)
        print("Cleaned dist/ directory")


if __name__ == "__main__":
    import sys

    # Handle command line arguments
    if len(sys.argv) > 1 and sys.argv[1] == "clean":
        clean_dist()
    else:
        build_extension()
