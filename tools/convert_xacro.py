#!/usr/bin/env python3
"""Standalone XACRO to URDF converter.

This utility converts XACRO files to URDF without requiring a full ROS installation.
It uses the xacrodoc library, which is ROS-independent.

Installation:
    pip install xacrodoc

Usage:
    python convert_xacro.py robot.urdf.xacro
    python convert_xacro.py robot.urdf.xacro -o output.urdf
    python convert_xacro.py robot.urdf.xacro --pretty

For more information about xacrodoc:
    https://github.com/adamheins/xacrodoc
"""

import argparse
import sys
from pathlib import Path


def check_xacrodoc_installed() -> bool:
    """Check if xacrodoc is installed."""
    try:
        import xacrodoc  # noqa: F401
        return True
    except ImportError:
        return False


def convert_xacro_to_urdf(
    xacro_path: Path,
    output_path: Path | None = None,
    pretty: bool = True,
) -> None:
    """Convert XACRO file to URDF.

    Args:
        xacro_path: Path to input XACRO file
        output_path: Path to output URDF file (optional, defaults to stem.urdf)
        pretty: If True, format output with indentation

    Raises:
        FileNotFoundError: If XACRO file doesn't exist
        ImportError: If xacrodoc is not installed
        Exception: If conversion fails
    """
    if not xacro_path.exists():
        raise FileNotFoundError(f"XACRO file not found: {xacro_path}")

    if not check_xacrodoc_installed():
        raise ImportError(
            "xacrodoc is not installed. Install it with:\n"
            "  pip install xacrodoc\n\n"
            "For more information: https://github.com/adamheins/xacrodoc"
        )

    try:
        from xacrodoc import XacroDoc
    except ImportError as e:
        raise ImportError(
            f"Failed to import xacrodoc: {e}\n"
            "Try reinstalling with: pip install --upgrade xacrodoc"
        ) from e

    # Determine output path
    if output_path is None:
        output_path = xacro_path.parent / f"{xacro_path.stem}.urdf"

    print(f"Converting {xacro_path.name} to URDF...")

    try:
        # Load and convert XACRO
        doc = XacroDoc.from_file(str(xacro_path))
        urdf_string = doc.to_urdf_string()

        # Optionally pretty-print
        if pretty:
            import xml.etree.ElementTree as ET
            from xml.dom import minidom

            # Parse and reformat
            root = ET.fromstring(urdf_string)
            rough_string = ET.tostring(root, encoding="unicode")
            reparsed = minidom.parseString(rough_string)
            pretty_xml = reparsed.toprettyxml(indent="  ")

            # Clean up extra blank lines
            lines = [line for line in pretty_xml.split("\n") if line.strip()]
            urdf_string = "\n".join(lines) + "\n"

        # Write output
        output_path.write_text(urdf_string, encoding="utf-8")

        print(f"Success! URDF saved to: {output_path}")
        print(f"File size: {output_path.stat().st_size:,} bytes")

    except Exception as e:
        raise RuntimeError(f"Conversion failed: {e}") from e


def main() -> int:
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Convert XACRO files to URDF (ROS-independent)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Convert robot.urdf.xacro to robot.urdf
  python convert_xacro.py robot.urdf.xacro

  # Specify output file
  python convert_xacro.py robot.urdf.xacro -o my_robot.urdf

  # Disable pretty printing (compact output)
  python convert_xacro.py robot.urdf.xacro --no-pretty

Requirements:
  pip install xacrodoc

For more information:
  https://github.com/adamheins/xacrodoc
        """,
    )

    parser.add_argument(
        "input",
        type=Path,
        help="Input XACRO file path",
    )

    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="Output URDF file path (default: input_stem.urdf)",
    )

    parser.add_argument(
        "--pretty",
        dest="pretty",
        action="store_true",
        default=True,
        help="Format output with indentation (default: True)",
    )

    parser.add_argument(
        "--no-pretty",
        dest="pretty",
        action="store_false",
        help="Disable pretty printing (compact output)",
    )

    parser.add_argument(
        "--check",
        action="store_true",
        help="Check if xacrodoc is installed and exit",
    )

    args = parser.parse_args()

    # Handle --check flag
    if args.check:
        if check_xacrodoc_installed():
            print("xacrodoc is installed and ready to use")
            return 0
        else:
            print("xacrodoc is not installed")
            print("Install it with: pip install xacrodoc")
            return 1

    # Convert XACRO to URDF
    try:
        convert_xacro_to_urdf(
            xacro_path=args.input,
            output_path=args.output,
            pretty=args.pretty,
        )
        return 0

    except FileNotFoundError as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1

    except ImportError as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1

    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        import traceback

        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
