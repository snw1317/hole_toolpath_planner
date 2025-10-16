#!/usr/bin/env python3
"""Scale an STL mesh authored in inches into metres."""
from __future__ import annotations

import argparse
import pathlib
import sys

try:
    from stl import mesh  # type: ignore
except ImportError as exc:  # pragma: no cover
    sys.exit(
        "Missing dependency 'numpy-stl'. Install it with:\n"
        "    python3 -m pip install --user numpy-stl\n"
    )

INCH_TO_METRE = 0.0254  # constant scale factor for inches -> metres


def scale_mesh(input_path: pathlib.Path, output_path: pathlib.Path, scale: float) -> None:
    """Load an STL, apply uniform scaling, and write the result."""
    part = mesh.Mesh.from_file(str(input_path))
    part.points *= scale
    part.update_normals()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    part.save(str(output_path))
    print(f"Wrote scaled mesh -> {output_path}")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Scale an STL mesh by a constant factor.")
    parser.add_argument("--input", "-i", type=pathlib.Path, required=True, help="Source STL file")
    parser.add_argument("--output", "-o", type=pathlib.Path, required=True, help="Destination STL file")
    parser.add_argument(
        "--scale",
        "-s",
        type=float,
        default=INCH_TO_METRE,
        help=f"Scale factor to apply (default {INCH_TO_METRE} for inches to metres)",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    args = parse_args(sys.argv[1:] if argv is None else argv)

    if not args.input.exists():
        sys.exit(f"Input file not found: {args.input}")

    scale_mesh(args.input, args.output, args.scale)


if __name__ == "__main__":
    main()
