#!/usr/bin/env python3
"""Generate 3D aircraft and camera-image trajectory plots from a recording."""

from __future__ import annotations

import argparse
from pathlib import Path

from trajectory_plotting import plot_record


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path, help="trajectory_recorder CSV file")
    parser.add_argument("--output-directory", type=Path, default=None)
    parser.add_argument("--camera-width", type=int, default=3840)
    parser.add_argument("--camera-height", type=int, default=2160)
    parser.add_argument("--show", action="store_true")
    arguments = parser.parse_args()
    output_directory = arguments.output_directory or arguments.csv.parent
    outputs = plot_record(
        arguments.csv,
        output_directory,
        arguments.camera_width,
        arguments.camera_height,
        arguments.show,
    )
    for output in outputs:
        print(output)


if __name__ == "__main__":
    main()
