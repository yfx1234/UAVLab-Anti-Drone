#!/usr/bin/env python3
"""整理单次飞行记录并生成双相机、姿态和打击轨迹图。"""

from __future__ import annotations

import argparse
from pathlib import Path

from trajectory_plotting import plot_run


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path, help="trajectory_recorder 生成的 CSV")
    parser.add_argument(
        "--output-directory",
        type=Path,
        default=None,
        help="试飞结果外层目录；默认使用CSV所在目录",
    )
    parser.add_argument("--camera-width", type=int, default=3840)
    parser.add_argument("--camera-height", type=int, default=2160)
    parser.add_argument(
        "--no-organize",
        action="store_true",
        help="只绘图，不把CSV、日志和元数据移入“原始数据”目录",
    )
    parser.add_argument("--show", action="store_true")
    arguments = parser.parse_args()
    output_directory = arguments.output_directory or arguments.csv.parent
    outputs = plot_run(
        arguments.csv,
        output_directory,
        arguments.camera_width,
        arguments.camera_height,
        not arguments.no_organize,
        arguments.show,
    )
    for output in outputs:
        print(output)


if __name__ == "__main__":
    main()
