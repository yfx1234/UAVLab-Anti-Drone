#!/usr/bin/env python3
"""为P7闭环验收目录生成五张中文可视化图片。"""

from __future__ import annotations

import argparse
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from p7_acceptance_plotting import plot_p7_run


def default_config_path() -> Path:
    return (
        Path(get_package_share_directory("px4_offboard_tracking"))
        / "config"
        / "high_speed_profiles.yaml"
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="读取data/status.csv、px4_0.ulg和px4_1.ulg生成P7验收图片"
    )
    parser.add_argument("run_directory", type=Path, help="单次P7验收结果目录")
    parser.add_argument(
        "--config",
        type=Path,
        default=default_config_path(),
        help="速度档位配置文件",
    )
    arguments = parser.parse_args()
    for output in plot_p7_run(
        arguments.run_directory.expanduser().resolve(),
        arguments.config.expanduser().resolve(),
    ):
        print(output)


if __name__ == "__main__":
    main()
