#!/usr/bin/env python3
"""Fail fast when logical stereo left/right mapping drifts across packages."""

from __future__ import annotations

import argparse
import re
from pathlib import Path

FRONT_SENSOR_ID = 1
BACK_SENSOR_ID = 0
FRONT_PORT = "CAM1"
BACK_PORT = "CAM0"


def require_text(path: Path, pattern: str, description: str) -> None:
    text = path.read_text(encoding="utf-8")
    if re.search(pattern, text, re.MULTILINE) is None:
        raise RuntimeError(f"{description}: {path}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--workspace-src",
        type=Path,
        default=Path(__file__).resolve().parents[2],
        help="ROS workspace src directory",
    )
    args = parser.parse_args()
    source = args.workspace_src.resolve()
    intercept = source / "falcon_intercept"
    deepstream = source / "falcon_intercept_deepstream"

    catalog_path = intercept / "config/camera/camera_catalog.yaml"
    require_text(
        catalog_path,
        rf"stereo_front:\s+physical_port:\s*{FRONT_PORT}\s+argus_sensor_id:\s*{FRONT_SENSOR_ID}",
        "camera catalog left mapping mismatch",
    )
    require_text(
        catalog_path,
        rf"stereo_back:\s+physical_port:\s*{BACK_PORT}\s+argus_sensor_id:\s*{BACK_SENSOR_ID}",
        "camera catalog right mapping mismatch",
    )

    calibration_path = intercept / "config/camera/stereo_calib.yaml"
    require_text(
        calibration_path,
        rf"front_sensor:\s*{FRONT_PORT}\s+back_sensor:\s*{BACK_PORT}",
        "stereo calibration sensor mapping mismatch",
    )
    require_text(
        calibration_path,
        r"horizontal_disparity_sign:\s*1\.0",
        "stereo calibration disparity sign mismatch",
    )

    extrinsics_path = intercept / "config/camera/extrinsics.yaml"
    require_text(
        extrinsics_path,
        rf"{BACK_PORT}:\s*stereo_back_camera\s+{FRONT_PORT}:\s*stereo_front_camera",
        "extrinsics sensor mapping mismatch",
    )

    require_text(
        deepstream / "src/deepstream_three_camera_node.cpp",
        rf"kDefaultFrontSensorId\s*=\s*{FRONT_SENSOR_ID}\s*;",
        "DeepStream left sensor default mismatch",
    )
    require_text(
        deepstream / "src/deepstream_three_camera_node.cpp",
        rf"kDefaultBackSensorId\s*=\s*{BACK_SENSOR_ID}\s*;",
        "DeepStream right sensor default mismatch",
    )
    for script in ("three_camera_preview.sh", "three_camera_preview_power_safe.sh"):
        path = deepstream / "tools" / script
        require_text(
            path,
            rf'FRONT_SENSOR_ID="\$\{{FRONT_SENSOR_ID:-{FRONT_SENSOR_ID}\}}"',
            "recorder front sensor default mismatch",
        )
        require_text(
            path,
            rf'BACK_SENSOR_ID="\$\{{BACK_SENSOR_ID:-{BACK_SENSOR_ID}\}}"',
            "recorder back sensor default mismatch",
        )

        # The Jetson historically exposed a second, operator-facing copy under
        # ~/search_camera_test/tools.  Check it whenever it exists so an old
        # deployment entry cannot silently preserve the opposite mapping.
        deployed_path = Path.home() / "search_camera_test/tools" / script
        if deployed_path.exists():
            require_text(
                deployed_path,
                rf'FRONT_SENSOR_ID="\$\{{FRONT_SENSOR_ID:-{FRONT_SENSOR_ID}\}}"',
                "deployed recorder front sensor default mismatch",
            )
            require_text(
                deployed_path,
                rf'BACK_SENSOR_ID="\$\{{BACK_SENSOR_ID:-{BACK_SENSOR_ID}\}}"',
                "deployed recorder back sensor default mismatch",
            )
            if deployed_path.resolve() != path.resolve() and (
                deployed_path.read_bytes() != path.read_bytes()
            ):
                raise RuntimeError(
                    f"deployed recorder is not canonical or byte-identical: {deployed_path}"
                )

    for launch in (
        intercept / "launch/stereo_cameras_v0.launch.py",
        intercept / "launch/three_cameras_v0.launch.py",
        intercept / "launch/three_yolo_v0.launch.py",
        deepstream / "launch/deepstream_three_camera_v0.launch.py",
    ):
        require_text(
            launch,
            rf'"front_sensor_id", default_value="{FRONT_SENSOR_ID}"',
            "launch front sensor default mismatch",
        )
        require_text(
            launch,
            rf'"back_sensor_id", default_value="{BACK_SENSOR_ID}"',
            "launch back sensor default mismatch",
        )

    print(
        "CAMERA_MAPPING_CONTRACT_OK "
        f"front={FRONT_PORT}/sensor{FRONT_SENSOR_ID} "
        f"back={BACK_PORT}/sensor{BACK_SENSOR_ID} search=USB"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
