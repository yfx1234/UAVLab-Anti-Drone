"""Field terminal 1: production three-camera DeepStream perception."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    camera_launch = str(
        Path(get_package_share_directory("falcon_intercept_deepstream"))
        / "launch"
        / "deepstream_three_camera_v0.launch.py"
    )
    return LaunchDescription(
        [
            SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
            SetEnvironmentVariable("DISPLAY", ":0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(camera_launch),
                launch_arguments={
                    "display": "true",
                    "record": "false",
                    "stereo_roi_enabled": "true",
                    # 2026-08-14: lower the detector floor to improve stereo
                    # detection continuity (handover needs ~4 depth frames with
                    # <0.2 s gaps).  The search bearing node keeps its own
                    # 0.5 minimum, and the solver's range-rate gate rejects
                    # mis-paired garbage, so the extra recall is safe.
                    "confidence": "0.35",
                }.items(),
            ),
        ]
    )
