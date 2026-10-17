"""Field terminal 3: explicitly authorized production visual closed-loop Live."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _authorized_actions(context):
    authorized = LaunchConfiguration("field_authorized").perform(context).strip().lower()
    if authorized not in {"true", "1", "yes"}:
        raise RuntimeError(
            "REAL FLIGHT REFUSED: pass field_authorized:=true after pilot and field approval"
        )
    real_launch = str(
        Path(get_package_share_directory("falcon_intercept"))
        / "launch"
        / "real_strike_v0.launch.py"
    )
    return [
        LogInfo(msg="REAL VISUAL CLOSED LOOP AUTHORIZED: pilot retains Offboard control"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(real_launch),
            launch_arguments={
                "adapter_mode": "live",
                "allow_live_control": "true",
                "calibration_ready": "true",
            }.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("field_authorized", default_value="false"),
            SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
            OpaqueFunction(function=_authorized_actions),
        ]
    )
