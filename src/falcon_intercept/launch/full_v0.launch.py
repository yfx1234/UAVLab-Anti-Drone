from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    control_config = str(
        Path(get_package_share_directory("falcon_intercept_control"))
        / "config"
        / "control_v0.yaml"
    )
    camera_calibration = str(
        Path(get_package_share_directory("falcon_intercept"))
        / "config"
        / "camera"
        / "stereo_calib.yaml"
    )
    adapter_mode = LaunchConfiguration("adapter_mode")
    return LaunchDescription(
        [
            DeclareLaunchArgument("adapter_mode", default_value="shadow"),
            Node(
                package="falcon_intercept",
                executable="stereo_target_solver_node",
                name="stereo_target_solver_node",
                output="screen",
                parameters=[{"calibration_path": camera_calibration}],
            ),
            Node(
                package="falcon_intercept",
                executable="target_kf_node",
                name="target_kf_node",
                output="screen",
                parameters=[{"use_async_primary": True}],
            ),
            Node(
                package="falcon_intercept",
                executable="search_bearing_node",
                name="search_bearing_node",
                output="screen",
            ),
            *[
                Node(
                    package="falcon_intercept_control",
                    executable=executable,
                    name=executable,
                    output="screen",
                    parameters=[control_config],
                )
                for executable in (
                    "search_controller_node",
                    "intercept_fsm_node",
                    "guidance_node",
                    "safety_node",
                )
            ],
            Node(
                package="falcon_intercept",
                executable="px4_adapter_node",
                name="px4_velocity_adapter",
                output="screen",
                parameters=[
                    control_config,
                    {
                        "mode": adapter_mode,
                        "allow_live_control": False,
                        "calibration_ready": False,
                    },
                ],
            ),
        ]
    )
