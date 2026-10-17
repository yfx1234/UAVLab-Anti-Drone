from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = str(
        Path(get_package_share_directory("falcon_intercept_control"))
        / "config"
        / "control_v0.yaml"
    )
    mode = LaunchConfiguration("adapter_mode")
    return LaunchDescription(
        [
            DeclareLaunchArgument("adapter_mode", default_value="shadow"),
            *[
                Node(
                    package="falcon_intercept_control",
                    executable=executable,
                    name=executable,
                    output="screen",
                    parameters=[config],
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
                parameters=[config, {"mode": mode}],
            ),
        ]
    )
