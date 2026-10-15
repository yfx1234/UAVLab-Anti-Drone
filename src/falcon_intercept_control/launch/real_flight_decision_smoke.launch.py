from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="falcon_intercept_control",
                executable=name,
                output="screen",
                parameters=(
                    [{"calibration_ready": True}] if name == "safety_node" else
                    [{"search_timeout_s": 4.0}] if name == "search_controller_node" else []
                ),
            )
            for name in (
                "search_controller_node",
                "intercept_fsm_node",
                "guidance_node",
                "safety_node",
            )
        ]
    )
