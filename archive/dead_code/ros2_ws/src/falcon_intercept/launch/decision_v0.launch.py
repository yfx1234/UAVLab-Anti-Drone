"""Decision-only shadow entry using the production typed FLU chain.

This launch intentionally has no fake vehicle state and never enables live PX4 output.
Use falcon_intercept_control/test/run_real_flight_decision_smoke.sh for synthetic tests.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = str(
        Path(get_package_share_directory("falcon_intercept_control"))
        / "config"
        / "control_v0.yaml"
    )
    nodes = [
        Node(
            package="falcon_intercept_control",
            executable=name,
            name=name,
            output="screen",
            parameters=[config],
        )
        for name in (
            "search_controller_node",
            "intercept_fsm_node",
            "guidance_node",
            "safety_node",
        )
    ]
    nodes.append(
        Node(
            package="falcon_intercept",
            executable="px4_adapter_node",
            name="px4_velocity_adapter",
            output="screen",
            parameters=[config, {"mode": "shadow"}],
        )
    )
    return LaunchDescription(nodes)
