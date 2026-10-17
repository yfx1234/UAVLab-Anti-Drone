from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory("px4_interceptor_bridge")) / "config" / "interceptor_bridge.yaml"
    return LaunchDescription(
        [
            Node(
                package="px4_interceptor_bridge",
                executable="interceptor_bridge_node",
                name="px4_interceptor_bridge",
                output="screen",
                parameters=[str(config)],
            )
        ]
    )
