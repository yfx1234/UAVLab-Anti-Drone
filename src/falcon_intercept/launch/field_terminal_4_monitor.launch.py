"""Field terminal 4: concise transition-oriented read-only monitoring."""

from launch import LaunchDescription
from launch.actions import LogInfo, SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
            LogInfo(
                msg=(
                    "Monitor: state_name, safe_to_exit_offboard and virtual shot events. "
                    "Any anomaly: pilot exits Offboard first."
                )
            ),
            Node(
                package="falcon_intercept",
                executable="field_monitor_node",
                name="field_monitor_node",
                output="screen",
            ),
        ]
    )
