"""Test-only FIRE-to-COMPLETE replay, isolated from every production PX4 topic."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    upstream_launch = str(
        Path(get_package_share_directory("px4_offboard_tracking"))
        / "launch"
        / "pure_interceptor_manual_real.launch.py"
    )
    return LaunchDescription(
        [
            SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
            Node(
                package="falcon_intercept",
                executable="replay_px4_state_node",
                name="replay_px4_state_node",
                output="screen",
            ),
            Node(
                package="falcon_intercept",
                executable="synthetic_engagement_node",
                name="synthetic_engagement_node",
                output="screen",
            ),
            GroupAction(
                [
                    SetRemap(
                        src="/test/fmu/in/offboard_control_mode",
                        dst="/upstream/test_fmu/in/offboard_control_mode",
                    ),
                    SetRemap(
                        src="/test/fmu/in/trajectory_setpoint",
                        dst="/upstream/test_fmu/in/trajectory_setpoint",
                    ),
                    SetRemap(
                        src="/test/fmu/in/vehicle_attitude_setpoint",
                        dst="/upstream/test_fmu/in/vehicle_attitude_setpoint",
                    ),
                    SetRemap(
                        src="/test/fmu/in/vehicle_command",
                        dst="/upstream/test_blocked/vehicle_command",
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(upstream_launch),
                        launch_arguments={
                            "pursuer_prefix": "/test/fmu",
                            "pursuer_attitude_topic": "/test/fmu/out/vehicle_attitude",
                            "required_captures": "1",
                        }.items(),
                    ),
                ]
            ),
            Node(
                package="falcon_intercept",
                executable="upstream_gate_node",
                name="upstream_gate_node",
                output="screen",
                parameters=[
                    {
                        "upstream_prefix": "/upstream/test_fmu/in",
                        "adapter_prefix": "/intercept/replay_px4",
                    }
                ],
            ),
            Node(
                package="falcon_intercept",
                executable="px4_upstream_adapter_node",
                name="px4_upstream_adapter",
                output="screen",
                parameters=[
                    {
                        "mode": "shadow",
                        "allow_live_control": False,
                        "calibration_ready": False,
                        "px4_namespace": "/test/fmu",
                        "upstream_prefix": "/intercept/replay_px4",
                        "rate_hz": 50.0,
                    }
                ],
            ),
        ]
    )
