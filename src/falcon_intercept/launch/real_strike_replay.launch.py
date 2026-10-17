"""Isolated deterministic replay harness; never connects to real PX4 inputs."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter, SetRemap


def generate_launch_description():
    camera_calibration = str(
        Path(get_package_share_directory("falcon_intercept"))
        / "config"
        / "camera"
        / "stereo_calib.yaml"
    )
    upstream_launch = str(
        Path(get_package_share_directory("px4_offboard_tracking"))
        / "launch"
        / "pure_interceptor_manual_real.launch.py"
    )
    replay_group = GroupAction(
        [
            SetParameter(name="use_sim_time", value=True),
            Node(
                package="falcon_intercept",
                executable="stereo_target_solver_node",
                name="stereo_target_solver_node",
                output="screen",
                parameters=[
                    {"calibration_path": camera_calibration, "use_px4_pose_history": False}
                ],
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
            Node(
                package="falcon_intercept",
                executable="perception_bridge_node",
                name="perception_bridge_node",
                output="screen",
                parameters=[{"search_canonical_tilt_deg": 0.0}],
            ),
            Node(
                package="falcon_intercept",
                executable="replay_px4_state_node",
                name="replay_px4_state_node",
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
    return LaunchDescription(
        [SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"), replay_group]
    )
