"""Production perception + unmodified upstream decision + isolated PX4 shadow chain."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue


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
    adapter_mode = LaunchConfiguration("adapter_mode")
    allow_live_control = LaunchConfiguration("allow_live_control")
    calibration_ready = LaunchConfiguration("calibration_ready")

    upstream_isolated = GroupAction(
        [
            SetRemap(
                src="/fmu/in/offboard_control_mode",
                dst="/upstream/fmu/in/offboard_control_mode",
            ),
            SetRemap(
                src="/fmu/in/trajectory_setpoint",
                dst="/upstream/fmu/in/trajectory_setpoint",
            ),
            SetRemap(
                src="/fmu/in/vehicle_attitude_setpoint",
                dst="/upstream/fmu/in/vehicle_attitude_setpoint",
            ),
            # No node subscribes to this sink. It is deliberately not connected
            # to either the safety adapter or the real PX4 namespace.
            SetRemap(
                src="/fmu/in/vehicle_command",
                dst="/upstream/blocked/vehicle_command",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(upstream_launch),
                launch_arguments={"required_captures": "1"}.items(),
            ),
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("adapter_mode", default_value="shadow"),
            DeclareLaunchArgument("allow_live_control", default_value="false"),
            DeclareLaunchArgument("calibration_ready", default_value="false"),
            SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
            Node(
                package="falcon_intercept",
                executable="stereo_target_solver_node",
                name="stereo_target_solver_node",
                output="screen",
                parameters=[
                    {
                        "calibration_path": camera_calibration,
                        # 2026-08-13 field chain-validation override: the stereo
                        # rig drifted mechanically after the 08-04 calibration
                        # (rectified vertical error median ~75 px vs the 8 px
                        # gate), rejecting 98% of pairs. A drift rotation fitted
                        # from live correspondences (drift_rotation_back_rad in
                        # stereo_calib.yaml) brings the residual to ~9 px
                        # median / ~21 px p90, so the gate sits at 25 px until
                        # the next checkerboard re-calibration restores 8.0.
                        "max_vertical_error_px": 25.0,
                        # 2026-08-14: the 0.30 s fixed-lag smoother plus pipeline
                        # latency put the depth observation age at ~370 ms
                        # median (tail >600 ms), flickering the controller's
                        # freshness gate and oscillating INTERCEPT<->IMM_COAST.
                        # 0.20 s keeps pairing/smoothing depth while bringing
                        # the median age to ~270 ms, inside the 0.45 s gate.
                        "async_fixed_lag_s": 0.20,
                    }
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
                parameters=[
                    {
                        # Physical search boresight is already 10 deg toward
                        # the nose and the upstream controller owns that
                        # extrinsic.  The bridge corrects only the -90 deg
                        # image roll; a second 10 deg pitch cancels the real
                        # mount and creates the observed handover bias.
                        "search_canonical_tilt_deg": 0.0,
                    }
                ],
            ),
            upstream_isolated,
            Node(
                package="falcon_intercept",
                executable="upstream_gate_node",
                name="upstream_gate_node",
                output="screen",
            ),
            Node(
                package="falcon_intercept",
                executable="px4_upstream_adapter_node",
                name="px4_upstream_adapter",
                output="screen",
                parameters=[
                    {
                        "mode": adapter_mode,
                        "allow_live_control": ParameterValue(
                            allow_live_control, value_type=bool
                        ),
                        "calibration_ready": ParameterValue(
                            calibration_ready, value_type=bool
                        ),
                        "rate_hz": 50.0,
                    }
                ],
            ),
        ]
    )
