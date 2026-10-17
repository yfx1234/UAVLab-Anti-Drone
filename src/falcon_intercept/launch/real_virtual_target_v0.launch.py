"""Real aircraft chasing a coordinate-only virtual target.

This is deliberately separate from full_v0.launch.py and every RTK field-test
entry.  It never starts physical cameras or DeepStream.  The virtual target
enters the production perception boundary as asynchronous detection JSON; its
truth topic is reserved for the independent referee.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    intercept_share = Path(get_package_share_directory("falcon_intercept"))
    control_share = Path(get_package_share_directory("falcon_intercept_control"))
    calibration = str(intercept_share / "config" / "camera" / "stereo_calib.yaml")
    control_config = str(control_share / "config" / "control_v0.yaml")

    adapter_mode = LaunchConfiguration("adapter_mode")
    allow_live = LaunchConfiguration("allow_live_control")
    calibration_ready = LaunchConfiguration("calibration_ready")
    scenario = LaunchConfiguration("target_scenario")
    speed = LaunchConfiguration("target_speed_mps")
    radius = LaunchConfiguration("target_radius_m")
    path_width = LaunchConfiguration("target_path_width_m")
    path_length = LaunchConfiguration("target_path_length_m")
    initial_forward = LaunchConfiguration("initial_forward_m")
    initial_left = LaunchConfiguration("initial_left_m")
    initial_up = LaunchConfiguration("initial_up_m")
    motion_delay = LaunchConfiguration("target_motion_delay_s")
    motion_ramp = LaunchConfiguration("target_motion_ramp_s")
    detection_mode = LaunchConfiguration("detection_mode")
    detection_start_delay = LaunchConfiguration("detection_start_delay_s")
    dropout_start = LaunchConfiguration("detection_dropout_start_s")
    dropout_duration = LaunchConfiguration("detection_dropout_duration_s")
    attack_delay = LaunchConfiguration("attack_enable_delay_s")
    required_shots = LaunchConfiguration("required_shots")
    recoil_duration = LaunchConfiguration("recoil_duration_s")
    recoil_speed = LaunchConfiguration("recoil_opening_speed_mps")
    reattack_cooldown = LaunchConfiguration("reattack_cooldown_s")

    control_overrides = {
        "calibration_ready": calibration_ready,
        "capture_minimum_range_m": 15.0,
        "capture_maximum_range_m": 35.0,
        "shot_hard_alignment_rad": 0.034906585,
        "shot_preferred_alignment_rad": 0.017453293,
        "shot_timing_prediction_horizon_s": 0.04,
        "shot_error_filter_time_constant_s": 0.08,
        "shot_maximum_error_rate_rad_s": 0.5,
        "shot_minimum_ready_hold_s": 0.04,
        "capture_minimum_distinct_frames": 2,
        "capture_maximum_los_rate_rad_s": 0.25,
        "capture_maximum_relative_speed_mps": 10.0,
        "capture_maximum_predicted_miss_distance_m": 35.0,
        "capture_maximum_time_to_closest_approach_s": 15.0,
        "projectile_speed_mps": 400.0,
        "shot_maximum_time_of_flight_s": 0.25,
        "shot_maximum_lead_angle_rad": 0.069813170,
        "shot_maximum_lateral_sigma_m": 0.75,
        "maximum_forward_speed_mps": 3.0,
        "maximum_lateral_speed_mps": 1.5,
        "maximum_vertical_speed_mps": 1.0,
        "maximum_yaw_rate_rps": 0.5,
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("adapter_mode", default_value="shadow"),
            DeclareLaunchArgument("allow_live_control", default_value="false"),
            DeclareLaunchArgument("calibration_ready", default_value="true"),
            DeclareLaunchArgument("target_scenario", default_value="straight"),
            DeclareLaunchArgument("target_speed_mps", default_value="2.0"),
            DeclareLaunchArgument("target_radius_m", default_value="20.0"),
            DeclareLaunchArgument("target_path_width_m", default_value="8.0"),
            DeclareLaunchArgument("target_path_length_m", default_value="40.0"),
            DeclareLaunchArgument("initial_forward_m", default_value="25.0"),
            DeclareLaunchArgument("initial_left_m", default_value="0.0"),
            DeclareLaunchArgument("initial_up_m", default_value="0.0"),
            DeclareLaunchArgument("target_motion_delay_s", default_value="2.0"),
            DeclareLaunchArgument("target_motion_ramp_s", default_value="2.0"),
            DeclareLaunchArgument("detection_mode", default_value="all"),
            DeclareLaunchArgument("detection_start_delay_s", default_value="0.0"),
            DeclareLaunchArgument("detection_dropout_start_s", default_value="-1.0"),
            DeclareLaunchArgument("detection_dropout_duration_s", default_value="0.0"),
            DeclareLaunchArgument("attack_enable_delay_s", default_value="30.0"),
            DeclareLaunchArgument("required_shots", default_value="2"),
            DeclareLaunchArgument("recoil_duration_s", default_value="0.2"),
            DeclareLaunchArgument("recoil_opening_speed_mps", default_value="1.0"),
            DeclareLaunchArgument("reattack_cooldown_s", default_value="3.0"),
            Node(
                package="falcon_intercept",
                executable="real_aircraft_virtual_target",
                name="real_aircraft_virtual_target",
                output="screen",
                parameters=[
                    {
                        "calibration_path": calibration,
                        "target_scenario": scenario,
                        "target_speed_mps": speed,
                        "target_radius_m": radius,
                        "target_path_width_m": path_width,
                        "target_path_length_m": path_length,
                        "initial_forward_m": initial_forward,
                        "initial_left_m": initial_left,
                        "initial_up_m": initial_up,
                        "target_motion_delay_s": motion_delay,
                        "target_motion_ramp_s": motion_ramp,
                        "detection_mode": detection_mode,
                        "detection_start_delay_s": detection_start_delay,
                        "detection_dropout_start_s": dropout_start,
                        "detection_dropout_duration_s": dropout_duration,
                        "right_phase_offset_s": 0.012,
                        "require_armed": True,
                        "require_offboard": True,
                        "latch_on_offboard_exit": True,
                    }
                ],
            ),
            Node(
                package="falcon_intercept",
                executable="virtual_shot_referee",
                name="virtual_shot_referee",
                output="screen",
                parameters=[
                    {
                        "projectile_speed_mps": 400.0,
                        "maximum_time_s": 0.25,
                        "hit_radius_m": 1.0,
                    }
                ],
            ),
            Node(
                package="falcon_intercept",
                executable="stereo_target_solver_node",
                name="stereo_target_solver_node",
                output="screen",
                parameters=[{"calibration_path": calibration}],
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
                package="falcon_intercept_control",
                executable="search_controller_node",
                name="search_controller_node",
                output="screen",
                parameters=[control_config],
            ),
            Node(
                package="falcon_intercept_control",
                executable="intercept_fsm_node",
                name="intercept_fsm_node",
                output="screen",
                parameters=[
                    control_config,
                    {
                        "attack_enable_delay_s": attack_delay,
                        "required_shots": required_shots,
                        "recoil_duration_s": recoil_duration,
                        "recoil_opening_speed_mps": recoil_speed,
                        "reattack_cooldown_s": reattack_cooldown,
                    },
                ],
            ),
            Node(
                package="falcon_intercept_control",
                executable="guidance_node",
                name="guidance_node",
                output="screen",
                parameters=[
                    control_config,
                    {
                        "position_gain_forward": 0.40,
                        "velocity_feedforward": 1.0,
                        "desired_distance_m": 25.0,
                        "maximum_forward_speed_mps": 3.0,
                        "maximum_lateral_speed_mps": 1.5,
                        "maximum_vertical_speed_mps": 1.0,
                        "maximum_yaw_rate_rps": 0.5,
                        "maximum_acceleration_mps2": 1.5,
                    },
                ],
            ),
            Node(
                package="falcon_intercept_control",
                executable="safety_node",
                name="safety_node",
                output="screen",
                parameters=[control_config, control_overrides],
            ),
            Node(
                package="falcon_intercept",
                executable="px4_adapter_node",
                name="px4_velocity_adapter",
                output="screen",
                parameters=[
                    control_config,
                    {
                        "mode": adapter_mode,
                        "allow_live_control": allow_live,
                        "calibration_ready": calibration_ready,
                    },
                ],
            ),
        ]
    )
