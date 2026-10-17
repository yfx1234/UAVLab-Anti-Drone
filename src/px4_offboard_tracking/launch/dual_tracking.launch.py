from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    target_state_topic = LaunchConfiguration("target_state_topic")
    common_origin_topic = LaunchConfiguration("common_origin_topic")
    relative_state_topic = LaunchConfiguration("relative_state_topic")
    relative_estimate_topic = LaunchConfiguration("relative_estimate_topic")
    relative_kinematics_topic = LaunchConfiguration("relative_kinematics_topic")
    common_frame_id = LaunchConfiguration("common_frame_id")
    completion_monitor = Node(
        package="px4_offboard_tracking",
        executable="mission_completion_monitor",
        namespace="mission",
        name="completion_monitor",
        output="screen",
        condition=IfCondition(LaunchConfiguration("shutdown_on_mission_complete")),
        parameters=[{
            "shutdown_delay_s": LaunchConfiguration("mission_shutdown_delay_s"),
        }],
    )
    return LaunchDescription([
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
        DeclareLaunchArgument("control_level", default_value="attitude"),
        DeclareLaunchArgument("guidance_controller", default_value="mpc_3d"),
        DeclareLaunchArgument("mpc_time_step_s", default_value="0.1"),
        DeclareLaunchArgument("mpc_horizon_steps", default_value="25"),
        DeclareLaunchArgument("mpc_position_weight", default_value="1.0"),
        DeclareLaunchArgument("mpc_velocity_weight", default_value="4.0"),
        DeclareLaunchArgument("mpc_acceleration_weight", default_value="1.5"),
        DeclareLaunchArgument("mpc_terminal_weight_multiplier", default_value="4.0"),
        DeclareLaunchArgument("mpc_prediction_lookahead_s", default_value="0.5"),
        DeclareLaunchArgument(
            "mpc_maximum_horizontal_acceleration_mps2", default_value="1.3"
        ),
        DeclareLaunchArgument("mpc_desired_vertical_separation_m", default_value="30.0"),
        DeclareLaunchArgument(
            "mpc_maximum_vertical_acceleration_mps2", default_value="0.5"
        ),
        DeclareLaunchArgument("mpc_vertical_position_weight", default_value="0.25"),
        DeclareLaunchArgument("mpc_vertical_velocity_weight", default_value="6.0"),
        DeclareLaunchArgument("mpc_vertical_acceleration_weight", default_value="4.0"),
        DeclareLaunchArgument("mpc_vertical_absolute_hold_blend", default_value="0.7"),
        DeclareLaunchArgument("mpc_3d_entry_vertical_tolerance_m", default_value="1.0"),
        DeclareLaunchArgument(
            "mpc_3d_entry_vertical_speed_tolerance_mps", default_value="0.5"
        ),
        DeclareLaunchArgument("mpc_3d_entry_hold_time_s", default_value="2.0"),
        DeclareLaunchArgument("target_prefix", default_value="/fmu"),
        DeclareLaunchArgument("pursuer_prefix", default_value="/px4_1/fmu"),
        DeclareLaunchArgument("target_state_topic", default_value="/tracking/target/state"),
        DeclareLaunchArgument("common_origin_topic", default_value="/tracking/common_origin"),
        DeclareLaunchArgument("relative_state_topic", default_value="/tracking/relative/state"),
        DeclareLaunchArgument(
            "relative_estimate_topic", default_value="/tracking/relative/estimate"
        ),
        DeclareLaunchArgument(
            "relative_kinematics_topic",
            default_value="/tracking/relative/kinematics",
        ),
        DeclareLaunchArgument("common_frame_id", default_value="map_ned"),
        DeclareLaunchArgument("target_scenario", default_value="s_curve"),
        DeclareLaunchArgument("target_height_m", default_value="35.0"),
        DeclareLaunchArgument("pursuer_height_m", default_value="5.0"),
        DeclareLaunchArgument("target_radius_m", default_value="4.0"),
        DeclareLaunchArgument("target_speed_mps", default_value="1.2"),
        DeclareLaunchArgument("target_heading_rad", default_value="0.0"),
        DeclareLaunchArgument("target_path_length_m", default_value="8.0"),
        DeclareLaunchArgument("target_path_width_m", default_value="3.0"),
        DeclareLaunchArgument(
            "target_trajectory_feedforward_enabled", default_value="false"
        ),
        DeclareLaunchArgument("target_motion_delay_s", default_value="20.0"),
        DeclareLaunchArgument("target_motion_ramp_s", default_value="3.0"),
        DeclareLaunchArgument("attack_enable_delay_s", default_value="30.0"),
        DeclareLaunchArgument("target_stable_motion_time_s", default_value="10.0"),
        DeclareLaunchArgument("takeoff_ready_tolerance_m", default_value="0.6"),
        DeclareLaunchArgument("offboard_rate_hz", default_value="50.0"),
        DeclareLaunchArgument("offboard_warmup_s", default_value="1.0"),
        DeclareLaunchArgument("preflight_stability_hold_s", default_value="5.0"),
        DeclareLaunchArgument("pursuer_start_delay_s", default_value="0.0"),
        DeclareLaunchArgument("startup_search_enabled", default_value="false"),
        DeclareLaunchArgument("startup_search_yaw_rate_rad_s", default_value="0.5"),
        DeclareLaunchArgument("startup_search_turns", default_value="1.0"),
        DeclareLaunchArgument("startup_search_settle_time_s", default_value="0.5"),
        DeclareLaunchArgument("startup_search_stop_on_acquisition", default_value="true"),
        DeclareLaunchArgument("startup_search_repeat_until_acquired", default_value="true"),
        DeclareLaunchArgument(
            "startup_search_acquisition_hold_time_s", default_value="0.2"
        ),
        DeclareLaunchArgument(
            "search_handover_alignment_blend", default_value="0.7"
        ),
        DeclareLaunchArgument(
            "search_handover_maximum_tilt_rad", default_value="0.35"
        ),
        DeclareLaunchArgument("depth_confirmation_hold_time_s", default_value="0.2"),
        DeclareLaunchArgument("depth_confirmation_minimum_frames", default_value="5"),
        DeclareLaunchArgument("pursuer_hover_thrust", default_value="0.725"),
        DeclareLaunchArgument("pursuer_max_thrust", default_value="0.9"),
        DeclareLaunchArgument("pursuer_max_tilt_rad", default_value="0.5"),
        DeclareLaunchArgument("alignment_maximum_tilt_rad", default_value="0.40"),
        DeclareLaunchArgument("pursuer_altitude_position_gain", default_value="0.8"),
        DeclareLaunchArgument("pursuer_altitude_velocity_gain", default_value="1.2"),
        DeclareLaunchArgument("pursuer_altitude_max_speed_mps", default_value="2.0"),
        DeclareLaunchArgument("pursuer_altitude_max_acceleration_mps2", default_value="2.0"),
        DeclareLaunchArgument("invalid_estimate_velocity_damping", default_value="1.4"),
        DeclareLaunchArgument(
            "invalid_estimate_maximum_acceleration_mps2", default_value="1.2"
        ),
        DeclareLaunchArgument(
            "command_maximum_horizontal_speed_mps", default_value="4.0"
        ),
        DeclareLaunchArgument("command_speed_soft_zone_mps", default_value="0.5"),
        DeclareLaunchArgument("command_overspeed_braking_gain", default_value="2.0"),
        DeclareLaunchArgument(
            "command_maximum_braking_acceleration_mps2", default_value="2.0"
        ),
        DeclareLaunchArgument(
            "attitude_smoothing_time_constant_s", default_value="0.35"
        ),
        DeclareLaunchArgument(
            "maximum_attitude_slew_rate_rad_s", default_value="1.0"
        ),
        DeclareLaunchArgument("alignment_prediction_time_s", default_value="0.0"),
        DeclareLaunchArgument(
            "alignment_entry_horizontal_distance_m", default_value="1.5"
        ),
        DeclareLaunchArgument("aim_entry_horizontal_distance_m", default_value="10.0"),
        DeclareLaunchArgument("alignment_ready_error_rad", default_value="0.12"),
        DeclareLaunchArgument("aim_exit_horizontal_distance_m", default_value="12.0"),
        DeclareLaunchArgument("capture_minimum_range_m", default_value="10.0"),
        DeclareLaunchArgument("capture_maximum_range_m", default_value="35.0"),
        DeclareLaunchArgument("midcourse_alignment_blend", default_value="0.3"),
        DeclareLaunchArgument("terminal_navigation_constant", default_value="3.0"),
        DeclareLaunchArgument(
            "terminal_minimum_closing_speed_mps", default_value="0.5"
        ),
        DeclareLaunchArgument("terminal_los_acceleration_mps2", default_value="0.6"),
        DeclareLaunchArgument(
            "terminal_acceleration_feedforward_gain", default_value="0.5"
        ),
        DeclareLaunchArgument(
            "terminal_maximum_horizontal_acceleration_mps2", default_value="2.0"
        ),
        DeclareLaunchArgument(
            "terminal_maximum_vertical_acceleration_mps2", default_value="0.8"
        ),
        DeclareLaunchArgument("terminal_alignment_blend", default_value="0.7"),
        DeclareLaunchArgument(
            "terminal_capture_alignment_blend", default_value="0.95"
        ),
        DeclareLaunchArgument(
            "terminal_alignment_weight_slew_rate_per_s", default_value="0.8"
        ),
        DeclareLaunchArgument("estimate_measurement_hold_s", default_value="0.4"),
        DeclareLaunchArgument("collision_exit_range_m", default_value="5.0"),
        DeclareLaunchArgument("enable_camera_observation", default_value="true"),
        DeclareLaunchArgument("enable_relative_estimator", default_value="true"),
        DeclareLaunchArgument("enable_relative_kinematics", default_value="true"),
        DeclareLaunchArgument("enable_trajectory_recorder", default_value="false"),
        DeclareLaunchArgument(
            "trajectory_output_directory",
            default_value="",
        ),
        DeclareLaunchArgument("trajectory_filename", default_value=""),
        DeclareLaunchArgument("trajectory_record_rate_hz", default_value="50.0"),
        DeclareLaunchArgument("camera_rate_hz", default_value="30.0"),
        DeclareLaunchArgument(
            "pursuer_attitude_topic",
            default_value="/px4_1/fmu/out/vehicle_attitude",
        ),
        DeclareLaunchArgument(
            "camera_observation_topic",
            default_value="/tracking/camera/observation",
        ),
        DeclareLaunchArgument("camera_frame_id", default_value="pursuer/camera_optical"),
        DeclareLaunchArgument("camera_width", default_value="3840"),
        DeclareLaunchArgument("camera_height", default_value="2160"),
        DeclareLaunchArgument("camera_fx", default_value="12847.019977569425"),
        DeclareLaunchArgument("camera_fy", default_value="6124.984365187126"),
        DeclareLaunchArgument("camera_cx", default_value="1919.5"),
        DeclareLaunchArgument("camera_cy", default_value="1079.5"),
        DeclareLaunchArgument("search_camera_fx", default_value="6089.462020537369"),
        DeclareLaunchArgument("search_camera_fy", default_value="1870.614872174388"),
        DeclareLaunchArgument("search_camera_cx", default_value="1919.5"),
        DeclareLaunchArgument("search_camera_cy", default_value="1079.5"),
        DeclareLaunchArgument("camera_minimum_depth_m", default_value="0.05"),
        DeclareLaunchArgument("camera_translation_body_x_m", default_value="0.0"),
        DeclareLaunchArgument("camera_translation_body_y_m", default_value="0.0"),
        DeclareLaunchArgument("camera_translation_body_z_m", default_value="0.0"),
        DeclareLaunchArgument("camera_q_body_to_optical_w", default_value="0.0"),
        DeclareLaunchArgument(
            "camera_q_body_to_optical_x",
            default_value="0.7071067811865476",
        ),
        DeclareLaunchArgument(
            "camera_q_body_to_optical_y",
            default_value="0.7071067811865476",
        ),
        DeclareLaunchArgument("camera_q_body_to_optical_z", default_value="0.0"),
        DeclareLaunchArgument(
            "depth_camera_horizontal_fov_rad", default_value="0.296705973"
        ),
        DeclareLaunchArgument(
            "depth_camera_vertical_fov_rad", default_value="0.349065850"
        ),
        DeclareLaunchArgument(
            "search_camera_horizontal_fov_rad", default_value="0.610865238"
        ),
        DeclareLaunchArgument(
            "search_camera_vertical_fov_rad", default_value="1.047197551"
        ),
        DeclareLaunchArgument(
            "search_camera_q_body_to_optical_w",
            default_value="-0.06162841671621935",
        ),
        DeclareLaunchArgument(
            "search_camera_q_body_to_optical_x",
            default_value="0.7044160264027587",
        ),
        DeclareLaunchArgument(
            "search_camera_q_body_to_optical_y",
            default_value="0.7044160264027587",
        ),
        DeclareLaunchArgument(
            "search_camera_q_body_to_optical_z",
            default_value="0.06162841671621935",
        ),
        DeclareLaunchArgument("enable_pixel_noise", default_value="false"),
        DeclareLaunchArgument("pixel_noise_stddev_px", default_value="1.0"),
        DeclareLaunchArgument("enable_depth_noise", default_value="false"),
        DeclareLaunchArgument("depth_noise_stddev_m", default_value="0.05"),
        DeclareLaunchArgument("enable_latency", default_value="false"),
        DeclareLaunchArgument("latency_ms", default_value="0.0"),
        DeclareLaunchArgument("enable_dropout", default_value="false"),
        DeclareLaunchArgument("dropout_probability", default_value="0.0"),
        DeclareLaunchArgument("enable_outliers", default_value="false"),
        DeclareLaunchArgument("outlier_probability", default_value="0.0"),
        DeclareLaunchArgument("outlier_min_depth_m", default_value="1.0"),
        DeclareLaunchArgument("outlier_max_depth_m", default_value="50.0"),
        DeclareLaunchArgument("observation_random_seed", default_value="1"),
        DeclareLaunchArgument("estimator_rate_hz", default_value="50.0"),
        DeclareLaunchArgument("estimator_max_data_age_s", default_value="0.5"),
        DeclareLaunchArgument("estimator_max_position_variance", default_value="25.0"),
        DeclareLaunchArgument("attitude_sync_tolerance_s", default_value="0.05"),
        DeclareLaunchArgument("attitude_buffer_s", default_value="2.0"),
        DeclareLaunchArgument("measurement_pixel_stddev_px", default_value="1.5"),
        DeclareLaunchArgument("measurement_depth_stddev_m", default_value="0.10"),
        DeclareLaunchArgument("acceleration_noise_stddev_mps2", default_value="2.0"),
        DeclareLaunchArgument("initial_position_stddev_m", default_value="1.0"),
        DeclareLaunchArgument("initial_velocity_stddev_mps", default_value="3.0"),
        DeclareLaunchArgument("innovation_gate_mahalanobis", default_value="11.345"),
        DeclareLaunchArgument("safety_minimum_range_m", default_value="5.0"),
        DeclareLaunchArgument("safety_maximum_tilt_rad", default_value="0.45"),
        DeclareLaunchArgument("safety_maximum_speed_mps", default_value="4.0"),
        DeclareLaunchArgument("safety_maximum_altitude_error_m", default_value="20.0"),
        DeclareLaunchArgument("recovery_maximum_tilt_rad", default_value="0.12"),
        DeclareLaunchArgument("recovery_maximum_speed_mps", default_value="0.5"),
        DeclareLaunchArgument("recovery_maximum_altitude_error_m", default_value="0.6"),
        DeclareLaunchArgument("recovery_required_hold_time_s", default_value="1.0"),
        DeclareLaunchArgument("required_virtual_captures", default_value="2"),
        DeclareLaunchArgument("reattack_cooldown_s", default_value="3.0"),
        DeclareLaunchArgument("post_capture_recoil_duration_s", default_value="0.6"),
        DeclareLaunchArgument(
            "post_capture_recoil_acceleration_mps2", default_value="1.0"
        ),
        DeclareLaunchArgument("shutdown_on_mission_complete", default_value="false"),
        DeclareLaunchArgument("mission_shutdown_delay_s", default_value="2.0"),
        DeclareLaunchArgument("pursuer_yaw_mode", default_value="velocity"),
        DeclareLaunchArgument("pursuer_fixed_yaw_rad", default_value="0.0"),
        DeclareLaunchArgument("pursuer_yaw_minimum_speed_mps", default_value="0.3"),
        # Depth fx=12847.02 px: fx*tan(2 deg)=448.628 px.
        DeclareLaunchArgument("capture_maximum_pixel_error_px", default_value="448.628"),
        DeclareLaunchArgument("capture_maximum_alignment_error_rad", default_value="0.034906585"),
        DeclareLaunchArgument("capture_maximum_los_rate_rad_s", default_value="0.25"),
        DeclareLaunchArgument("capture_required_hold_time_s", default_value="0.2"),
        DeclareLaunchArgument("capture_minimum_distinct_frames", default_value="5"),
        Node(
            package="px4_offboard_tracking",
            executable="offboard_vehicle",
            namespace="target",
            name="offboard_vehicle",
            output="screen",
            parameters=[{
                "role": "target",
                "topic_prefix": LaunchConfiguration("target_prefix"),
                "target_system": 1,
                "target_state_topic": target_state_topic,
                "common_origin_topic": common_origin_topic,
                "common_frame_id": common_frame_id,
                "target_scenario": LaunchConfiguration("target_scenario"),
                "takeoff_height_m": LaunchConfiguration("target_height_m"),
                "target_radius_m": LaunchConfiguration("target_radius_m"),
                "target_speed_mps": LaunchConfiguration("target_speed_mps"),
                "target_heading_rad": LaunchConfiguration("target_heading_rad"),
                "target_path_length_m": LaunchConfiguration("target_path_length_m"),
                "target_path_width_m": LaunchConfiguration("target_path_width_m"),
                "target_trajectory_feedforward_enabled": LaunchConfiguration(
                    "target_trajectory_feedforward_enabled"
                ),
                "target_motion_delay_s": LaunchConfiguration("target_motion_delay_s"),
                "target_motion_ramp_s": LaunchConfiguration("target_motion_ramp_s"),
                "offboard_rate_hz": LaunchConfiguration("offboard_rate_hz"),
                "offboard_warmup_s": LaunchConfiguration("offboard_warmup_s"),
                "preflight_stability_hold_s": LaunchConfiguration(
                    "preflight_stability_hold_s"
                ),
                "attack_enable_delay_s": LaunchConfiguration("attack_enable_delay_s"),
            }],
        ),
        TimerAction(
            period=LaunchConfiguration("pursuer_start_delay_s"),
            actions=[Node(
            package="px4_offboard_tracking",
            executable="offboard_vehicle",
            namespace="pursuer",
            name="offboard_vehicle",
            output="screen",
            parameters=[{
                "role": "pursuer",
                "control_level": LaunchConfiguration("control_level"),
                "guidance_controller": LaunchConfiguration("guidance_controller"),
                "topic_prefix": LaunchConfiguration("pursuer_prefix"),
                "target_system": 2,
                "target_state_topic": target_state_topic,
                "common_origin_topic": common_origin_topic,
                "relative_state_topic": relative_state_topic,
                "relative_estimate_topic": relative_estimate_topic,
                "relative_kinematics_topic": relative_kinematics_topic,
                "observation_topic": LaunchConfiguration("camera_observation_topic"),
                "common_frame_id": common_frame_id,
                "takeoff_height_m": LaunchConfiguration("pursuer_height_m"),
                "offboard_rate_hz": LaunchConfiguration("offboard_rate_hz"),
                "offboard_warmup_s": LaunchConfiguration("offboard_warmup_s"),
                "preflight_stability_hold_s": LaunchConfiguration(
                    "preflight_stability_hold_s"
                ),
                "startup_search_enabled": LaunchConfiguration("startup_search_enabled"),
                "startup_search_yaw_rate_rad_s": LaunchConfiguration(
                    "startup_search_yaw_rate_rad_s"
                ),
                "startup_search_turns": LaunchConfiguration("startup_search_turns"),
                "startup_search_settle_time_s": LaunchConfiguration(
                    "startup_search_settle_time_s"
                ),
                "startup_search_stop_on_acquisition": LaunchConfiguration(
                    "startup_search_stop_on_acquisition"
                ),
                "startup_search_repeat_until_acquired": LaunchConfiguration(
                    "startup_search_repeat_until_acquired"
                ),
                "startup_search_acquisition_hold_time_s": LaunchConfiguration(
                    "startup_search_acquisition_hold_time_s"
                ),
                "search_handover_alignment_blend": LaunchConfiguration(
                    "search_handover_alignment_blend"
                ),
                "search_handover_maximum_tilt_rad": LaunchConfiguration(
                    "search_handover_maximum_tilt_rad"
                ),
                "depth_confirmation_hold_time_s": LaunchConfiguration(
                    "depth_confirmation_hold_time_s"
                ),
                "depth_confirmation_minimum_frames": LaunchConfiguration(
                    "depth_confirmation_minimum_frames"
                ),
                "search_camera_width": LaunchConfiguration("camera_width"),
                "search_camera_height": LaunchConfiguration("camera_height"),
                "search_camera_fx": LaunchConfiguration("search_camera_fx"),
                "search_camera_fy": LaunchConfiguration("search_camera_fy"),
                "search_camera_cx": LaunchConfiguration("search_camera_cx"),
                "search_camera_cy": LaunchConfiguration("search_camera_cy"),
                "search_camera_horizontal_fov_rad": LaunchConfiguration(
                    "search_camera_horizontal_fov_rad"
                ),
                "search_camera_vertical_fov_rad": LaunchConfiguration(
                    "search_camera_vertical_fov_rad"
                ),
                "altitude_hold_position_gain": LaunchConfiguration(
                    "pursuer_altitude_position_gain"
                ),
                "altitude_hold_velocity_gain": LaunchConfiguration(
                    "pursuer_altitude_velocity_gain"
                ),
                "altitude_hold_max_speed_mps": LaunchConfiguration(
                    "pursuer_altitude_max_speed_mps"
                ),
                "altitude_hold_max_acceleration_mps2": LaunchConfiguration(
                    "pursuer_altitude_max_acceleration_mps2"
                ),
                "position_gain": 0.6,
                "velocity_feedforward": 1.0,
                "max_horizontal_speed_mps": 5.0,
                "max_vertical_speed_mps": 2.0,
                "standoff_distance_m": 3.0,
                "acceleration_position_gain": 0.8,
                "acceleration_velocity_gain": 1.2,
                "max_horizontal_acceleration_mps2": 3.0,
                "max_vertical_acceleration_mps2": 2.0,
                "corridor_vertical_separation_m": 30.0,
                "corridor_prediction_time_s": 0.8,
                "corridor_maximum_lead_distance_m": 2.0,
                "corridor_horizontal_position_gain": 0.45,
                "corridor_horizontal_velocity_gain": 1.5,
                "corridor_maximum_horizontal_acceleration_mps2": 1.5,
                "corridor_minimum_target_above_m": 0.5,
                "mpc_time_step_s": LaunchConfiguration("mpc_time_step_s"),
                "mpc_horizon_steps": LaunchConfiguration("mpc_horizon_steps"),
                "mpc_position_weight": LaunchConfiguration("mpc_position_weight"),
                "mpc_velocity_weight": LaunchConfiguration("mpc_velocity_weight"),
                "mpc_acceleration_weight": LaunchConfiguration(
                    "mpc_acceleration_weight"
                ),
                "mpc_terminal_weight_multiplier": LaunchConfiguration(
                    "mpc_terminal_weight_multiplier"
                ),
                "mpc_prediction_lookahead_s": LaunchConfiguration(
                    "mpc_prediction_lookahead_s"
                ),
                "mpc_maximum_horizontal_acceleration_mps2": LaunchConfiguration(
                    "mpc_maximum_horizontal_acceleration_mps2"
                ),
                "mpc_desired_vertical_separation_m": LaunchConfiguration(
                    "mpc_desired_vertical_separation_m"
                ),
                "mpc_maximum_vertical_acceleration_mps2": LaunchConfiguration(
                    "mpc_maximum_vertical_acceleration_mps2"
                ),
                "mpc_vertical_position_weight": LaunchConfiguration(
                    "mpc_vertical_position_weight"
                ),
                "mpc_vertical_velocity_weight": LaunchConfiguration(
                    "mpc_vertical_velocity_weight"
                ),
                "mpc_vertical_acceleration_weight": LaunchConfiguration(
                    "mpc_vertical_acceleration_weight"
                ),
                "mpc_vertical_absolute_hold_blend": LaunchConfiguration(
                    "mpc_vertical_absolute_hold_blend"
                ),
                "mpc_3d_entry_vertical_tolerance_m": LaunchConfiguration(
                    "mpc_3d_entry_vertical_tolerance_m"
                ),
                "mpc_3d_entry_vertical_speed_tolerance_mps": LaunchConfiguration(
                    "mpc_3d_entry_vertical_speed_tolerance_mps"
                ),
                "mpc_3d_entry_hold_time_s": LaunchConfiguration(
                    "mpc_3d_entry_hold_time_s"
                ),
                "pursuer_yaw_mode": LaunchConfiguration("pursuer_yaw_mode"),
                "pursuer_fixed_yaw_rad": LaunchConfiguration("pursuer_fixed_yaw_rad"),
                "pursuer_yaw_minimum_speed_mps": LaunchConfiguration(
                    "pursuer_yaw_minimum_speed_mps"
                ),
                "invalid_estimate_velocity_damping": LaunchConfiguration(
                    "invalid_estimate_velocity_damping"
                ),
                "invalid_estimate_maximum_acceleration_mps2": LaunchConfiguration(
                    "invalid_estimate_maximum_acceleration_mps2"
                ),
                "command_maximum_horizontal_speed_mps": LaunchConfiguration(
                    "command_maximum_horizontal_speed_mps"
                ),
                "command_speed_soft_zone_mps": LaunchConfiguration(
                    "command_speed_soft_zone_mps"
                ),
                "command_overspeed_braking_gain": LaunchConfiguration(
                    "command_overspeed_braking_gain"
                ),
                "command_maximum_braking_acceleration_mps2": LaunchConfiguration(
                    "command_maximum_braking_acceleration_mps2"
                ),
                "hover_thrust": LaunchConfiguration("pursuer_hover_thrust"),
                "max_thrust": LaunchConfiguration("pursuer_max_thrust"),
                "max_tilt_rad": LaunchConfiguration("pursuer_max_tilt_rad"),
                "alignment_maximum_tilt_rad": LaunchConfiguration(
                    "alignment_maximum_tilt_rad"
                ),
                "alignment_prediction_time_s": LaunchConfiguration(
                    "alignment_prediction_time_s"
                ),
                "alignment_entry_horizontal_distance_m": LaunchConfiguration(
                    "alignment_entry_horizontal_distance_m"
                ),
                "alignment_entry_relative_speed_mps": 0.8,
                "minimum_thrust": 0.1,
                "attitude_smoothing_time_constant_s": LaunchConfiguration(
                    "attitude_smoothing_time_constant_s"
                ),
                "maximum_attitude_slew_rate_rad_s": LaunchConfiguration(
                    "maximum_attitude_slew_rate_rad_s"
                ),
                "attitude_gain": 3.0,
                "max_body_rate_rad_s": 3.0,
                "alignment_feasibility_topic": "/tracking/alignment/feasibility",
                "feasibility_maximum_tilt_rad": 0.42,
                "feasibility_minimum_thrust_margin": 0.08,
                "feasibility_maximum_body_rate_rad_s": 2.0,
                "feasibility_maximum_altitude_loss_m": 1.2,
                "feasibility_minimum_range_m": 10.0,
                "feasibility_maximum_alignment_time_s": 60.0,
                "feasibility_maximum_available_thrust": 0.9,
                "mission_status_topic": "/tracking/mission/status",
                "aim_entry_horizontal_distance_m": LaunchConfiguration(
                    "aim_entry_horizontal_distance_m"
                ),
                "aim_exit_horizontal_distance_m": LaunchConfiguration(
                    "aim_exit_horizontal_distance_m"
                ),
                "capture_minimum_range_m": LaunchConfiguration(
                    "capture_minimum_range_m"
                ),
                "capture_maximum_range_m": LaunchConfiguration(
                    "capture_maximum_range_m"
                ),
                "midcourse_alignment_blend": LaunchConfiguration(
                    "midcourse_alignment_blend"
                ),
                "terminal_navigation_constant": LaunchConfiguration(
                    "terminal_navigation_constant"
                ),
                "terminal_minimum_closing_speed_mps": LaunchConfiguration(
                    "terminal_minimum_closing_speed_mps"
                ),
                "terminal_los_acceleration_mps2": LaunchConfiguration(
                    "terminal_los_acceleration_mps2"
                ),
                "terminal_acceleration_feedforward_gain": LaunchConfiguration(
                    "terminal_acceleration_feedforward_gain"
                ),
                "terminal_maximum_horizontal_acceleration_mps2": LaunchConfiguration(
                    "terminal_maximum_horizontal_acceleration_mps2"
                ),
                "terminal_maximum_vertical_acceleration_mps2": LaunchConfiguration(
                    "terminal_maximum_vertical_acceleration_mps2"
                ),
                "terminal_alignment_blend": LaunchConfiguration(
                    "terminal_alignment_blend"
                ),
                "terminal_capture_alignment_blend": LaunchConfiguration(
                    "terminal_capture_alignment_blend"
                ),
                "terminal_alignment_weight_slew_rate_per_s": LaunchConfiguration(
                    "terminal_alignment_weight_slew_rate_per_s"
                ),
                "estimate_measurement_hold_s": LaunchConfiguration(
                    "estimate_measurement_hold_s"
                ),
                "collision_exit_range_m": LaunchConfiguration(
                    "collision_exit_range_m"
                ),
                "alignment_ready_error_rad": LaunchConfiguration(
                    "alignment_ready_error_rad"
                ),
                "takeoff_ready_tolerance_m": LaunchConfiguration(
                    "takeoff_ready_tolerance_m"
                ),
                "altitude_correction_threshold_m": 0.8,
                "mission_minimum_target_above_m": 0.5,
                "tracking_quality_topic": "/tracking/quality_gate",
                "quality_maximum_position_variance": 4.0,
                "quality_maximum_innovation_mahalanobis": 11.345,
                "quality_maximum_observation_age_s": 0.2,
                "quality_maximum_estimate_age_s": 0.2,
                "capture_window_topic": "/tracking/capture_window",
                "virtual_capture_event_topic": "/tracking/virtual_capture_event",
                "safety_status_topic": "/tracking/safety/status",
                "mission_complete_topic": "/tracking/mission/complete",
                "required_virtual_captures": LaunchConfiguration(
                    "required_virtual_captures"
                ),
                "target_speed_mps": LaunchConfiguration("target_speed_mps"),
                "target_motion_delay_s": LaunchConfiguration("target_motion_delay_s"),
                "attack_enable_delay_s": LaunchConfiguration("attack_enable_delay_s"),
                "target_stable_motion_time_s": LaunchConfiguration(
                    "target_stable_motion_time_s"
                ),
                "reattack_cooldown_s": LaunchConfiguration("reattack_cooldown_s"),
                "post_capture_recoil_duration_s": LaunchConfiguration(
                    "post_capture_recoil_duration_s"
                ),
                "post_capture_recoil_acceleration_mps2": LaunchConfiguration(
                    "post_capture_recoil_acceleration_mps2"
                ),
                "safety_minimum_range_m": LaunchConfiguration("safety_minimum_range_m"),
                "safety_maximum_tilt_rad": LaunchConfiguration("safety_maximum_tilt_rad"),
                "safety_maximum_speed_mps": LaunchConfiguration("safety_maximum_speed_mps"),
                "safety_maximum_altitude_error_m": LaunchConfiguration(
                    "safety_maximum_altitude_error_m"
                ),
                "recovery_maximum_tilt_rad": LaunchConfiguration(
                    "recovery_maximum_tilt_rad"
                ),
                "recovery_maximum_speed_mps": LaunchConfiguration(
                    "recovery_maximum_speed_mps"
                ),
                "recovery_maximum_altitude_error_m": LaunchConfiguration(
                    "recovery_maximum_altitude_error_m"
                ),
                "recovery_required_hold_time_s": LaunchConfiguration(
                    "recovery_required_hold_time_s"
                ),
                "capture_maximum_pixel_error_px": LaunchConfiguration(
                    "capture_maximum_pixel_error_px"
                ),
                "capture_maximum_alignment_error_rad": LaunchConfiguration(
                    "capture_maximum_alignment_error_rad"
                ),
                "capture_maximum_los_rate_rad_s": LaunchConfiguration(
                    "capture_maximum_los_rate_rad_s"
                ),
                "capture_maximum_predicted_miss_distance_m": 35.0,
                "capture_maximum_time_to_closest_approach_s": 15.0,
                "capture_required_hold_time_s": LaunchConfiguration(
                    "capture_required_hold_time_s"
                ),
                "capture_minimum_distinct_frames": LaunchConfiguration(
                    "capture_minimum_distinct_frames"
                ),
                "body_rate_roll_pitch_gain": 4.0,
                "body_rate_yaw_gain": 2.0,
                "body_rate_maximum_roll_pitch_rate_rad_s": 2.5,
                "body_rate_maximum_yaw_rate_rad_s": 1.5,
                "capture_camera_cx": LaunchConfiguration("camera_cx"),
                "capture_camera_cy": LaunchConfiguration("camera_cy"),
            }],
            )],
        ),
        Node(
            package="px4_offboard_tracking",
            executable="target_observation",
            namespace="perception",
            name="target_observation",
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_camera_observation")),
            parameters=[{
                "relative_state_topic": relative_state_topic,
                "observation_topic": LaunchConfiguration("camera_observation_topic"),
                "attitude_topic": LaunchConfiguration("pursuer_attitude_topic"),
                "common_frame_id": common_frame_id,
                "camera_frame_id": LaunchConfiguration("camera_frame_id"),
                "camera_rate_hz": LaunchConfiguration("camera_rate_hz"),
                "camera_width": LaunchConfiguration("camera_width"),
                "camera_height": LaunchConfiguration("camera_height"),
                "camera_fx": LaunchConfiguration("camera_fx"),
                "camera_fy": LaunchConfiguration("camera_fy"),
                "camera_cx": LaunchConfiguration("camera_cx"),
                "camera_cy": LaunchConfiguration("camera_cy"),
                "search_camera_fx": LaunchConfiguration("search_camera_fx"),
                "search_camera_fy": LaunchConfiguration("search_camera_fy"),
                "search_camera_cx": LaunchConfiguration("search_camera_cx"),
                "search_camera_cy": LaunchConfiguration("search_camera_cy"),
                "camera_minimum_depth_m": LaunchConfiguration("camera_minimum_depth_m"),
                "camera_translation_body_x_m": LaunchConfiguration(
                    "camera_translation_body_x_m"
                ),
                "camera_translation_body_y_m": LaunchConfiguration(
                    "camera_translation_body_y_m"
                ),
                "camera_translation_body_z_m": LaunchConfiguration(
                    "camera_translation_body_z_m"
                ),
                "camera_q_body_to_optical_w": LaunchConfiguration(
                    "camera_q_body_to_optical_w"
                ),
                "camera_q_body_to_optical_x": LaunchConfiguration(
                    "camera_q_body_to_optical_x"
                ),
                "camera_q_body_to_optical_y": LaunchConfiguration(
                    "camera_q_body_to_optical_y"
                ),
                "camera_q_body_to_optical_z": LaunchConfiguration(
                    "camera_q_body_to_optical_z"
                ),
                "depth_camera_horizontal_fov_rad": LaunchConfiguration(
                    "depth_camera_horizontal_fov_rad"
                ),
                "depth_camera_vertical_fov_rad": LaunchConfiguration(
                    "depth_camera_vertical_fov_rad"
                ),
                "search_camera_horizontal_fov_rad": LaunchConfiguration(
                    "search_camera_horizontal_fov_rad"
                ),
                "search_camera_vertical_fov_rad": LaunchConfiguration(
                    "search_camera_vertical_fov_rad"
                ),
                "search_camera_q_body_to_optical_w": LaunchConfiguration(
                    "search_camera_q_body_to_optical_w"
                ),
                "search_camera_q_body_to_optical_x": LaunchConfiguration(
                    "search_camera_q_body_to_optical_x"
                ),
                "search_camera_q_body_to_optical_y": LaunchConfiguration(
                    "search_camera_q_body_to_optical_y"
                ),
                "search_camera_q_body_to_optical_z": LaunchConfiguration(
                    "search_camera_q_body_to_optical_z"
                ),
                "enable_pixel_noise": LaunchConfiguration("enable_pixel_noise"),
                "pixel_noise_stddev_px": LaunchConfiguration(
                    "pixel_noise_stddev_px"
                ),
                "enable_depth_noise": LaunchConfiguration("enable_depth_noise"),
                "depth_noise_stddev_m": LaunchConfiguration(
                    "depth_noise_stddev_m"
                ),
                "enable_latency": LaunchConfiguration("enable_latency"),
                "latency_ms": LaunchConfiguration("latency_ms"),
                "enable_dropout": LaunchConfiguration("enable_dropout"),
                "dropout_probability": LaunchConfiguration("dropout_probability"),
                "enable_outliers": LaunchConfiguration("enable_outliers"),
                "outlier_probability": LaunchConfiguration("outlier_probability"),
                "outlier_min_depth_m": LaunchConfiguration("outlier_min_depth_m"),
                "outlier_max_depth_m": LaunchConfiguration("outlier_max_depth_m"),
                "random_seed": LaunchConfiguration("observation_random_seed"),
            }],
        ),
        Node(
            package="px4_offboard_tracking",
            executable="relative_estimator",
            namespace="perception",
            name="relative_estimator",
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_relative_estimator")),
            parameters=[{
                "observation_topic": LaunchConfiguration("camera_observation_topic"),
                "estimate_topic": relative_estimate_topic,
                "attitude_topic": LaunchConfiguration("pursuer_attitude_topic"),
                "common_frame_id": common_frame_id,
                "estimator_rate_hz": LaunchConfiguration("estimator_rate_hz"),
                "max_data_age_s": LaunchConfiguration("estimator_max_data_age_s"),
                "max_position_variance": LaunchConfiguration(
                    "estimator_max_position_variance"
                ),
                "attitude_sync_tolerance_s": LaunchConfiguration(
                    "attitude_sync_tolerance_s"
                ),
                "attitude_buffer_s": LaunchConfiguration("attitude_buffer_s"),
                "measurement_pixel_stddev_px": LaunchConfiguration(
                    "measurement_pixel_stddev_px"
                ),
                "measurement_depth_stddev_m": LaunchConfiguration(
                    "measurement_depth_stddev_m"
                ),
                "acceleration_noise_stddev_mps2": LaunchConfiguration(
                    "acceleration_noise_stddev_mps2"
                ),
                "initial_position_stddev_m": LaunchConfiguration(
                    "initial_position_stddev_m"
                ),
                "initial_velocity_stddev_mps": LaunchConfiguration(
                    "initial_velocity_stddev_mps"
                ),
                "innovation_gate_mahalanobis": LaunchConfiguration(
                    "innovation_gate_mahalanobis"
                ),
                "camera_width": LaunchConfiguration("camera_width"),
                "camera_height": LaunchConfiguration("camera_height"),
                "camera_fx": LaunchConfiguration("camera_fx"),
                "camera_fy": LaunchConfiguration("camera_fy"),
                "camera_cx": LaunchConfiguration("camera_cx"),
                "camera_cy": LaunchConfiguration("camera_cy"),
                "camera_minimum_depth_m": LaunchConfiguration("camera_minimum_depth_m"),
                "camera_translation_body_x_m": LaunchConfiguration(
                    "camera_translation_body_x_m"
                ),
                "camera_translation_body_y_m": LaunchConfiguration(
                    "camera_translation_body_y_m"
                ),
                "camera_translation_body_z_m": LaunchConfiguration(
                    "camera_translation_body_z_m"
                ),
                "camera_q_body_to_optical_w": LaunchConfiguration(
                    "camera_q_body_to_optical_w"
                ),
                "camera_q_body_to_optical_x": LaunchConfiguration(
                    "camera_q_body_to_optical_x"
                ),
                "camera_q_body_to_optical_y": LaunchConfiguration(
                    "camera_q_body_to_optical_y"
                ),
                "camera_q_body_to_optical_z": LaunchConfiguration(
                    "camera_q_body_to_optical_z"
                ),
            }],
        ),
        Node(
            package="px4_offboard_tracking",
            executable="relative_kinematics_node",
            namespace="perception",
            name="relative_kinematics",
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_relative_kinematics")),
            parameters=[{
                "estimate_topic": relative_estimate_topic,
                "kinematics_topic": relative_kinematics_topic,
            }],
        ),
        Node(
            package="px4_offboard_tracking",
            executable="trajectory_recorder",
            namespace="visualization",
            name="trajectory_recorder",
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_trajectory_recorder")),
            parameters=[{
                "target_state_topic": target_state_topic,
                "relative_state_topic": relative_state_topic,
                "observation_topic": LaunchConfiguration("camera_observation_topic"),
                "estimate_topic": relative_estimate_topic,
                "kinematics_topic": relative_kinematics_topic,
                "alignment_topic": "/tracking/alignment/feasibility",
                "output_directory": LaunchConfiguration(
                    "trajectory_output_directory"
                ),
                "filename": LaunchConfiguration("trajectory_filename"),
                "record_rate_hz": LaunchConfiguration(
                    "trajectory_record_rate_hz"
                ),
                "mission_topic": "/tracking/mission/status",
                "quality_topic": "/tracking/quality_gate",
                "capture_topic": "/tracking/capture_window",
                "safety_topic": "/tracking/safety/status",
                "pursuer_attitude_topic": "/px4_1/fmu/out/vehicle_attitude",
            }],
        ),
        completion_monitor,
        RegisterEventHandler(
            OnProcessExit(
                target_action=completion_monitor,
                on_exit=[EmitEvent(event=Shutdown(reason="mission complete"))],
            )
        ),
    ])
