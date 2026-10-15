"""实机/纯控制入口。

本 launch 不启动任何仿真感知或评估节点。外部真实相机/YOLO/双目节点只需发布：
  /perception/search/observation
  /perception/depth/observation

实物敌机由另一台电脑独立Offboard控制；本入口只连接拦截机自身/fmu。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    search_topic = LaunchConfiguration("search_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    estimate_topic = LaunchConfiguration("estimate_topic")
    pursuer_prefix = LaunchConfiguration("pursuer_prefix")
    pursuer_attitude_topic = LaunchConfiguration("pursuer_attitude_topic")

    return LaunchDescription(
        [
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
            DeclareLaunchArgument(
                "search_topic", default_value="/perception/search/observation"
            ),
            DeclareLaunchArgument(
                "depth_topic", default_value="/perception/depth/observation"
            ),
            DeclareLaunchArgument(
                "estimate_topic", default_value="/tracking/relative/estimate"
            ),
            DeclareLaunchArgument(
                "shot_topic", default_value="/tracking/virtual_shot_event"
            ),
            DeclareLaunchArgument("pursuer_prefix", default_value="/fmu"),
            DeclareLaunchArgument(
                "pursuer_attitude_topic",
                default_value="/fmu/out/vehicle_attitude",
            ),
            DeclareLaunchArgument("control_enabled", default_value="true"),
            DeclareLaunchArgument("auto_engage", default_value="false"),
            DeclareLaunchArgument("takeoff_enabled", default_value="false"),
            DeclareLaunchArgument(
                "auto_disable_on_offboard_exit", default_value="true"
            ),
            DeclareLaunchArgument(
                "enable_topic", default_value="/tracking/pure_interceptor/enable"
            ),
            DeclareLaunchArgument("takeoff_height_m", default_value="5.0"),
            DeclareLaunchArgument("minimum_flight_height_m", default_value="3.0"),
            DeclareLaunchArgument("attack_enable_delay_s", default_value="0.0"),
            DeclareLaunchArgument("first_attack_tracking_hold_s", default_value="3.0"),
            DeclareLaunchArgument(
                "terminal_alignment_tracking_hold_s", default_value="2.0"
            ),
            DeclareLaunchArgument("terminal_alignment_ramp_s", default_value="1.0"),
            DeclareLaunchArgument("required_captures", default_value="2"),
            DeclareLaunchArgument("recoil_duration_s", default_value="0.2"),
            DeclareLaunchArgument(
                "recoil_opening_speed_mps", default_value="1.0"
            ),
            DeclareLaunchArgument("reattack_cooldown_s", default_value="3.0"),
            DeclareLaunchArgument("maximum_tilt_rad", default_value="0.35"),
            DeclareLaunchArgument("hover_thrust", default_value="0.42"),
            DeclareLaunchArgument("maximum_thrust", default_value="0.70"),
            DeclareLaunchArgument("maximum_acceleration_mps2", default_value="5.0"),
            DeclareLaunchArgument("desired_closing_speed_mps", default_value="2.0"),
            DeclareLaunchArgument("radial_velocity_gain", default_value="0.8"),
            DeclareLaunchArgument("transverse_velocity_gain", default_value="1.0"),
            DeclareLaunchArgument(
                "target_acceleration_feedforward_gain", default_value="0.2"
            ),
            DeclareLaunchArgument(
                "target_acceleration_feedforward_minimum_ca_probability",
                default_value="0.25",
            ),
            DeclareLaunchArgument(
                "target_acceleration_feedforward_full_ca_probability",
                default_value="0.75",
            ),
            DeclareLaunchArgument(
                "target_acceleration_feedforward_maximum_stddev_mps2",
                default_value="5.0",
            ),
            DeclareLaunchArgument(
                "target_acceleration_feedforward_filter_time_constant_s",
                default_value="0.2",
            ),
            DeclareLaunchArgument(
                "target_acceleration_feedforward_maximum_mps2",
                default_value="4.0",
            ),
            DeclareLaunchArgument("capture_minimum_range_m", default_value="15.0"),
            DeclareLaunchArgument("capture_maximum_range_m", default_value="35.0"),
            DeclareLaunchArgument("preferred_capture_range_m", default_value="17.0"),
            DeclareLaunchArgument(
                "launch_position_planner_enabled", default_value="true"
            ),
            DeclareLaunchArgument(
                "launch_position_planner_blend", default_value="1.0"
            ),
            DeclareLaunchArgument(
                "launch_planner_minimum_geometry_blend", default_value="0.55"
            ),
            DeclareLaunchArgument(
                "launch_planner_full_inbound_speed_mps", default_value="4.0"
            ),
            DeclareLaunchArgument(
                "launch_planner_minimum_horizon_s", default_value="0.5"
            ),
            DeclareLaunchArgument(
                "launch_planner_maximum_horizon_s", default_value="8.0"
            ),
            DeclareLaunchArgument(
                "launch_planner_preferred_range_m", default_value="25.0"
            ),
            DeclareLaunchArgument(
                "launch_planner_maximum_horizontal_acceleration_mps2",
                default_value="1.0",
            ),
            DeclareLaunchArgument(
                "launch_planner_maximum_vertical_acceleration_mps2",
                default_value="0.5",
            ),
            DeclareLaunchArgument(
                "launch_planner_maximum_speed_mps", default_value="12.0"
            ),
            DeclareLaunchArgument(
                "launch_planner_terminal_relative_speed_weight",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                "launch_planner_terminal_relative_speed_scale_mps",
                default_value="4.0",
            ),
            DeclareLaunchArgument(
                "launch_planner_closing_speed_reserve_mps", default_value="4.0"
            ),
            DeclareLaunchArgument(
                "launch_planner_excess_speed_weight", default_value="0.2"
            ),
            DeclareLaunchArgument(
                "launch_planner_excess_speed_scale_mps", default_value="2.0"
            ),
            DeclareLaunchArgument(
                "capture_maximum_approach_speed_mps", default_value="4.0"
            ),
            DeclareLaunchArgument(
                "handover_maximum_acceleration_mps2", default_value="1.5"
            ),
            DeclareLaunchArgument("egress_distance_m", default_value="5.0"),
            DeclareLaunchArgument("egress_speed_mps", default_value="1.0"),
            DeclareLaunchArgument("command_maximum_speed_mps", default_value="12.0"),
            DeclareLaunchArgument("command_speed_soft_zone_mps", default_value="2.5"),
            DeclareLaunchArgument("command_maximum_vertical_speed_mps", default_value="1.0"),
            DeclareLaunchArgument("safety_maximum_speed_mps", default_value="14.0"),
            DeclareLaunchArgument("search_pattern_enabled", default_value="true"),
            DeclareLaunchArgument("search_yaw_rate_rad_s", default_value="0.35"),
            DeclareLaunchArgument("search_prediction_hold_s", default_value="1.5"),
            DeclareLaunchArgument(
                "known_target_direction_enabled", default_value="false"
            ),
            DeclareLaunchArgument(
                "known_target_bearing_ned_rad", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "known_target_search_hold_s", default_value="6.0"
            ),
            DeclareLaunchArgument(
                "known_target_search_acceleration_mps2", default_value="1.0"
            ),
            DeclareLaunchArgument(
                "known_target_search_acceleration_duration_s", default_value="1.0"
            ),
            DeclareLaunchArgument(
                "known_target_course_enabled", default_value="false"
            ),
            DeclareLaunchArgument(
                "known_target_course_ned_rad", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "known_target_search_velocity_mps", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "known_target_velocity_gain", default_value="1.0"
            ),
            DeclareLaunchArgument(
                "known_target_course_minimum_speed_fraction", default_value="0.35"
            ),
            DeclareLaunchArgument("search_pattern_turns", default_value="1.0"),
            DeclareLaunchArgument(
                "search_repeat_until_acquired", default_value="false"
            ),
            DeclareLaunchArgument("fov_recovery_entry_hold_s", default_value="0.15"),
            DeclareLaunchArgument("tracking_invalid_hysteresis_s", default_value="0.2"),
            DeclareLaunchArgument("depth_loss_hysteresis_s", default_value="0.2"),
            DeclareLaunchArgument("completion_braking_hold_s", default_value="0.5"),
            DeclareLaunchArgument(
                "completion_position_hold_maximum_speed_mps", default_value="0.5"
            ),
            DeclareLaunchArgument(
                "completion_position_hold_settle_s", default_value="0.5"
            ),
            DeclareLaunchArgument(
                "completion_safe_hover_maximum_speed_mps", default_value="0.3"
            ),
            DeclareLaunchArgument(
                "completion_safe_hover_hold_s", default_value="2.0"
            ),
            DeclareLaunchArgument(
                "minimum_height_prediction_horizon_s", default_value="1.0"
            ),
            DeclareLaunchArgument(
                "minimum_height_recovery_m", default_value="3.3"
            ),
            DeclareLaunchArgument(
                "shot_timing_prediction_horizon_s", default_value="0.04"
            ),
            DeclareLaunchArgument(
                "shot_timing_error_rate_filter_time_constant_s",
                default_value="0.08",
            ),
            DeclareLaunchArgument(
                "shot_timing_maximum_absolute_error_rate_rad_s",
                default_value="0.5",
            ),
            DeclareLaunchArgument(
                "shot_timing_minimum_ready_hold_s", default_value="0.04"
            ),
            Node(
                package="px4_offboard_tracking",
                executable="relative_estimator",
                namespace="estimation",
                name="relative_estimator",
                output="screen",
                parameters=[
                    {
                        "observation_topic": depth_topic,
                        "estimate_topic": estimate_topic,
                        "shot_topic": LaunchConfiguration("shot_topic"),
                        "projectile_speed_mps": 400.0,
                        "shot_maximum_time_of_flight_s": 0.25,
                        "shot_maximum_lead_angle_rad": 0.06981317007977318,
                        "shot_aim_tolerance_rad": 0.017453292519943295,
                        "shot_maximum_lateral_sigma_m": 0.75,
                        "attitude_topic": pursuer_attitude_topic,
                        "common_frame_id": "map_ned",
                        "camera_fx": 12847.019977569425,
                        "camera_fy": 6124.984365187126,
                        "camera_cx": 1919.5,
                        "camera_cy": 1079.5,
                        "camera_width": 3840,
                        "camera_height": 2160,
                    }
                ],
            ),
            Node(
                package="px4_offboard_tracking",
                executable="pure_interceptor_controller",
                namespace="pursuer",
                name="pure_interceptor_controller",
                output="screen",
                parameters=[
                    {
                        "topic_prefix": pursuer_prefix,
                        "target_system": 1,
                        "control_enabled": LaunchConfiguration("control_enabled"),
                        "auto_engage": LaunchConfiguration("auto_engage"),
                        "takeoff_enabled": LaunchConfiguration("takeoff_enabled"),
                        "auto_disable_on_offboard_exit": LaunchConfiguration(
                            "auto_disable_on_offboard_exit"
                        ),
                        "enable_topic": LaunchConfiguration("enable_topic"),
                        # 2026-08-14 field override: in flight the USB search
                        # branch PTS lag climbs to a ~280 ms plateau (ground
                        # full-stack 73 ms; suspected power/thermal throttle
                        # under motor load), starving the 0.25 s freshness
                        # gate and blocking search acquisition.  0.45 s keeps
                        # coverage continuous at 15 fps with margin.
                        "observation_timeout_s": 0.45,
                        "search_topic": search_topic,
                        "depth_topic": depth_topic,
                        "estimate_topic": estimate_topic,
                        "takeoff_height_m": LaunchConfiguration("takeoff_height_m"),
                        "search_pattern_enabled": LaunchConfiguration(
                            "search_pattern_enabled"
                        ),
                        "search_yaw_rate_rad_s": LaunchConfiguration(
                            "search_yaw_rate_rad_s"
                        ),
                        "search_prediction_hold_s": LaunchConfiguration(
                            "search_prediction_hold_s"
                        ),
                        "known_target_direction_enabled": LaunchConfiguration(
                            "known_target_direction_enabled"
                        ),
                        "known_target_bearing_ned_rad": LaunchConfiguration(
                            "known_target_bearing_ned_rad"
                        ),
                        "known_target_search_hold_s": LaunchConfiguration(
                            "known_target_search_hold_s"
                        ),
                        "known_target_search_acceleration_mps2": LaunchConfiguration(
                            "known_target_search_acceleration_mps2"
                        ),
                        "known_target_search_acceleration_duration_s":
                            LaunchConfiguration(
                                "known_target_search_acceleration_duration_s"
                            ),
                        "known_target_course_enabled": LaunchConfiguration(
                            "known_target_course_enabled"
                        ),
                        "known_target_course_ned_rad": LaunchConfiguration(
                            "known_target_course_ned_rad"
                        ),
                        "known_target_search_velocity_mps": LaunchConfiguration(
                            "known_target_search_velocity_mps"
                        ),
                        "known_target_velocity_gain": LaunchConfiguration(
                            "known_target_velocity_gain"
                        ),
                        "known_target_course_minimum_speed_fraction":
                            LaunchConfiguration(
                                "known_target_course_minimum_speed_fraction"
                            ),
                        "search_pattern_turns": LaunchConfiguration(
                            "search_pattern_turns"
                        ),
                        "search_repeat_until_acquired": LaunchConfiguration(
                            "search_repeat_until_acquired"
                        ),
                        "minimum_flight_height_m": LaunchConfiguration(
                            "minimum_flight_height_m"
                        ),
                        "attack_enable_delay_s": LaunchConfiguration(
                            "attack_enable_delay_s"
                        ),
                        "first_attack_tracking_hold_s": LaunchConfiguration(
                            "first_attack_tracking_hold_s"
                        ),
                        "terminal_alignment_tracking_hold_s": LaunchConfiguration(
                            "terminal_alignment_tracking_hold_s"
                        ),
                        "terminal_alignment_ramp_s": LaunchConfiguration(
                            "terminal_alignment_ramp_s"
                        ),
                        "required_captures": LaunchConfiguration(
                            "required_captures"
                        ),
                        "recoil_duration_s": LaunchConfiguration(
                            "recoil_duration_s"
                        ),
                        "recoil_opening_speed_mps": LaunchConfiguration(
                            "recoil_opening_speed_mps"
                        ),
                        "reattack_cooldown_s": LaunchConfiguration(
                            "reattack_cooldown_s"
                        ),
                        "maximum_tilt_rad": LaunchConfiguration("maximum_tilt_rad"),
                        "handover_maximum_tilt_rad": LaunchConfiguration(
                            "maximum_tilt_rad"
                        ),
                        "hover_thrust": LaunchConfiguration("hover_thrust"),
                        "maximum_thrust": LaunchConfiguration("maximum_thrust"),
                        "maximum_acceleration_mps2": LaunchConfiguration(
                            "maximum_acceleration_mps2"
                        ),
                        "desired_closing_speed_mps": LaunchConfiguration(
                            "desired_closing_speed_mps"
                        ),
                        "radial_velocity_gain": LaunchConfiguration(
                            "radial_velocity_gain"
                        ),
                        "transverse_velocity_gain": LaunchConfiguration(
                            "transverse_velocity_gain"
                        ),
                        "target_acceleration_feedforward_gain":
                            LaunchConfiguration(
                                "target_acceleration_feedforward_gain"
                            ),
                        "target_acceleration_feedforward_minimum_ca_probability":
                            LaunchConfiguration(
                                "target_acceleration_feedforward_minimum_ca_probability"
                            ),
                        "target_acceleration_feedforward_full_ca_probability":
                            LaunchConfiguration(
                                "target_acceleration_feedforward_full_ca_probability"
                            ),
                        "target_acceleration_feedforward_maximum_stddev_mps2":
                            LaunchConfiguration(
                                "target_acceleration_feedforward_maximum_stddev_mps2"
                            ),
                        "target_acceleration_feedforward_filter_time_constant_s":
                            LaunchConfiguration(
                                "target_acceleration_feedforward_filter_time_constant_s"
                            ),
                        "target_acceleration_feedforward_maximum_mps2":
                            LaunchConfiguration(
                                "target_acceleration_feedforward_maximum_mps2"
                            ),
                        "capture_minimum_range_m": LaunchConfiguration(
                            "capture_minimum_range_m"
                        ),
                        "capture_maximum_range_m": LaunchConfiguration(
                            "capture_maximum_range_m"
                        ),
                        "preferred_capture_range_m": LaunchConfiguration(
                            "preferred_capture_range_m"
                        ),
                        "launch_position_planner_enabled": LaunchConfiguration(
                            "launch_position_planner_enabled"
                        ),
                        "launch_position_planner_blend": LaunchConfiguration(
                            "launch_position_planner_blend"
                        ),
                        "launch_planner_minimum_geometry_blend":
                            LaunchConfiguration(
                                "launch_planner_minimum_geometry_blend"
                            ),
                        "launch_planner_full_inbound_speed_mps":
                            LaunchConfiguration(
                                "launch_planner_full_inbound_speed_mps"
                            ),
                        "launch_planner_minimum_horizon_s": LaunchConfiguration(
                            "launch_planner_minimum_horizon_s"
                        ),
                        "launch_planner_maximum_horizon_s": LaunchConfiguration(
                            "launch_planner_maximum_horizon_s"
                        ),
                        "launch_planner_preferred_range_m": LaunchConfiguration(
                            "launch_planner_preferred_range_m"
                        ),
                        "launch_planner_maximum_horizontal_acceleration_mps2":
                            LaunchConfiguration(
                                "launch_planner_maximum_horizontal_acceleration_mps2"
                            ),
                        "launch_planner_maximum_vertical_acceleration_mps2":
                            LaunchConfiguration(
                                "launch_planner_maximum_vertical_acceleration_mps2"
                            ),
                        "launch_planner_maximum_speed_mps": LaunchConfiguration(
                            "launch_planner_maximum_speed_mps"
                        ),
                        "launch_planner_terminal_relative_speed_weight":
                            LaunchConfiguration(
                                "launch_planner_terminal_relative_speed_weight"
                            ),
                        "launch_planner_terminal_relative_speed_scale_mps":
                            LaunchConfiguration(
                                "launch_planner_terminal_relative_speed_scale_mps"
                            ),
                        "launch_planner_closing_speed_reserve_mps":
                            LaunchConfiguration(
                                "launch_planner_closing_speed_reserve_mps"
                            ),
                        "launch_planner_excess_speed_weight": LaunchConfiguration(
                            "launch_planner_excess_speed_weight"
                        ),
                        "launch_planner_excess_speed_scale_mps":
                            LaunchConfiguration(
                                "launch_planner_excess_speed_scale_mps"
                            ),
                        "capture_maximum_approach_speed_mps": LaunchConfiguration(
                            "capture_maximum_approach_speed_mps"
                        ),
                        "handover_maximum_acceleration_mps2": LaunchConfiguration(
                            "handover_maximum_acceleration_mps2"
                        ),
                        "egress_distance_m": LaunchConfiguration(
                            "egress_distance_m"
                        ),
                        "egress_speed_mps": LaunchConfiguration("egress_speed_mps"),
                        "command_maximum_speed_mps": LaunchConfiguration(
                            "command_maximum_speed_mps"
                        ),
                        "command_speed_soft_zone_mps": LaunchConfiguration(
                            "command_speed_soft_zone_mps"
                        ),
                        "command_maximum_vertical_speed_mps": LaunchConfiguration(
                            "command_maximum_vertical_speed_mps"
                        ),
                        "safety_maximum_speed_mps": LaunchConfiguration(
                            "safety_maximum_speed_mps"
                        ),
                        "fov_recovery_entry_hold_s": LaunchConfiguration(
                            "fov_recovery_entry_hold_s"
                        ),
                        "tracking_invalid_hysteresis_s": LaunchConfiguration(
                            "tracking_invalid_hysteresis_s"
                        ),
                        "depth_loss_hysteresis_s": LaunchConfiguration(
                            "depth_loss_hysteresis_s"
                        ),
                        "completion_braking_hold_s": LaunchConfiguration(
                            "completion_braking_hold_s"
                        ),
                        "completion_position_hold_maximum_speed_mps":
                            LaunchConfiguration(
                                "completion_position_hold_maximum_speed_mps"
                            ),
                        "completion_position_hold_settle_s": LaunchConfiguration(
                            "completion_position_hold_settle_s"
                        ),
                        "completion_safe_hover_maximum_speed_mps":
                            LaunchConfiguration(
                                "completion_safe_hover_maximum_speed_mps"
                            ),
                        "completion_safe_hover_hold_s": LaunchConfiguration(
                            "completion_safe_hover_hold_s"
                        ),
                        "minimum_height_prediction_horizon_s": LaunchConfiguration(
                            "minimum_height_prediction_horizon_s"
                        ),
                        "minimum_height_recovery_m": LaunchConfiguration(
                            "minimum_height_recovery_m"
                        ),
                        "shot_timing_prediction_horizon_s":
                            LaunchConfiguration(
                                "shot_timing_prediction_horizon_s"
                            ),
                        "shot_timing_error_rate_filter_time_constant_s":
                            LaunchConfiguration(
                                "shot_timing_error_rate_filter_time_constant_s"
                            ),
                        "shot_timing_maximum_absolute_error_rate_rad_s":
                            LaunchConfiguration(
                                "shot_timing_maximum_absolute_error_rate_rad_s"
                            ),
                        "shot_timing_minimum_ready_hold_s":
                            LaunchConfiguration(
                                "shot_timing_minimum_ready_hold_s"
                            ),
                    }
                ],
            ),
        ]
    )
