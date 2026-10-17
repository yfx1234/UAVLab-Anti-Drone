"""完整预测跟踪/打击的显式实机人工入口。

敌机由另一台电脑独立控制；本入口只连接拦截机PX4和两路实机相机观测。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    base = (
        Path(get_package_share_directory("px4_offboard_tracking"))
        / "launch"
        / "pure_interceptor_real.launch.py"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("pursuer_prefix", default_value="/fmu"),
            DeclareLaunchArgument(
                "search_topic", default_value="/perception/search/observation"
            ),
            DeclareLaunchArgument(
                "depth_topic", default_value="/perception/depth/observation"
            ),
            DeclareLaunchArgument(
                "pursuer_attitude_topic",
                default_value="/fmu/out/vehicle_attitude",
            ),
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
            DeclareLaunchArgument("command_maximum_speed_mps", default_value="12.0"),
            DeclareLaunchArgument("command_speed_soft_zone_mps", default_value="2.5"),
            DeclareLaunchArgument("command_maximum_vertical_speed_mps", default_value="1.0"),
            DeclareLaunchArgument("safety_maximum_speed_mps", default_value="14.0"),
            DeclareLaunchArgument(
                "handover_maximum_acceleration_mps2", default_value="1.5"
            ),
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
            DeclareLaunchArgument("fov_recovery_entry_hold_s", default_value="0.15"),
            DeclareLaunchArgument("tracking_invalid_hysteresis_s", default_value="0.2"),
            DeclareLaunchArgument("depth_loss_hysteresis_s", default_value="0.2"),
            DeclareLaunchArgument("completion_braking_hold_s", default_value="0.5"),
            DeclareLaunchArgument(
                "completion_position_hold_maximum_speed_mps", default_value="0.5"
            ),
            DeclareLaunchArgument("attack_enable_delay_s", default_value="0.0"),
            DeclareLaunchArgument(
                "first_attack_tracking_hold_s", default_value="3.0"
            ),
            DeclareLaunchArgument(
                "terminal_alignment_tracking_hold_s", default_value="2.0"
            ),
            DeclareLaunchArgument("terminal_alignment_ramp_s", default_value="1.0"),
            DeclareLaunchArgument("recoil_duration_s", default_value="0.2"),
            DeclareLaunchArgument(
                "recoil_opening_speed_mps", default_value="1.0"
            ),
            DeclareLaunchArgument("reattack_cooldown_s", default_value="3.0"),
            DeclareLaunchArgument("launch_position_planner_enabled", default_value="true"),
            DeclareLaunchArgument("launch_position_planner_blend", default_value="1.0"),
            DeclareLaunchArgument("launch_planner_minimum_geometry_blend", default_value="0.55"),
            DeclareLaunchArgument("launch_planner_full_inbound_speed_mps", default_value="4.0"),
            DeclareLaunchArgument("launch_planner_minimum_horizon_s", default_value="0.5"),
            DeclareLaunchArgument("launch_planner_maximum_horizon_s", default_value="8.0"),
            DeclareLaunchArgument("launch_planner_preferred_range_m", default_value="25.0"),
            DeclareLaunchArgument(
                "launch_planner_maximum_horizontal_acceleration_mps2",
                default_value="1.0",
            ),
            DeclareLaunchArgument(
                "launch_planner_maximum_vertical_acceleration_mps2",
                default_value="0.5",
            ),
            DeclareLaunchArgument("launch_planner_maximum_speed_mps", default_value="12.0"),
            DeclareLaunchArgument(
                "launch_planner_terminal_relative_speed_weight", default_value="0.0"
            ),
            DeclareLaunchArgument("launch_planner_terminal_relative_speed_scale_mps", default_value="4.0"),
            DeclareLaunchArgument("launch_planner_closing_speed_reserve_mps", default_value="4.0"),
            DeclareLaunchArgument("launch_planner_excess_speed_weight", default_value="0.2"),
            DeclareLaunchArgument("launch_planner_excess_speed_scale_mps", default_value="2.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(base)),
                launch_arguments={
                    "pursuer_prefix": LaunchConfiguration("pursuer_prefix"),
                    "search_topic": LaunchConfiguration("search_topic"),
                    "depth_topic": LaunchConfiguration("depth_topic"),
                    "pursuer_attitude_topic": LaunchConfiguration(
                        "pursuer_attitude_topic"
                    ),
                    "maximum_tilt_rad": LaunchConfiguration("maximum_tilt_rad"),
                    "launch_position_planner_enabled": LaunchConfiguration(
                        "launch_position_planner_enabled"
                    ),
                    "launch_position_planner_blend": LaunchConfiguration(
                        "launch_position_planner_blend"
                    ),
                    "launch_planner_minimum_geometry_blend": LaunchConfiguration(
                        "launch_planner_minimum_geometry_blend"
                    ),
                    "launch_planner_full_inbound_speed_mps": LaunchConfiguration(
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
                    "launch_planner_terminal_relative_speed_scale_mps": LaunchConfiguration("launch_planner_terminal_relative_speed_scale_mps"),
                    "launch_planner_closing_speed_reserve_mps": LaunchConfiguration("launch_planner_closing_speed_reserve_mps"),
                    "launch_planner_excess_speed_weight": LaunchConfiguration("launch_planner_excess_speed_weight"),
                    "launch_planner_excess_speed_scale_mps": LaunchConfiguration("launch_planner_excess_speed_scale_mps"),
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
                    "target_acceleration_feedforward_gain": LaunchConfiguration(
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
                    "handover_maximum_acceleration_mps2": LaunchConfiguration(
                        "handover_maximum_acceleration_mps2"
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
                    "recoil_duration_s": LaunchConfiguration(
                        "recoil_duration_s"
                    ),
                    "recoil_opening_speed_mps": LaunchConfiguration(
                        "recoil_opening_speed_mps"
                    ),
                    "reattack_cooldown_s": LaunchConfiguration(
                        "reattack_cooldown_s"
                    ),
                    "control_enabled": "true",
                    "auto_engage": "false",
                    "takeoff_enabled": "false",
                    "auto_disable_on_offboard_exit": "true",
                }.items(),
            ),
        ]
    )
