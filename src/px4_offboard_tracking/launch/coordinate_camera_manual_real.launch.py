"""实物拦截机与虚拟坐标靶的人工Offboard测试入口。

本入口只控制/fmu下的实物拦截机。虚拟靶机在遥控器切入Offboard且飞机状态稳定后，
相对拦截机当前位置生成；目标坐标只进入逻辑搜索/双目相机，不进入正式控制器。
这是排除物理相机影响的试验入口，不是正式视觉验收入口。
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
        / "coordinate_camera_manual.launch.py"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rmw_implementation", default_value="rmw_fastrtps_cpp"
            ),
            DeclareLaunchArgument("pursuer_prefix", default_value="/fmu"),
            DeclareLaunchArgument(
                "pursuer_attitude_topic",
                default_value="/fmu/out/vehicle_attitude",
            ),
            DeclareLaunchArgument(
                "pursuer_global_topic",
                default_value="/fmu/out/vehicle_global_position",
            ),
            DeclareLaunchArgument(
                "pursuer_local_topic",
                default_value="/fmu/out/vehicle_local_position",
            ),
            DeclareLaunchArgument(
                "hover_thrust",
                description=(
                    "Measured hover thrust for the 2.76 kg T6143 airframe "
                    "with the 2200 mAh battery configuration"
                ),
                default_value="0.42",
            ),
            DeclareLaunchArgument(
                "maximum_thrust",
                description=(
                    "Conservative first-sortie normalized thrust limit for the "
                    "2.76 kg T6143 airframe"
                ),
                default_value="0.70",
            ),
            DeclareLaunchArgument("target_scenario", default_value="straight"),
            DeclareLaunchArgument("target_speed_mps", default_value="2.0"),
            DeclareLaunchArgument(
                "target_heading_rad", default_value="3.141592653589793"
            ),
            DeclareLaunchArgument("target_radius_m", default_value="20.0"),
            DeclareLaunchArgument("target_path_length_m", default_value="40.0"),
            DeclareLaunchArgument("target_path_width_m", default_value="12.0"),
            DeclareLaunchArgument(
                "virtual_target_initial_north_m", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "virtual_target_initial_east_m", default_value="0.0"
            ),
            DeclareLaunchArgument("target_motion_delay_s", default_value="0.0"),
            DeclareLaunchArgument("target_motion_ramp_s", default_value="0.0"),
            DeclareLaunchArgument("search_camera_rate_hz", default_value="30.0"),
            DeclareLaunchArgument("depth_camera_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("search_pattern_enabled", default_value="true"),
            DeclareLaunchArgument("search_yaw_rate_rad_s", default_value="0.35"),
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
                "search_repeat_until_acquired", default_value="true"
            ),
            DeclareLaunchArgument("minimum_flight_height_m", default_value="3.0"),
            DeclareLaunchArgument("maximum_tilt_rad", default_value="0.35"),
            DeclareLaunchArgument(
                "maximum_acceleration_mps2", default_value="1.0"
            ),
            DeclareLaunchArgument(
                "desired_closing_speed_mps", default_value="1.0"
            ),
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
            DeclareLaunchArgument(
                "command_maximum_speed_mps", default_value="12.0"
            ),
            DeclareLaunchArgument(
                "safety_maximum_speed_mps", default_value="14.0"
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(base)),
                launch_arguments={
                    "rmw_implementation": LaunchConfiguration(
                        "rmw_implementation"
                    ),
                    "pursuer_prefix": LaunchConfiguration("pursuer_prefix"),
                    "pursuer_attitude_topic": LaunchConfiguration(
                        "pursuer_attitude_topic"
                    ),
                    "pursuer_global_topic": LaunchConfiguration(
                        "pursuer_global_topic"
                    ),
                    "pursuer_local_topic": LaunchConfiguration(
                        "pursuer_local_topic"
                    ),
                    "target_source": "virtual",
                    "target_scenario": LaunchConfiguration("target_scenario"),
                    "target_speed_mps": LaunchConfiguration("target_speed_mps"),
                    "target_heading_rad": LaunchConfiguration(
                        "target_heading_rad"
                    ),
                    "target_radius_m": LaunchConfiguration("target_radius_m"),
                    "target_path_length_m": LaunchConfiguration(
                        "target_path_length_m"
                    ),
                    "target_path_width_m": LaunchConfiguration(
                        "target_path_width_m"
                    ),
                    "target_height_m": "25.0",
                    "virtual_target_initial_north_m": LaunchConfiguration(
                        "virtual_target_initial_north_m"
                    ),
                    "virtual_target_initial_east_m": LaunchConfiguration(
                        "virtual_target_initial_east_m"
                    ),
                    "virtual_target_spawn_relative_to_pursuer_on_ready": "true",
                    "virtual_target_publish_before_pursuer_ready": "false",
                    "virtual_target_use_pursuer_local_position": "true",
                    "virtual_target_anchor_ready_topic":
                        "/tracking/offboard_session_started",
                    "truth_pursuer_position_source": "local",
                    "target_motion_delay_s": LaunchConfiguration(
                        "target_motion_delay_s"
                    ),
                    "target_motion_ramp_s": LaunchConfiguration(
                        "target_motion_ramp_s"
                    ),
                    "search_camera_rate_hz": LaunchConfiguration(
                        "search_camera_rate_hz"
                    ),
                    "depth_camera_rate_hz": LaunchConfiguration(
                        "depth_camera_rate_hz"
                    ),
                    "search_pattern_enabled": LaunchConfiguration(
                        "search_pattern_enabled"
                    ),
                    "search_yaw_rate_rad_s": LaunchConfiguration(
                        "search_yaw_rate_rad_s"
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
                    "pursuer_hover_thrust": LaunchConfiguration("hover_thrust"),
                    "pursuer_maximum_thrust": LaunchConfiguration(
                        "maximum_thrust"
                    ),
                    "maximum_tilt_rad": LaunchConfiguration("maximum_tilt_rad"),
                    "maximum_acceleration_mps2": LaunchConfiguration(
                        "maximum_acceleration_mps2"
                    ),
                    "desired_closing_speed_mps": LaunchConfiguration(
                        "desired_closing_speed_mps"
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
                    "command_maximum_speed_mps": LaunchConfiguration(
                        "command_maximum_speed_mps"
                    ),
                    "safety_maximum_speed_mps": LaunchConfiguration(
                        "safety_maximum_speed_mps"
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
                    "pursuer_control_enabled": "true",
                    "pursuer_auto_engage": "false",
                    "pursuer_takeoff_enabled": "false",
                    "pursuer_auto_disable_on_offboard_exit": "true",
                }.items(),
            ),
        ]
    )
