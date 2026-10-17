"""单拦截机坐标驱动相机测试：人工起飞/切入Offboard入口。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    base = (
        Path(get_package_share_directory("px4_offboard_tracking"))
        / "launch"
        / "pure_interceptor_sim.launch.py"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rmw_implementation", default_value="rmw_cyclonedds_cpp"
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
                "target_source", default_value="external_px4"
            ),
            DeclareLaunchArgument("target_scenario", default_value="straight"),
            DeclareLaunchArgument("target_speed_mps", default_value="2.0"),
            DeclareLaunchArgument(
                "target_heading_rad", default_value="3.141592653589793"
            ),
            DeclareLaunchArgument("target_radius_m", default_value="20.0"),
            DeclareLaunchArgument("target_path_length_m", default_value="40.0"),
            DeclareLaunchArgument("target_path_width_m", default_value="12.0"),
            DeclareLaunchArgument("target_height_m", default_value="25.0"),
            DeclareLaunchArgument(
                "virtual_target_initial_north_m", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "virtual_target_initial_east_m", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "virtual_target_spawn_relative_to_pursuer_on_ready",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "virtual_target_publish_before_pursuer_ready",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "virtual_target_use_pursuer_local_position",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "virtual_target_anchor_ready_topic",
                default_value="/simulation/pursuer_takeoff_ready",
            ),
            DeclareLaunchArgument(
                "truth_pursuer_position_source", default_value="local"
            ),
            DeclareLaunchArgument("target_motion_delay_s", default_value="0.0"),
            DeclareLaunchArgument("target_motion_ramp_s", default_value="0.0"),
            DeclareLaunchArgument(
                "target_mission_ready_motion_s", default_value="0.0"
            ),
            DeclareLaunchArgument("search_camera_rate_hz", default_value="30.0"),
            DeclareLaunchArgument("depth_camera_rate_hz", default_value="20.0"),
            DeclareLaunchArgument(
                "pursuer_hover_thrust", default_value="0.42"
            ),
            DeclareLaunchArgument(
                "pursuer_maximum_thrust", default_value="0.9"
            ),
            DeclareLaunchArgument(
                "maximum_acceleration_mps2", default_value="5.0"
            ),
            DeclareLaunchArgument("maximum_tilt_rad", default_value="0.52"),
            DeclareLaunchArgument(
                "desired_closing_speed_mps", default_value="2.0"
            ),
            DeclareLaunchArgument(
                "minimum_flight_height_m", default_value="3.0"
            ),
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
            DeclareLaunchArgument(
                "shot_timing_prediction_horizon_s", default_value="0.04"
            ),
            DeclareLaunchArgument(
                "shot_timing_minimum_ready_hold_s", default_value="0.04"
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
            DeclareLaunchArgument(
                "command_maximum_speed_mps", default_value="12.0"
            ),
            DeclareLaunchArgument(
                "safety_maximum_speed_mps", default_value="14.0"
            ),
            DeclareLaunchArgument("attitude_gain", default_value="3.0"),
            DeclareLaunchArgument(
                "maximum_body_rate_rad_s", default_value="3.0"
            ),
            DeclareLaunchArgument(
                "transverse_velocity_gain", default_value="1.0"
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
                default_value="2.5",
            ),
            DeclareLaunchArgument(
                "launch_planner_maximum_vertical_acceleration_mps2",
                default_value="1.0",
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
                "external_target_global_topic",
                default_value="/target/fmu/out/vehicle_global_position",
            ),
            DeclareLaunchArgument(
                "external_target_local_topic",
                default_value="/target/fmu/out/vehicle_local_position",
            ),
            DeclareLaunchArgument(
                "pursuer_control_enabled", default_value="true"
            ),
            DeclareLaunchArgument(
                "pursuer_auto_engage", default_value="false"
            ),
            DeclareLaunchArgument(
                "pursuer_takeoff_enabled", default_value="false"
            ),
            DeclareLaunchArgument(
                "pursuer_auto_disable_on_offboard_exit", default_value="true"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(base)),
                launch_arguments={
                    "rmw_implementation": LaunchConfiguration(
                        "rmw_implementation"
                    ),
                    "target_source": LaunchConfiguration("target_source"),
                    "pursuer_prefix": LaunchConfiguration("pursuer_prefix"),
                    "pursuer_target_system": "1",
                    "pursuer_attitude_topic": LaunchConfiguration(
                        "pursuer_attitude_topic"
                    ),
                    "pursuer_global_topic": LaunchConfiguration(
                        "pursuer_global_topic"
                    ),
                    "pursuer_local_topic": LaunchConfiguration(
                        "pursuer_local_topic"
                    ),
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
                    "target_height_m": LaunchConfiguration("target_height_m"),
                    "virtual_target_initial_north_m": LaunchConfiguration(
                        "virtual_target_initial_north_m"
                    ),
                    "virtual_target_initial_east_m": LaunchConfiguration(
                        "virtual_target_initial_east_m"
                    ),
                    "virtual_target_spawn_relative_to_pursuer_on_ready":
                        LaunchConfiguration(
                            "virtual_target_spawn_relative_to_pursuer_on_ready"
                        ),
                    "virtual_target_publish_before_pursuer_ready":
                        LaunchConfiguration(
                            "virtual_target_publish_before_pursuer_ready"
                        ),
                    "virtual_target_use_pursuer_local_position":
                        LaunchConfiguration(
                            "virtual_target_use_pursuer_local_position"
                        ),
                    "virtual_target_anchor_ready_topic": LaunchConfiguration(
                        "virtual_target_anchor_ready_topic"
                    ),
                    "truth_pursuer_position_source": LaunchConfiguration(
                        "truth_pursuer_position_source"
                    ),
                    "target_motion_delay_s": LaunchConfiguration(
                        "target_motion_delay_s"
                    ),
                    "target_motion_ramp_s": LaunchConfiguration(
                        "target_motion_ramp_s"
                    ),
                    "target_mission_ready_motion_s": LaunchConfiguration(
                        "target_mission_ready_motion_s"
                    ),
                    "search_camera_rate_hz": LaunchConfiguration(
                        "search_camera_rate_hz"
                    ),
                    "depth_camera_rate_hz": LaunchConfiguration(
                        "depth_camera_rate_hz"
                    ),
                    "pursuer_hover_thrust": LaunchConfiguration(
                        "pursuer_hover_thrust"
                    ),
                    "pursuer_maximum_thrust": LaunchConfiguration(
                        "pursuer_maximum_thrust"
                    ),
                    "maximum_acceleration_mps2": LaunchConfiguration(
                        "maximum_acceleration_mps2"
                    ),
                    "maximum_tilt_rad": LaunchConfiguration(
                        "maximum_tilt_rad"
                    ),
                    "desired_closing_speed_mps": LaunchConfiguration(
                        "desired_closing_speed_mps"
                    ),
                    "minimum_flight_height_m": LaunchConfiguration(
                        "minimum_flight_height_m"
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
                    "shot_timing_prediction_horizon_s": LaunchConfiguration(
                        "shot_timing_prediction_horizon_s"
                    ),
                    "shot_timing_minimum_ready_hold_s": LaunchConfiguration(
                        "shot_timing_minimum_ready_hold_s"
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
                    "command_maximum_speed_mps": LaunchConfiguration(
                        "command_maximum_speed_mps"
                    ),
                    "safety_maximum_speed_mps": LaunchConfiguration(
                        "safety_maximum_speed_mps"
                    ),
                    "attitude_gain": LaunchConfiguration("attitude_gain"),
                    "maximum_body_rate_rad_s": LaunchConfiguration(
                        "maximum_body_rate_rad_s"
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
                    "external_target_global_topic": LaunchConfiguration(
                        "external_target_global_topic"
                    ),
                    "external_target_local_topic": LaunchConfiguration(
                        "external_target_local_topic"
                    ),
                    "pursuer_control_enabled": LaunchConfiguration(
                        "pursuer_control_enabled"
                    ),
                    "pursuer_auto_engage": LaunchConfiguration(
                        "pursuer_auto_engage"
                    ),
                    "pursuer_takeoff_enabled": LaunchConfiguration(
                        "pursuer_takeoff_enabled"
                    ),
                    "pursuer_auto_disable_on_offboard_exit":
                        LaunchConfiguration(
                            "pursuer_auto_disable_on_offboard_exit"
                        ),
                }.items(),
            ),
            Node(
                package="virtual_interception_referee",
                executable="virtual_projectile_referee",
                namespace="evaluation",
                name="virtual_projectile_referee",
                output="screen",
                parameters=[
                    {
                        "shot_topic": "/tracking/virtual_shot_event",
                        "result_topic": "/evaluation/virtual_hit_result",
                        "solver_result_topic":
                            "/evaluation/virtual_solver_hit_result",
                        "truth_topic": "/evaluation/relative_truth",
                        "task_origin_topic": "/tracking/task_origin_ned",
                        "pursuer_local_topic": LaunchConfiguration(
                            "pursuer_local_topic"
                        ),
                        "pursuer_attitude_topic": LaunchConfiguration(
                            "pursuer_attitude_topic"
                        ),
                        "projectile_speed_mps": 400.0,
                        "hit_radius_m": 0.4,
                        "effect_reference_range_m": 35.0,
                        "effect_reference_diameter_m": 1.7,
                    }
                ],
            ),
        ]
    )
