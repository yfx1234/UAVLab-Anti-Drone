"""双机仿真入口。

真值仅进入 simulation_truth_bridge 和 logical_camera_simulator。正式估计器与控制器
和 pure_interceptor_real.launch.py 使用完全相同的相机消息接口。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    target_prefix = LaunchConfiguration("target_prefix")
    pursuer_prefix = LaunchConfiguration("pursuer_prefix")
    pursuer_attitude_topic = LaunchConfiguration("pursuer_attitude_topic")
    pursuer_global_topic = LaunchConfiguration("pursuer_global_topic")
    pursuer_local_topic = LaunchConfiguration("pursuer_local_topic")
    target_state_topic = "/tracking/target/state"
    common_origin_topic = "/tracking/common_origin"
    truth_topic = "/evaluation/relative_truth"
    search_topic = "/perception/search/observation"
    depth_topic = "/perception/depth/observation"
    estimate_topic = "/tracking/relative/estimate"
    target_source = LaunchConfiguration("target_source")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rmw_implementation", default_value="rmw_cyclonedds_cpp"
            ),
            SetEnvironmentVariable(
                "RMW_IMPLEMENTATION", LaunchConfiguration("rmw_implementation")
            ),
            DeclareLaunchArgument("target_prefix", default_value="/fmu"),
            DeclareLaunchArgument("pursuer_prefix", default_value="/px4_1/fmu"),
            DeclareLaunchArgument("pursuer_target_system", default_value="2"),
            DeclareLaunchArgument(
                "target_source",
                default_value="dual_px4",
                description="dual_px4, virtual, or external_px4",
            ),
            DeclareLaunchArgument(
                "external_target_global_topic",
                default_value="/target/fmu/out/vehicle_global_position",
            ),
            DeclareLaunchArgument(
                "external_target_local_topic",
                default_value="/target/fmu/out/vehicle_local_position",
            ),
            DeclareLaunchArgument(
                "pursuer_attitude_topic",
                default_value="/px4_1/fmu/out/vehicle_attitude",
            ),
            DeclareLaunchArgument(
                "pursuer_global_topic",
                default_value="/px4_1/fmu/out/vehicle_global_position",
            ),
            DeclareLaunchArgument(
                "pursuer_local_topic",
                default_value="/px4_1/fmu/out/vehicle_local_position",
            ),
            DeclareLaunchArgument("target_scenario", default_value="straight"),
            DeclareLaunchArgument("target_speed_mps", default_value="1.2"),
            DeclareLaunchArgument("search_camera_rate_hz", default_value="30.0"),
            DeclareLaunchArgument("depth_camera_rate_hz", default_value="30.0"),
            DeclareLaunchArgument("target_heading_rad", default_value="0.0"),
            DeclareLaunchArgument("target_radius_m", default_value="20.0"),
            DeclareLaunchArgument("target_path_length_m", default_value="40.0"),
            DeclareLaunchArgument("target_path_width_m", default_value="12.0"),
            # Simulation orchestration only: the target begins its independent
            # trajectory before the pursuer controller is created. No target
            # state or readiness signal is connected to the production
            # controller.
            DeclareLaunchArgument("target_motion_delay_s", default_value="5.0"),
            DeclareLaunchArgument("target_motion_ramp_s", default_value="3.0"),
            DeclareLaunchArgument(
                "target_mission_ready_motion_s", default_value="0.2"
            ),
            DeclareLaunchArgument(
                "target_wait_for_pursuer_takeoff_ready",
                default_value="true",
                description=(
                    "true for synchronized full-auto SITL; false when the "
                    "target is independently operated before pursuer Offboard"
                ),
            ),
            DeclareLaunchArgument(
                "pursuer_controller_start_delay_s", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "pursuer_control_enabled", default_value="true"
            ),
            DeclareLaunchArgument("pursuer_auto_engage", default_value="true"),
            DeclareLaunchArgument(
                "pursuer_takeoff_enabled", default_value="true"
            ),
            DeclareLaunchArgument(
                "pursuer_auto_disable_on_offboard_exit", default_value="false"
            ),
            # The default geometry starts inside the ideal range without relying
            # on the retired fixed 30 m vertical corridor.
            DeclareLaunchArgument("target_height_m", default_value="30.0"),
            DeclareLaunchArgument(
                "virtual_target_initial_north_m", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "virtual_target_initial_east_m", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "virtual_target_spawn_relative_to_pursuer_on_ready",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "virtual_target_publish_before_pursuer_ready",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "virtual_target_use_pursuer_local_position",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "virtual_target_anchor_ready_topic",
                default_value="/simulation/pursuer_takeoff_ready",
                description=(
                    "Gate whose rising true sample latches a pursuer-relative "
                    "virtual-target anchor"
                ),
            ),
            DeclareLaunchArgument(
                "truth_pursuer_position_source", default_value="global"
            ),
            DeclareLaunchArgument("pursuer_takeoff_height_m", default_value="5.0"),
            DeclareLaunchArgument("minimum_flight_height_m", default_value="3.0"),
            DeclareLaunchArgument(
                "pursuer_hover_thrust",
                # Calibrated from steady hover of gz_my_quad_real/22001.
                default_value="0.42",
            ),
            DeclareLaunchArgument(
                "pursuer_maximum_thrust", default_value="0.9"
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
            DeclareLaunchArgument("attack_enable_delay_s", default_value="10.0"),
            DeclareLaunchArgument("estimate_stability_hold_s", default_value="1.0"),
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
            DeclareLaunchArgument("desired_closing_speed_mps", default_value="2.0"),
            DeclareLaunchArgument("radial_velocity_gain", default_value="0.8"),
            DeclareLaunchArgument("maximum_acceleration_mps2", default_value="5.0"),
            DeclareLaunchArgument("maximum_tilt_rad", default_value="0.52"),
            DeclareLaunchArgument(
                "depth_fov_constraint_rad",
                # Prefer an 8 deg camera-axis error while retaining a 2 deg
                # ballistic firing gate.
                default_value="0.13962634015954636",
            ),
            DeclareLaunchArgument(
                "depth_horizontal_half_fov_rad",
                # Simulation hard visibility boundary: 10 deg.
                default_value="0.17453292519943295",
            ),
            DeclareLaunchArgument(
                "depth_vertical_half_fov_rad",
                default_value="0.17453292519943295",
            ),
            DeclareLaunchArgument(
                "estimator_process_jerk_stddev_mps3", default_value="2.0"
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
                default_value="2.5",
            ),
            DeclareLaunchArgument(
                "launch_planner_maximum_vertical_acceleration_mps2",
                default_value="1.0",
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
                "handover_maximum_acceleration_mps2", default_value="2.5"
            ),
            DeclareLaunchArgument("fov_normal_boundary_rad", default_value="0.08726646259971647"),
            DeclareLaunchArgument("fov_centering_boundary_rad", default_value="0.13962634015954636"),
            DeclareLaunchArgument("fov_hard_boundary_rad", default_value="0.17453292519943295"),
            DeclareLaunchArgument("fov_recovery_exit_rad", default_value="0.10471975511965977"),
            DeclareLaunchArgument("fov_prediction_horizon_s", default_value="0.2"),
            DeclareLaunchArgument("fov_recovery_entry_hold_s", default_value="0.15"),
            DeclareLaunchArgument("fov_recovery_exit_hold_s", default_value="0.5"),
            DeclareLaunchArgument("tracking_invalid_hysteresis_s", default_value="0.2"),
            DeclareLaunchArgument("depth_loss_hysteresis_s", default_value="0.2"),
            DeclareLaunchArgument("egress_distance_m", default_value="5.0"),
            DeclareLaunchArgument("egress_speed_mps", default_value="1.0"),
            DeclareLaunchArgument("command_maximum_speed_mps", default_value="12.0"),
            DeclareLaunchArgument("command_speed_soft_zone_mps", default_value="2.5"),
            DeclareLaunchArgument("command_maximum_vertical_speed_mps", default_value="1.0"),
            DeclareLaunchArgument(
                "command_overspeed_braking_gain", default_value="4.0"
            ),
            DeclareLaunchArgument(
                "command_maximum_braking_acceleration_mps2", default_value="5.0"
            ),
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
                "known_target_search_acceleration_mps2", default_value="1.5"
            ),
            DeclareLaunchArgument(
                "known_target_search_acceleration_duration_s", default_value="1.5"
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
                executable="offboard_vehicle",
                namespace="target",
                name="offboard_vehicle",
                output="screen",
                condition=IfCondition(
                    PythonExpression(["'", target_source, "' == 'dual_px4'"])
                ),
                parameters=[
                    {
                        "role": "target",
                        "topic_prefix": target_prefix,
                        "target_system": 1,
                        "target_state_topic": target_state_topic,
                        "common_origin_topic": common_origin_topic,
                        "common_frame_id": "map_ned",
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
                        "takeoff_height_m": LaunchConfiguration("target_height_m"),
                        "target_motion_delay_s": LaunchConfiguration(
                            "target_motion_delay_s"
                        ),
                        "target_wait_for_pursuer_intercept": False,
                        "target_motion_ready_topic": "/simulation/target_motion_ready",
                        "target_wait_for_pursuer_takeoff_ready":
                            LaunchConfiguration(
                                "target_wait_for_pursuer_takeoff_ready"
                            ),
                        "pursuer_takeoff_ready_topic":
                            "/simulation/pursuer_takeoff_ready",
                        "target_motion_ramp_s": LaunchConfiguration(
                            "target_motion_ramp_s"
                        ),
                        "target_trajectory_feedforward_enabled": True,
                    }
                ],
            ),
            Node(
                package="px4_offboard_tracking",
                executable="target_state_source",
                namespace="simulation",
                name="target_state_source",
                output="screen",
                condition=IfCondition(
                    PythonExpression(["'", target_source, "' != 'dual_px4'"])
                ),
                parameters=[
                    {
                        "target_source": target_source,
                        "target_state_topic": target_state_topic,
                        "common_origin_topic": common_origin_topic,
                        "mission_ready_topic": "/simulation/target_motion_ready",
                        "pursuer_takeoff_ready_topic": LaunchConfiguration(
                            "virtual_target_anchor_ready_topic"
                        ),
                        "pursuer_global_topic": pursuer_global_topic,
                        "pursuer_local_topic": pursuer_local_topic,
                        "external_target_global_topic": LaunchConfiguration(
                            "external_target_global_topic"
                        ),
                        "external_target_local_topic": LaunchConfiguration(
                            "external_target_local_topic"
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
                        "virtual_initial_north_m": LaunchConfiguration(
                            "virtual_target_initial_north_m"
                        ),
                        "virtual_initial_east_m": LaunchConfiguration(
                            "virtual_target_initial_east_m"
                        ),
                        "virtual_spawn_relative_to_pursuer_on_ready":
                            LaunchConfiguration(
                                "virtual_target_spawn_relative_to_pursuer_on_ready"
                            ),
                        "virtual_publish_before_pursuer_ready":
                            LaunchConfiguration(
                                "virtual_target_publish_before_pursuer_ready"
                            ),
                        "virtual_use_pursuer_local_position":
                            LaunchConfiguration(
                                "virtual_target_use_pursuer_local_position"
                            ),
                        "target_motion_delay_s": LaunchConfiguration(
                            "target_motion_delay_s"
                        ),
                        "target_motion_ramp_s": LaunchConfiguration(
                            "target_motion_ramp_s"
                        ),
                        "mission_ready_motion_s": LaunchConfiguration(
                            "target_mission_ready_motion_s"
                        ),
                        "wait_for_pursuer_takeoff_ready": LaunchConfiguration(
                            "target_wait_for_pursuer_takeoff_ready"
                        ),
                    }
                ],
            ),
            Node(
                package="px4_offboard_tracking",
                executable="simulation_truth_bridge",
                namespace="simulation",
                name="simulation_truth_bridge",
                output="screen",
                parameters=[
                    {
                        "target_state_topic": target_state_topic,
                        "common_origin_topic": common_origin_topic,
                        "pursuer_global_topic": pursuer_global_topic,
                        "pursuer_local_topic": pursuer_local_topic,
                        "pursuer_position_source": LaunchConfiguration(
                            "truth_pursuer_position_source"
                        ),
                        "output_topic": truth_topic,
                    }
                ],
            ),
            Node(
                package="px4_offboard_tracking",
                executable="logical_camera_simulator",
                namespace="simulation",
                name="logical_search_camera_simulator",
                output="screen",
                parameters=[
                    {
                        "camera_output": "search",
                        "truth_topic": truth_topic,
                        "attitude_topic": pursuer_attitude_topic,
                        "search_topic": search_topic,
                        "camera_rate_hz": LaunchConfiguration(
                            "search_camera_rate_hz"
                        ),
                        "search_horizontal_half_fov_rad": 0.305432619,
                        "search_vertical_half_fov_rad": 0.523598776,
                    }
                ],
            ),
            Node(
                package="px4_offboard_tracking",
                executable="logical_camera_simulator",
                namespace="simulation",
                name="logical_depth_camera_simulator",
                output="screen",
                parameters=[
                    {
                        "camera_output": "depth",
                        "truth_topic": truth_topic,
                        "attitude_topic": pursuer_attitude_topic,
                        "depth_topic": depth_topic,
                        "camera_rate_hz": LaunchConfiguration(
                            "depth_camera_rate_hz"
                        ),
                        "depth_horizontal_half_fov_rad": LaunchConfiguration(
                            "depth_horizontal_half_fov_rad"
                        ),
                        "depth_vertical_half_fov_rad": LaunchConfiguration(
                            "depth_vertical_half_fov_rad"
                        ),
                    }
                ],
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
                        "attitude_topic": pursuer_attitude_topic,
                        "camera_fx": 12847.019977569425,
                        "camera_fy": 6124.984365187126,
                        "camera_cx": 1919.5,
                        "camera_cy": 1079.5,
                        "camera_width": 3840,
                        "camera_height": 2160,
                        "acceleration_noise_stddev_mps2": LaunchConfiguration(
                            "estimator_process_jerk_stddev_mps3"
                        ),
                    }
                ],
            ),
            Node(
                package="px4_offboard_tracking",
                executable="estimator_evaluator",
                namespace="evaluation",
                name="estimator_evaluator",
                output="screen",
                parameters=[
                    {
                        "truth_topic": truth_topic,
                        "estimate_topic": estimate_topic,
                        "report_period_s": 1.0,
                    }
                ],
            ),
            TimerAction(
                period=LaunchConfiguration("pursuer_controller_start_delay_s"),
                actions=[
                    Node(
                        package="px4_offboard_tracking",
                        executable="pure_interceptor_controller",
                        namespace="pursuer",
                        name="pure_interceptor_controller",
                        output="screen",
                        parameters=[
                            {
                                "topic_prefix": pursuer_prefix,
                                "target_system": LaunchConfiguration(
                                    "pursuer_target_system"
                                ),
                                "control_enabled": LaunchConfiguration(
                                    "pursuer_control_enabled"
                                ),
                                "auto_engage": LaunchConfiguration(
                                    "pursuer_auto_engage"
                                ),
                                "takeoff_enabled": LaunchConfiguration(
                                    "pursuer_takeoff_enabled"
                                ),
                                "auto_disable_on_offboard_exit":
                                    LaunchConfiguration(
                                        "pursuer_auto_disable_on_offboard_exit"
                                    ),
                                "search_topic": search_topic,
                                "depth_topic": depth_topic,
                                "estimate_topic": estimate_topic,
                                "takeoff_height_m": LaunchConfiguration(
                                    "pursuer_takeoff_height_m"
                                ),
                                "search_pattern_enabled": LaunchConfiguration(
                                    "search_pattern_enabled"
                                ),
                                "search_yaw_rate_rad_s": LaunchConfiguration(
                                    "search_yaw_rate_rad_s"
                                ),
                                "search_prediction_hold_s": LaunchConfiguration(
                                    "search_prediction_hold_s"
                                ),
                                "known_target_direction_enabled":
                                    LaunchConfiguration(
                                        "known_target_direction_enabled"
                                    ),
                                "known_target_bearing_ned_rad":
                                    LaunchConfiguration(
                                        "known_target_bearing_ned_rad"
                                    ),
                                "known_target_search_hold_s":
                                    LaunchConfiguration(
                                        "known_target_search_hold_s"
                                    ),
                                "known_target_search_acceleration_mps2":
                                    LaunchConfiguration(
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
                                "known_target_search_velocity_mps":
                                    LaunchConfiguration(
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
                                "search_repeat_until_acquired":
                                    LaunchConfiguration(
                                        "search_repeat_until_acquired"
                                    ),
                                "minimum_flight_height_m": LaunchConfiguration(
                                    "minimum_flight_height_m"
                                ),
                                "hover_thrust": LaunchConfiguration(
                                    "pursuer_hover_thrust"
                                ),
                                "maximum_thrust": LaunchConfiguration(
                                    "pursuer_maximum_thrust"
                                ),
                                "attitude_gain": LaunchConfiguration(
                                    "attitude_gain"
                                ),
                                "maximum_body_rate_rad_s": LaunchConfiguration(
                                    "maximum_body_rate_rad_s"
                                ),
                                "transverse_velocity_gain":
                                    LaunchConfiguration(
                                        "transverse_velocity_gain"
                                    ),
                                "attack_enable_delay_s": LaunchConfiguration(
                                    "attack_enable_delay_s"
                                ),
                                "estimate_stability_hold_s": LaunchConfiguration(
                                    "estimate_stability_hold_s"
                                ),
                                "first_attack_tracking_hold_s":
                                    LaunchConfiguration(
                                        "first_attack_tracking_hold_s"
                                    ),
                                "terminal_alignment_tracking_hold_s":
                                    LaunchConfiguration(
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
                                "shot_topic": "/tracking/virtual_shot_event",
                                "projectile_speed_mps": 400.0,
                                "shot_maximum_time_of_flight_s": 0.25,
                                "shot_maximum_lead_angle_rad":
                                    0.06981317007977318,
                                "shot_aim_tolerance_rad":
                                    0.017453292519943295,
                                "shot_maximum_lateral_sigma_m": 0.75,
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
                                "desired_closing_speed_mps": LaunchConfiguration(
                                    "desired_closing_speed_mps"
                                ),
                                "radial_velocity_gain": LaunchConfiguration(
                                    "radial_velocity_gain"
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
                                "launch_position_planner_enabled":
                                    LaunchConfiguration(
                                        "launch_position_planner_enabled"
                                    ),
                                "launch_position_planner_blend":
                                    LaunchConfiguration(
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
                                "launch_planner_minimum_horizon_s":
                                    LaunchConfiguration(
                                        "launch_planner_minimum_horizon_s"
                                    ),
                                "launch_planner_maximum_horizon_s":
                                    LaunchConfiguration(
                                        "launch_planner_maximum_horizon_s"
                                    ),
                                "launch_planner_preferred_range_m":
                                    LaunchConfiguration(
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
                                "launch_planner_maximum_speed_mps":
                                    LaunchConfiguration(
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
                                "launch_planner_excess_speed_weight":
                                    LaunchConfiguration(
                                        "launch_planner_excess_speed_weight"
                                    ),
                                "launch_planner_excess_speed_scale_mps":
                                    LaunchConfiguration(
                                        "launch_planner_excess_speed_scale_mps"
                                    ),
                                "capture_maximum_approach_speed_mps":
                                    LaunchConfiguration(
                                        "capture_maximum_approach_speed_mps"
                                    ),
                                "handover_maximum_acceleration_mps2":
                                    LaunchConfiguration(
                                        "handover_maximum_acceleration_mps2"
                                    ),
                                "fov_normal_boundary_rad":
                                    LaunchConfiguration(
                                        "fov_normal_boundary_rad"
                                    ),
                                "fov_centering_boundary_rad":
                                    LaunchConfiguration(
                                        "fov_centering_boundary_rad"
                                    ),
                                "fov_hard_boundary_rad":
                                    LaunchConfiguration(
                                        "fov_hard_boundary_rad"
                                    ),
                                "fov_recovery_exit_rad":
                                    LaunchConfiguration(
                                        "fov_recovery_exit_rad"
                                    ),
                                "fov_prediction_horizon_s":
                                    LaunchConfiguration(
                                        "fov_prediction_horizon_s"
                                    ),
                                "fov_recovery_entry_hold_s":
                                    LaunchConfiguration(
                                        "fov_recovery_entry_hold_s"
                                    ),
                                "fov_recovery_exit_hold_s":
                                    LaunchConfiguration(
                                        "fov_recovery_exit_hold_s"
                                    ),
                                "tracking_invalid_hysteresis_s":
                                    LaunchConfiguration(
                                        "tracking_invalid_hysteresis_s"
                                    ),
                                "depth_loss_hysteresis_s": LaunchConfiguration(
                                    "depth_loss_hysteresis_s"
                                ),
                                "maximum_acceleration_mps2": LaunchConfiguration(
                                    "maximum_acceleration_mps2"
                                ),
                                "maximum_tilt_rad": LaunchConfiguration(
                                    "maximum_tilt_rad"
                                ),
                                "handover_maximum_tilt_rad": LaunchConfiguration(
                                    "maximum_tilt_rad"
                                ),
                                "depth_fov_constraint_rad": LaunchConfiguration(
                                    "depth_fov_constraint_rad"
                                ),
                                "egress_distance_m": LaunchConfiguration(
                                    "egress_distance_m"
                                ),
                                "egress_speed_mps": LaunchConfiguration(
                                    "egress_speed_mps"
                                ),
                                "command_maximum_speed_mps": LaunchConfiguration(
                                    "command_maximum_speed_mps"
                                ),
                                "command_speed_soft_zone_mps": LaunchConfiguration(
                                    "command_speed_soft_zone_mps"
                                ),
                                "command_maximum_vertical_speed_mps":
                                    LaunchConfiguration(
                                        "command_maximum_vertical_speed_mps"
                                    ),
                                "command_overspeed_braking_gain":
                                    LaunchConfiguration(
                                        "command_overspeed_braking_gain"
                                    ),
                                "command_maximum_braking_acceleration_mps2":
                                    LaunchConfiguration(
                                        "command_maximum_braking_acceleration_mps2"
                                    ),
                                "safety_maximum_speed_mps": LaunchConfiguration(
                                    "safety_maximum_speed_mps"
                                ),
                                "completion_braking_hold_s": LaunchConfiguration(
                                    "completion_braking_hold_s"
                                ),
                                "completion_position_hold_maximum_speed_mps":
                                    LaunchConfiguration(
                                        "completion_position_hold_maximum_speed_mps"
                                    ),
                                "completion_position_hold_settle_s":
                                    LaunchConfiguration(
                                        "completion_position_hold_settle_s"
                                    ),
                                "completion_safe_hover_maximum_speed_mps":
                                    LaunchConfiguration(
                                        "completion_safe_hover_maximum_speed_mps"
                                    ),
                                "completion_safe_hover_hold_s": LaunchConfiguration(
                                    "completion_safe_hover_hold_s"
                                ),
                                "minimum_height_prediction_horizon_s":
                                    LaunchConfiguration(
                                        "minimum_height_prediction_horizon_s"
                                    ),
                                "minimum_height_recovery_m": LaunchConfiguration(
                                    "minimum_height_recovery_m"
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
                                "require_mission_enable": True,
                                "mission_enable_topic":
                                    "/simulation/target_motion_ready",
                                "pursuer_takeoff_ready_topic":
                                    "/simulation/pursuer_takeoff_ready",
                            }
                        ],
                    )
                ],
            ),
        ]
    )
