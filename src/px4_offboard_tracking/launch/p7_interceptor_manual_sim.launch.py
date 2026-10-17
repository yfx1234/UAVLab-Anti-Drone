"""P7速度档完整跟踪/打击的人工起飞仿真入口。"""

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
        / "p7_interceptor_sim.launch.py"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("profile", default_value="v0"),
            DeclareLaunchArgument("encounter", default_value="escape"),
            DeclareLaunchArgument("search_camera_rate_hz", default_value="30.0"),
            DeclareLaunchArgument("depth_camera_rate_hz", default_value="30.0"),
            DeclareLaunchArgument(
                "handover_maximum_acceleration_mps2", default_value="2.5"
            ),
            DeclareLaunchArgument("attack_enable_delay_s", default_value="10.0"),
            DeclareLaunchArgument(
                "first_attack_tracking_hold_s", default_value="3.0"
            ),
            DeclareLaunchArgument(
                "terminal_alignment_tracking_hold_s", default_value="2.0"
            ),
            DeclareLaunchArgument("terminal_alignment_ramp_s", default_value="1.0"),
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
            DeclareLaunchArgument("recoil_duration_s", default_value="0.2"),
            DeclareLaunchArgument(
                "recoil_opening_speed_mps", default_value="1.0"
            ),
            DeclareLaunchArgument("reattack_cooldown_s", default_value="3.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(base)),
                launch_arguments={
                    "profile": LaunchConfiguration("profile"),
                    "encounter": LaunchConfiguration("encounter"),
                    "search_camera_rate_hz": LaunchConfiguration(
                        "search_camera_rate_hz"
                    ),
                    "depth_camera_rate_hz": LaunchConfiguration(
                        "depth_camera_rate_hz"
                    ),
                    "handover_maximum_acceleration_mps2": LaunchConfiguration(
                        "handover_maximum_acceleration_mps2"
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
