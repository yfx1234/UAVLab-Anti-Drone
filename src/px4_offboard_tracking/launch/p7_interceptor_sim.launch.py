"""P7 速度档位与相遇场景入口。

该文件只解析已审核的速度/距离配置，然后复用正式
pure_interceptor_sim.launch.py 控制链。PX4/Gazebo 的两机生成仍由外层
启动器负责，终端会打印与本档位匹配的生成坐标。
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def load_case(config_path: Path, profile_name: str, encounter_name: str) -> dict:
    with config_path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    profiles = document["profiles"]
    encounters = document["encounters"]
    if profile_name not in profiles:
        raise ValueError(f"未知 P7 速度档位: {profile_name}")
    profile = profiles[profile_name]
    if encounter_name not in profile["scenarios"]:
        raise ValueError(
            f"{profile_name} 未审核场景 {encounter_name}；"
            f"允许值为 {profile['scenarios']}"
        )
    encounter = encounters[encounter_name]
    window = document["capture_window"]
    target_speed = (
        float(profile["target_speed_mps"])
        * float(encounter["target_speed_scale"])
    )
    return {
        **window,
        **profile,
        **encounter,
        "profile_name": profile_name,
        "encounter_name": encounter_name,
        "target_scenario": encounter["target_scenario"],
        "target_heading_rad": float(encounter["target_heading_rad"]),
        "resolved_target_speed_mps": target_speed,
    }


def _include_profile(context):
    package_share = Path(get_package_share_directory("px4_offboard_tracking"))
    case = load_case(
        package_share / "config" / "high_speed_profiles.yaml",
        LaunchConfiguration("profile").perform(context).lower(),
        LaunchConfiguration("encounter").perform(context).lower(),
    )
    arguments = {
        "target_scenario": str(case["target_scenario"]),
        "target_speed_mps": str(case["resolved_target_speed_mps"]),
        "target_heading_rad": str(case["target_heading_rad"]),
        "target_radius_m": str(case.get("target_radius_m", 20.0)),
        "target_path_length_m": str(case.get("target_path_length_m", 40.0)),
        "target_path_width_m": str(case.get("target_path_width_m", 12.0)),
        "search_camera_rate_hz": LaunchConfiguration(
            "search_camera_rate_hz"
        ).perform(context),
        "depth_camera_rate_hz": LaunchConfiguration(
            "depth_camera_rate_hz"
        ).perform(context),
        # The target completes takeoff, its 5 s motion delay and 3 s speed ramp
        # before the pursuer controller is launched. This is simulation
        # sequencing, not a truth input to guidance or capture logic.
        "target_motion_delay_s": "5.0",
        "pursuer_controller_start_delay_s": "0.0",
        "target_height_m": str(case["target_height_m"]),
        "pursuer_takeoff_height_m": str(case["pursuer_takeoff_height_m"]),
        "first_attack_tracking_hold_s": LaunchConfiguration(
            "first_attack_tracking_hold_s"
        ).perform(context),
        "attack_enable_delay_s": LaunchConfiguration(
            "attack_enable_delay_s"
        ).perform(context),
        "terminal_alignment_tracking_hold_s": LaunchConfiguration(
            "terminal_alignment_tracking_hold_s"
        ).perform(context),
        "terminal_alignment_ramp_s": LaunchConfiguration(
            "terminal_alignment_ramp_s"
        ).perform(context),
        "maximum_acceleration_mps2": str(case["maximum_acceleration_mps2"]),
        "handover_maximum_acceleration_mps2": LaunchConfiguration(
            "handover_maximum_acceleration_mps2"
        ).perform(context),
        "transverse_velocity_gain": LaunchConfiguration(
            "transverse_velocity_gain"
        ).perform(context),
        "target_acceleration_feedforward_gain": LaunchConfiguration(
            "target_acceleration_feedforward_gain"
        ).perform(context),
        "target_acceleration_feedforward_minimum_ca_probability":
            LaunchConfiguration(
                "target_acceleration_feedforward_minimum_ca_probability"
            ).perform(context),
        "target_acceleration_feedforward_full_ca_probability":
            LaunchConfiguration(
                "target_acceleration_feedforward_full_ca_probability"
            ).perform(context),
        "target_acceleration_feedforward_maximum_stddev_mps2":
            LaunchConfiguration(
                "target_acceleration_feedforward_maximum_stddev_mps2"
            ).perform(context),
        "target_acceleration_feedforward_filter_time_constant_s":
            LaunchConfiguration(
                "target_acceleration_feedforward_filter_time_constant_s"
            ).perform(context),
        "target_acceleration_feedforward_maximum_mps2": LaunchConfiguration(
            "target_acceleration_feedforward_maximum_mps2"
        ).perform(context),
        "launch_position_planner_enabled": LaunchConfiguration(
            "launch_position_planner_enabled"
        ).perform(context),
        "launch_position_planner_blend": LaunchConfiguration(
            "launch_position_planner_blend"
        ).perform(context),
        "launch_planner_minimum_geometry_blend": LaunchConfiguration(
            "launch_planner_minimum_geometry_blend"
        ).perform(context),
        "launch_planner_full_inbound_speed_mps": LaunchConfiguration(
            "launch_planner_full_inbound_speed_mps"
        ).perform(context),
        "launch_planner_minimum_horizon_s": LaunchConfiguration(
            "launch_planner_minimum_horizon_s"
        ).perform(context),
        "launch_planner_maximum_horizon_s": LaunchConfiguration(
            "launch_planner_maximum_horizon_s"
        ).perform(context),
        "launch_planner_preferred_range_m": LaunchConfiguration(
            "launch_planner_preferred_range_m"
        ).perform(context),
        "launch_planner_maximum_horizontal_acceleration_mps2": LaunchConfiguration(
            "launch_planner_maximum_horizontal_acceleration_mps2"
        ).perform(context),
        "launch_planner_maximum_vertical_acceleration_mps2": LaunchConfiguration(
            "launch_planner_maximum_vertical_acceleration_mps2"
        ).perform(context),
        "launch_planner_maximum_speed_mps": LaunchConfiguration(
            "launch_planner_maximum_speed_mps"
        ).perform(context),
        "launch_planner_terminal_relative_speed_weight": LaunchConfiguration(
            "launch_planner_terminal_relative_speed_weight"
        ).perform(context),
        "launch_planner_terminal_relative_speed_scale_mps": LaunchConfiguration(
            "launch_planner_terminal_relative_speed_scale_mps"
        ).perform(context),
        "launch_planner_closing_speed_reserve_mps": LaunchConfiguration(
            "launch_planner_closing_speed_reserve_mps"
        ).perform(context),
        "launch_planner_excess_speed_weight": LaunchConfiguration(
            "launch_planner_excess_speed_weight"
        ).perform(context),
        "launch_planner_excess_speed_scale_mps": LaunchConfiguration(
            "launch_planner_excess_speed_scale_mps"
        ).perform(context),
        "maximum_tilt_rad": str(case["maximum_tilt_rad"]),
        "depth_fov_constraint_rad": str(
            case.get("depth_fov_constraint_rad", 0.08726646259971647)
        ),
        "estimator_process_jerk_stddev_mps3": str(
            case.get("estimator_process_jerk_stddev_mps3", 8.0)
        ),
        "capture_minimum_range_m": str(case["minimum_range_m"]),
        "capture_maximum_range_m": str(case["maximum_range_m"]),
        "preferred_capture_range_m": str(case["preferred_range_m"]),
        "capture_maximum_approach_speed_mps": str(
            case["capture_maximum_approach_speed_mps"]
        ),
        "recoil_duration_s": LaunchConfiguration(
            "recoil_duration_s"
        ).perform(context),
        "recoil_opening_speed_mps": LaunchConfiguration(
            "recoil_opening_speed_mps"
        ).perform(context),
        "reattack_cooldown_s": LaunchConfiguration(
            "reattack_cooldown_s"
        ).perform(context),
        "egress_distance_m": str(case["egress_distance_m"]),
        "egress_speed_mps": str(case["egress_speed_mps"]),
        "command_maximum_speed_mps": str(case["pursuer_command_speed_mps"]),
        "command_speed_soft_zone_mps": str(
            min(2.5, 0.25 * float(case["pursuer_command_speed_mps"]))
        ),
        "command_overspeed_braking_gain": str(
            case.get("command_overspeed_braking_gain", 4.0)
        ),
        "command_maximum_braking_acceleration_mps2": str(
            case.get("command_maximum_braking_acceleration_mps2", 5.0)
        ),
        "safety_maximum_speed_mps": str(case["pursuer_safety_speed_mps"]),
        "pursuer_control_enabled": LaunchConfiguration(
            "pursuer_control_enabled"
        ).perform(context),
        "pursuer_auto_engage": LaunchConfiguration(
            "pursuer_auto_engage"
        ).perform(context),
        "pursuer_takeoff_enabled": LaunchConfiguration(
            "pursuer_takeoff_enabled"
        ).perform(context),
        "pursuer_auto_disable_on_offboard_exit": LaunchConfiguration(
            "pursuer_auto_disable_on_offboard_exit"
        ).perform(context),
    }
    initial_y = -float(case["initial_horizontal_offset_m"])
    geometry = (
        f"P7 {case['profile_name']}/{case['encounter_name']}: "
        f"目标 {case['resolved_target_speed_mps']:.1f} m/s，"
        f"追踪命令/安全限速 "
        f"{case['pursuer_command_speed_mps']:.1f}/"
        f"{case['pursuer_safety_speed_mps']:.1f} m/s，"
        f"打击距离 {case['minimum_range_m']:.1f}~"
        f"{case['maximum_range_m']:.1f} m；"
        f"建议生成 TARGET_POSE=0,0,0.02,0,0,1.570796，"
        f"PURSUER_POSE=0,{initial_y:.2f},0.02,0,0,1.570796"
    )
    return [
        LogInfo(msg=geometry),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(package_share / "launch" / "pure_interceptor_sim.launch.py")
            ),
            launch_arguments=arguments.items(),
        ),
    ]


def generate_launch_description():
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
            OpaqueFunction(function=_include_profile),
        ]
    )
