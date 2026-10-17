"""最小下方跟踪的人工起飞/人工Offboard仿真入口。"""

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
        / "simple_below_tracker_sim.launch.py"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("tracking_mode", default_value="direct"),
            DeclareLaunchArgument("prediction_horizon_s", default_value="0.2"),
            DeclareLaunchArgument("target_scenario", default_value="straight"),
            DeclareLaunchArgument("target_speed_mps", default_value="1.2"),
            DeclareLaunchArgument("target_heading_rad", default_value="0.0"),
            DeclareLaunchArgument("target_radius_m", default_value="20.0"),
            DeclareLaunchArgument("target_path_length_m", default_value="40.0"),
            DeclareLaunchArgument("target_path_width_m", default_value="12.0"),
            DeclareLaunchArgument("target_motion_delay_s", default_value="5.0"),
            DeclareLaunchArgument("target_height_m", default_value="45.0"),
            DeclareLaunchArgument("pursuer_takeoff_height_m", default_value="5.0"),
            DeclareLaunchArgument("search_camera_rate_hz", default_value="30.0"),
            DeclareLaunchArgument("depth_camera_rate_hz", default_value="30.0"),
            DeclareLaunchArgument("control_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("observation_timeout_s", default_value="0.25"),
            DeclareLaunchArgument("horizontal_position_gain", default_value="0.6"),
            DeclareLaunchArgument("maximum_horizontal_speed_mps", default_value="0.5"),
            DeclareLaunchArgument(
                "maximum_horizontal_acceleration_mps2", default_value="0.3"
            ),
            DeclareLaunchArgument("maximum_vertical_speed_mps", default_value="1.0"),
            DeclareLaunchArgument("maximum_vehicle_speed_mps", default_value="8.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(base)),
                launch_arguments={
                    "tracking_mode": LaunchConfiguration("tracking_mode"),
                    "prediction_horizon_s": LaunchConfiguration(
                        "prediction_horizon_s"
                    ),
                    "target_scenario": LaunchConfiguration("target_scenario"),
                    "target_speed_mps": LaunchConfiguration("target_speed_mps"),
                    "target_heading_rad": LaunchConfiguration("target_heading_rad"),
                    "target_radius_m": LaunchConfiguration("target_radius_m"),
                    "target_path_length_m": LaunchConfiguration(
                        "target_path_length_m"
                    ),
                    "target_path_width_m": LaunchConfiguration(
                        "target_path_width_m"
                    ),
                    "target_motion_delay_s": LaunchConfiguration(
                        "target_motion_delay_s"
                    ),
                    "target_height_m": LaunchConfiguration("target_height_m"),
                    "pursuer_takeoff_height_m": LaunchConfiguration(
                        "pursuer_takeoff_height_m"
                    ),
                    "search_camera_rate_hz": LaunchConfiguration(
                        "search_camera_rate_hz"
                    ),
                    "depth_camera_rate_hz": LaunchConfiguration(
                        "depth_camera_rate_hz"
                    ),
                    "control_rate_hz": LaunchConfiguration("control_rate_hz"),
                    "observation_timeout_s": LaunchConfiguration(
                        "observation_timeout_s"
                    ),
                    "horizontal_position_gain": LaunchConfiguration(
                        "horizontal_position_gain"
                    ),
                    "maximum_horizontal_speed_mps": LaunchConfiguration(
                        "maximum_horizontal_speed_mps"
                    ),
                    "maximum_horizontal_acceleration_mps2": LaunchConfiguration(
                        "maximum_horizontal_acceleration_mps2"
                    ),
                    "maximum_vertical_speed_mps": LaunchConfiguration(
                        "maximum_vertical_speed_mps"
                    ),
                    "maximum_vehicle_speed_mps": LaunchConfiguration(
                        "maximum_vehicle_speed_mps"
                    ),
                    "target_wait_for_pursuer_takeoff_ready": "true",
                    "pursuer_control_enabled": "true",
                    "pursuer_auto_engage": "false",
                    "pursuer_takeoff_enabled": "false",
                    "pursuer_auto_disable_on_offboard_exit": "true",
                }.items(),
            ),
        ]
    )
