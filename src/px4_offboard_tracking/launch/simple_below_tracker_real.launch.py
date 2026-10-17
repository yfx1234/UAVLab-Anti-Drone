"""实物飞机最小下方跟踪入口。

本入口不启动仿真真值、逻辑相机、估计器或裁判。默认不自动解锁、不切换
Offboard；节点在起飞前启动并预发送，遥控器切入Offboard即开始控制。
实物敌机由另一台电脑独立Offboard控制；本入口只连接拦截机自身/fmu。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
            DeclareLaunchArgument(
                "depth_topic", default_value="/perception/depth/observation"
            ),
            DeclareLaunchArgument("pursuer_prefix", default_value="/fmu"),
            DeclareLaunchArgument("tracking_mode", default_value="predictive"),
            DeclareLaunchArgument("prediction_horizon_s", default_value="0.2"),
            DeclareLaunchArgument("control_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("observation_timeout_s", default_value="0.25"),
            DeclareLaunchArgument("horizontal_position_gain", default_value="0.4"),
            DeclareLaunchArgument("maximum_horizontal_speed_mps", default_value="1.5"),
            DeclareLaunchArgument(
                "maximum_horizontal_acceleration_mps2", default_value="0.8"
            ),
            DeclareLaunchArgument("maximum_vertical_speed_mps", default_value="0.5"),
            DeclareLaunchArgument("minimum_target_height_above_m", default_value="2.0"),
            DeclareLaunchArgument("maximum_tracking_range_m", default_value="40.0"),
            DeclareLaunchArgument("control_enabled", default_value="true"),
            DeclareLaunchArgument("auto_engage", default_value="false"),
            DeclareLaunchArgument(
                "auto_disable_on_offboard_exit", default_value="true"
            ),
            Node(
                package="px4_offboard_tracking",
                executable="simple_below_tracker",
                namespace="pursuer",
                name="simple_below_tracker",
                output="screen",
                parameters=[
                    {
                        "topic_prefix": LaunchConfiguration("pursuer_prefix"),
                        "target_system": 1,
                        "depth_topic": LaunchConfiguration("depth_topic"),
                        "tracking_mode": LaunchConfiguration("tracking_mode"),
                        "prediction_horizon_s": LaunchConfiguration(
                            "prediction_horizon_s"
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
                        "minimum_target_height_above_m": LaunchConfiguration(
                            "minimum_target_height_above_m"
                        ),
                        "maximum_tracking_range_m": LaunchConfiguration(
                            "maximum_tracking_range_m"
                        ),
                        "control_enabled": LaunchConfiguration("control_enabled"),
                        "auto_engage": LaunchConfiguration("auto_engage"),
                        "auto_disable_on_offboard_exit": LaunchConfiguration(
                            "auto_disable_on_offboard_exit"
                        ),
                        "takeoff_enabled": False,
                    }
                ],
            ),
        ]
    )
