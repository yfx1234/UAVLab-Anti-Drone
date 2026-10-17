"""双目触发的最小下方跟踪仿真。

目标真值仅用于 logical_camera_simulator 生成规范 DepthObservation；
simple_below_tracker 只接收双目消息和拦截机自身 PX4 状态。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    target_prefix = LaunchConfiguration("target_prefix")
    pursuer_prefix = LaunchConfiguration("pursuer_prefix")
    target_state_topic = "/tracking/target/state"
    common_origin_topic = "/tracking/common_origin"
    truth_topic = "/evaluation/relative_truth"
    search_topic = "/perception/search/observation"
    depth_topic = "/perception/depth/observation"

    return LaunchDescription(
        [
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
            DeclareLaunchArgument("target_prefix", default_value="/fmu"),
            DeclareLaunchArgument("pursuer_prefix", default_value="/px4_1/fmu"),
            DeclareLaunchArgument("tracking_mode", default_value="predictive"),
            DeclareLaunchArgument("prediction_horizon_s", default_value="0.2"),
            DeclareLaunchArgument("target_scenario", default_value="straight"),
            DeclareLaunchArgument("target_speed_mps", default_value="1.2"),
            DeclareLaunchArgument("search_camera_rate_hz", default_value="30.0"),
            DeclareLaunchArgument("depth_camera_rate_hz", default_value="30.0"),
            DeclareLaunchArgument("target_heading_rad", default_value="0.0"),
            DeclareLaunchArgument("target_radius_m", default_value="20.0"),
            DeclareLaunchArgument("target_path_length_m", default_value="40.0"),
            DeclareLaunchArgument("target_path_width_m", default_value="12.0"),
            # The custom Pursuer needs preflight stability, Offboard confirmation
            # and a 5 m takeoff before horizontal tracking is admitted. Keep the
            # target stationary long enough for that sequence; do not widen FOV.
            DeclareLaunchArgument("target_motion_delay_s", default_value="25.0"),
            DeclareLaunchArgument(
                "target_wait_for_pursuer_takeoff_ready", default_value="false"
            ),
            # This depth-only baseline has no search-camera handover. A 45 m /
            # 5 m geometry leaves the actual multi-instance spawn offset inside
            # the unchanged 10 degree logical depth-camera boundary.
            DeclareLaunchArgument("target_height_m", default_value="45.0"),
            DeclareLaunchArgument("pursuer_takeoff_height_m", default_value="5.0"),
            DeclareLaunchArgument("horizontal_position_gain", default_value="0.6"),
            DeclareLaunchArgument("control_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("observation_timeout_s", default_value="0.25"),
            DeclareLaunchArgument("maximum_horizontal_speed_mps", default_value="3.0"),
            DeclareLaunchArgument(
                "maximum_horizontal_acceleration_mps2", default_value="1.5"
            ),
            DeclareLaunchArgument("maximum_vertical_speed_mps", default_value="1.0"),
            DeclareLaunchArgument("maximum_vehicle_speed_mps", default_value="8.0"),
            # Defaults preserve the automatic algorithm-regression workflow.
            # Override all four arguments as documented to rehearse the
            # real-aircraft manual takeoff and Offboard switching sequence.
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
            Node(
                package="px4_offboard_tracking",
                executable="offboard_vehicle",
                namespace="target",
                name="offboard_vehicle",
                output="screen",
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
                        "target_wait_for_pursuer_takeoff_ready": LaunchConfiguration(
                            "target_wait_for_pursuer_takeoff_ready"
                        ),
                        "target_motion_ramp_s": 3.0,
                        "target_trajectory_feedforward_enabled": True,
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
                        "pursuer_global_topic":
                            "/px4_1/fmu/out/vehicle_global_position",
                        "pursuer_local_topic":
                            "/px4_1/fmu/out/vehicle_local_position",
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
                        "attitude_topic": "/px4_1/fmu/out/vehicle_attitude",
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
                        "attitude_topic": "/px4_1/fmu/out/vehicle_attitude",
                        "depth_topic": depth_topic,
                        "camera_rate_hz": LaunchConfiguration(
                            "depth_camera_rate_hz"
                        ),
                        "depth_horizontal_half_fov_rad": 0.17453292519943295,
                        "depth_vertical_half_fov_rad": 0.17453292519943295,
                    }
                ],
            ),
            Node(
                package="px4_offboard_tracking",
                executable="simple_below_tracker",
                namespace="pursuer",
                name="simple_below_tracker",
                output="screen",
                parameters=[
                    {
                        "topic_prefix": pursuer_prefix,
                        "target_system": 2,
                        "depth_topic": depth_topic,
                        "tracking_mode": LaunchConfiguration("tracking_mode"),
                        "prediction_horizon_s": LaunchConfiguration(
                            "prediction_horizon_s"
                        ),
                        "horizontal_position_gain": LaunchConfiguration(
                            "horizontal_position_gain"
                        ),
                        "control_rate_hz": LaunchConfiguration("control_rate_hz"),
                        "observation_timeout_s": LaunchConfiguration(
                            "observation_timeout_s"
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
                        "control_enabled": LaunchConfiguration(
                            "pursuer_control_enabled"
                        ),
                        "auto_engage": LaunchConfiguration(
                            "pursuer_auto_engage"
                        ),
                        "auto_disable_on_offboard_exit": LaunchConfiguration(
                            "pursuer_auto_disable_on_offboard_exit"
                        ),
                        "takeoff_enabled": LaunchConfiguration(
                            "pursuer_takeoff_enabled"
                        ),
                        "publish_pursuer_takeoff_ready": LaunchConfiguration(
                            "target_wait_for_pursuer_takeoff_ready"
                        ),
                        "takeoff_height_m": LaunchConfiguration(
                            "pursuer_takeoff_height_m"
                        ),
                        "minimum_flight_height_m": 3.0,
                    }
                ],
            ),
        ]
    )
