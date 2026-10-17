"""网枪拦截实机入口（2026-10-15 首版，六件套之四的骨架）。

与仿真入口（netgun_sim_12m.launch.py）的差异：
  - 无 Gazebo/虚拟目标/真值桥/裁判——感知由板上 ZED 感知（zed_tracking，
    main 侧部署）经 zed_perception_bridge 桥接提供：订阅其 JSON 观测通道
    /perception/observations_json（PerceptionFrame 合同 v2），映射发布
    /perception/depth/observation（DepthObservation V1 接口）
  - PX4 话题用实机命名空间 /fmu/...（无 /px4_1 前缀）
  - engage_mode=rc_gated：待机零 setpoint、永不下发 VehicleCommand
  - 授权延时 12→2s（仿真 12s 是与虚拟目标起跑对齐的口径；实机目标
    何时出现不可知，仅保留 2s 防抖）
  - 开火输出仍走 /tracking/virtual_shot_event——GPIO 触发节点（待做）
    订阅此话题执行硬件发射与联锁

调试：四节点自动发布 /debug/{guidance,strike,estimator,bridge}_state
（2Hz/1Hz，key=value），用法见 docs/实物调试指南.md 与
src/zed_perception_bridge/README.md。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ESTIMATE_TOPIC = "/tracking/relative/estimate"
DEPTH_TOPIC = "/perception/depth/observation"
SHOT_TOPIC = "/tracking/virtual_shot_event"


def generate_launch_description():
    return LaunchDescription(
        [
            # 授权延时（实机口径 2s；0=取消，慎用——失去就位防抖）
            DeclareLaunchArgument("attack_enable_delay_s", default_value="2.0"),
            DeclareLaunchArgument("stable_track_time_s", default_value="2.0"),
            DeclareLaunchArgument(
                "primary_axis_error_rad", default_value="0.03490658503988659"
            ),
            DeclareLaunchArgument(
                "window_axis_error_rad", default_value="0.06981317007977318"
            ),
            # 感知桥（板上 main 侧感知 -> 拦截链 V1 接口）
            DeclareLaunchArgument("use_perception_bridge", default_value="true"),
            DeclareLaunchArgument("bridge_selection", default_value="confidence"),
            # 桥输入话题可换（domain 0 全链测试用 /test/perception/observations_json
            # 避开真实感知的 JSON 流，见 tools/board_test/）
            DeclareLaunchArgument(
                "bridge_input_topic", default_value="/perception/observations_json"),
            # ---- 感知桥（PerceptionFrame v2 JSON -> DepthObservation V1） ----
            Node(
                package="zed_perception_bridge",
                executable="bridge_node",
                name="zed_perception_bridge",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_perception_bridge")),
                parameters=[
                    {
                        "input_json_topic": LaunchConfiguration(
                            "bridge_input_topic"),
                        "output_topic": DEPTH_TOPIC,
                        "debug_topic": "/debug/bridge_state",
                        "selection": LaunchConfiguration("bridge_selection"),
                    }
                ],
            ),
            # ---- IMM 估计器（输入=感知源 DepthObservation） ----
            Node(
                package="px4_offboard_tracking",
                executable="relative_estimator",
                namespace="pursuer",
                name="relative_estimator",
                output="screen",
                parameters=[
                    {
                        "observation_topic": DEPTH_TOPIC,
                        "estimate_topic": ESTIMATE_TOPIC,
                        "attitude_topic": "/fmu/out/vehicle_attitude",
                        # ZED X Mini 4mm 内参——2026-10-15 板上实测值
                        # （/debug/bridge_state 的 intrinsics_* 透传，HD1080），
                        # 替换初版 1274/960/540 假设值（cx 偏差 51.7px≈1.8°
                        # 视线偏差，影响 2° 火控门）
                        "camera_fx": 1266.5,
                        "camera_fy": 1266.5,
                        "camera_cx": 1011.7,
                        "camera_cy": 538.4,
                        "camera_width": 1920,
                        "camera_height": 1080,
                        # 安装四元数（宽轴沿机头）：与机加实物严格一致
                        "camera_q_body_to_optical_w": 0.0,
                        "camera_q_body_to_optical_x": 1.0,
                        "camera_q_body_to_optical_y": 0.0,
                        "camera_q_body_to_optical_z": 0.0,
                        "bearing_only_update_enabled": True,
                        "bearing_only_nominal_range_m": 12.0,
                        "bearing_only_range_sigma_m": 30.0,
                        # 秒单位，与相机帧率无关（实机 15Hz 零改参）
                        "divergence_reset_period_s": 1.5,
                        # 姿态采样容差：实机时间戳质量高于仿真，先沿用
                        "attitude_sync_tolerance_s": 0.2,
                        "innovation_gate_mahalanobis": 25.0,
                        "acceleration_noise_stddev_mps2": 4.0,
                    }
                ],
            ),
            # ---- 制导（rc_gated：切入前零 setpoint/切出即停/不下指令） ----
            Node(
                package="netgun_intercept",
                executable="netgun_guidance_node",
                namespace="pursuer",
                name="netgun_guidance_node",
                output="screen",
                # 输出话题在节点内硬编码为仿真命名空间 /px4_1/fmu/in/*（实机
                # 交接文档预留的 remap 方案），实机 PX4 走 /fmu/in/*。
                # vehicle_command 重映射到无订阅者的阻塞话题（R0-1，实机
                # "VehicleCommand 永久拦截"红线）：rc_gated 下本就不发，
                # 这里物理隔离兜底参数写错/回归的场景。
                remappings=[
                    ("/px4_1/fmu/in/offboard_control_mode",
                     "/fmu/in/offboard_control_mode"),
                    ("/px4_1/fmu/in/trajectory_setpoint",
                     "/fmu/in/trajectory_setpoint"),
                    ("/px4_1/fmu/in/vehicle_command",
                     "/netgun/blocked/vehicle_command"),
                ],
                parameters=[
                    {
                        "relative_estimate_topic": ESTIMATE_TOPIC,
                        "local_position_topic": "/fmu/out/vehicle_local_position",
                        "status_topic": "/fmu/out/vehicle_status",
                        "engage_mode": "rc_gated",
                        "takeoff_height_m": 5.8,
                        "minimum_height_m": 1.6,
                        "horizontal_position_gain": 2.0,
                        "maximum_horizontal_speed_mps": 10.0,
                        "maximum_horizontal_acceleration_mps2": 6.5,
                        "altitude_position_gain": 1.0,
                        "maximum_vertical_speed_mps": 1.5,
                        "feedforward_gain": 1.0,
                        "yaw_servo_enabled": True,
                        "yaw_max_rate_rad_s": 3.0,
                        "yaw_engage_speed_mps": 0.5,
                        "bearing_hold_enabled": True,
                        "bearing_hold_intercept_nav": False,
                        "bearing_hold_recover_speed_mps": 1.0,
                        "bearing_hold_recover_timeout_s": 2.0,
                        "bearing_servo_enabled": True,
                        "estimate_lowpass_tau_s": 0.3,
                        "horizontal_integral_gain": 0.1,
                        # AIM / SPIN（与仿真同参；倾角补偿实机需标定）
                        "aim_mode_enabled": True,
                        "aim_enter_delay_s": 2.0,
                        "aim_max_tilt_rad": 0.35,
                        "aim_timeout_s": 0.8,
                        "aim_tilt_compensation": 1.15,
                        "spin_flag_enabled": True,
                        "spin_rate_rad_s": 1.8,
                        "spin_total_rad": 6.2832,
                        # 初始搜索慢自旋：无目标时原地转机头扫整圆天顶
                        "search_spin_enabled": True,
                        "search_spin_rate_rad_s": 0.42,
                        "weapon_authorization_topic": "/tracking/weapon_authorization",
                    }
                ],
            ),
            # ---- 火控（开火事件输出口，GPIO 触发节点待接入） ----
            Node(
                package="netgun_intercept",
                executable="netgun_strike_node",
                namespace="pursuer",
                name="netgun_strike_node",
                output="screen",
                parameters=[
                    {
                        "relative_estimate_topic": ESTIMATE_TOPIC,
                        "attitude_topic": "/fmu/out/vehicle_attitude",
                        "status_topic": "/fmu/out/vehicle_status",
                        "local_position_topic": "/fmu/out/vehicle_local_position",
                        "shot_topic": SHOT_TOPIC,
                        "frame_id": "map_ned",
                        "muzzle_speed_mps": 55.0,
                        "coverage_half_extent_m": 1.5,
                        "deployment_time_s": 0.055,
                        "effective_range_min_m": 4.0,
                        "effective_range_max_m": 9.8,
                        "primary_axis_error_rad": LaunchConfiguration(
                            "primary_axis_error_rad"
                        ),
                        "window_axis_error_rad": LaunchConfiguration(
                            "window_axis_error_rad"
                        ),
                        "window_hold_s": 0.2,
                        "maximum_los_rate_rad_s": 0.5,
                        "blind_shot_enabled": False,
                        "minimum_height_m": 1.5,
                        # 实机口径 2s（仿真 12s=与虚拟目标起跑对齐，仅仿真用）
                        "attack_enable_delay_s": LaunchConfiguration(
                            "attack_enable_delay_s"
                        ),
                        "stable_track_time_s": LaunchConfiguration(
                            "stable_track_time_s"
                        ),
                        "engagement_timeout_s": 0.0,
                        "pursuer_velocity_compensation": True,
                        "weapon_authorization_topic": "/tracking/weapon_authorization",
                    }
                ],
            ),
        ]
    )
