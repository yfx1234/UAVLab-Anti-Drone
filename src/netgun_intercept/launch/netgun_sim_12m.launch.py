"""网枪链完整架构 v2 仿真入口。

架构（docs/网枪完整架构v2.md）：
  传感器三区（>12m 方位 / 10-12m 不稳定深度 / <10m 稳定深度）
  → IMM CV/CA/CT（协方差自适应）
  → netgun_guidance_node（三状态机 + 目标速度前馈 + 方位伺服）
  → netgun_strike_node（火控 2°/4° + 单发）
  → 裁判（3×3 即时覆盖）

布置：目标 (0,0) 12m 高；追踪机 (0,-2) 起飞 5.8m（下方 6.2m 站位）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TARGET_STATE_TOPIC = "/tracking/target/state"
COMMON_ORIGIN_TOPIC = "/tracking/common_origin"
TRUTH_TOPIC = "/evaluation/relative_truth"
DEPTH_TOPIC = "/perception/depth/observation"
ESTIMATE_TOPIC = "/tracking/relative/estimate"
SHOT_TOPIC = "/tracking/virtual_shot_event"

# ZED X Mini 4mm 镜头真实内参（Stereolabs 官方标定表，HD1080 1920x1080，
# 焦距 1274px，HFOV 74° / VFOV 45°；运行时应以 SDK getCameraInformation()
# 实标值为准）。深度区按用户实测：15m 内良好（规格书 4mm 上限 12m，
# 以实测为准）。
CAMERA_FX = 1274.0
CAMERA_FY = 1274.0
CAMERA_CX = 960.0
CAMERA_CY = 540.0
CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("target_scenario", default_value="straight"),
            DeclareLaunchArgument("target_speed_mps", default_value="5.0"),
            DeclareLaunchArgument("target_height_m", default_value="12.0"),
            DeclareLaunchArgument("target_radius_m", default_value="20.0"),
            DeclareLaunchArgument("target_motion_delay_s", default_value="10.0"),
            # 运动斜坡：镜像（netgun_mirror_run.py）目标在 SETTLE 瞬间跳到
            # 全速（等效 ramp≈0）→ 追踪机产生持续加速倾斜、机体轴与瞄准
            # 方向对齐形成开火窗口。3.0s 斜坡过软，倾斜不足 2° 主门难开。
            DeclareLaunchArgument("target_motion_ramp_s", default_value="2.0"),
            # 迎敌几何：目标初始北向偏移（负值=从南侧逼近；0=头顶起跑追击）
            DeclareLaunchArgument("target_initial_north_m", default_value="0.0"),
            # 航向伺服偏移（A/B 对比用）：0=机头沿航线（宽轴对准航线，
            # 实物安装几何）；π/2=机头垂直航线（窄轴对准航线=旧安装几何）
            DeclareLaunchArgument("yaw_course_offset_rad", default_value="0.0"),
            # 检测/深度分离区：>此距离 depth_valid=false 而 visible 保留
            # （复现实机 YOLO 检测在、估深无输出；-1=关闭=定案行为）
            DeclareLaunchArgument("depth_valid_max_m", default_value="-1.0"),
            # 相机帧率（实机硬件上限 15Hz；30=仿真默认）
            DeclareLaunchArgument("camera_rate_hz", default_value="30.0"),
            # 发散恢复阈值（2026-10-15 改秒）：距上次有效更新超此秒数
            # 强制重置滤波器；与相机帧率无关（任意频率零改参）
            DeclareLaunchArgument("divergence_reset_period_s", default_value="1.5"),
            # engage 模式：auto=仿真自动解锁+offboard；rc_gated=实机
            # 门控（切入前零 setpoint/切出即停/永不下发指令）。
            # auto 需 allow_auto_engage 白名单（R0-1 防误配）
            DeclareLaunchArgument("engage_mode", default_value="auto"),
            DeclareLaunchArgument("allow_auto_engage", default_value="true"),
            # BEARING_HOLD 导航（§八-8 死区外推，2026-10-15 矩阵证伪默认关；
            # false=§十七 方位逼近 3.5 m/s = matrix_geo2 验证口径）
            DeclareLaunchArgument(
                "bearing_hold_intercept_nav", default_value="false"
            ),
            # （仅 intercept_nav=true 时生效）死区外推逼近速度
            DeclareLaunchArgument("intercept_speed_mps", default_value="8.0"),
            # 初始搜索慢自旋（实机默认开；仿真默认关保矩阵口径，验证传 true）
            DeclareLaunchArgument("search_spin_enabled", default_value="false"),
            # 授权：就位后最短延时 + 连续有效跟踪时长（strike）
            DeclareLaunchArgument("attack_enable_delay_s", default_value="12.0"),
            DeclareLaunchArgument("stable_track_time_s", default_value="2.0"),
            DeclareLaunchArgument(
                "primary_axis_error_rad", default_value="0.03490658503988659"
            ),
            DeclareLaunchArgument(
                "window_axis_error_rad", default_value="0.06981317007977318"
            ),
            # ---- 目标源（virtual 合成，直发目标状态） ----
            Node(
                package="px4_offboard_tracking",
                executable="target_state_source",
                namespace="simulation",
                name="target_state_source",
                output="screen",
                parameters=[
                    {
                        "target_source": "virtual",
                        "target_state_topic": TARGET_STATE_TOPIC,
                        "common_origin_topic": COMMON_ORIGIN_TOPIC,
                        "mission_ready_topic": "/simulation/target_motion_ready",
                        "pursuer_takeoff_ready_topic": "/simulation/pursuer_takeoff_ready",
                        # 目标等追踪机进入 TRACK（就位信号）再起锚机动；
                        # False 会让目标在追踪机爬升段提前离场（视场死锁）
                        "wait_for_pursuer_takeoff_ready": True,
                        "target_scenario": LaunchConfiguration("target_scenario"),
                        "target_speed_mps": LaunchConfiguration("target_speed_mps"),
                        "target_heading_rad": 0.0,
                        "target_radius_m": LaunchConfiguration("target_radius_m"),
                        "target_motion_delay_s": LaunchConfiguration(
                            "target_motion_delay_s"
                        ),
                        "target_motion_ramp_s": LaunchConfiguration(
                            "target_motion_ramp_s"
                        ),
                        "target_height_m": LaunchConfiguration("target_height_m"),
                        "virtual_initial_north_m": LaunchConfiguration(
                            "target_initial_north_m"
                        ),
                        "virtual_initial_east_m": -2.0,
                        # 关键：默认指向 /fmu/（目标机），必须显式指向追踪机
                        "pursuer_local_topic": "/px4_1/fmu/out/vehicle_local_position",
                        "pursuer_global_topic": "/px4_1/fmu/out/vehicle_global_position",
                        # 关键：默认 false 会订阅全局位置（GPS）而非本地位置；
                        # 设 true 才会创建 pursuer_local_sub 并使用本地位置
                        "virtual_use_pursuer_local_position": True,
                    }
                ],
            ),
            # ---- 真值桥（local 模式） ----
            Node(
                package="px4_offboard_tracking",
                executable="simulation_truth_bridge",
                namespace="simulation",
                name="simulation_truth_bridge",
                output="screen",
                parameters=[
                    {
                        "target_state_topic": TARGET_STATE_TOPIC,
                        "task_origin_topic": "/tracking/task_origin_ned",
                        "pursuer_position_source": "local",
                        "pursuer_local_topic": "/px4_1/fmu/out/vehicle_local_position",
                        "output_topic": TRUTH_TOPIC,
                    }
                ],
            ),
            # ---- 传感器（三区深度模型） ----
            Node(
                package="px4_offboard_tracking",
                executable="logical_camera_simulator",
                namespace="simulation",
                name="logical_depth_camera_simulator",
                output="screen",
                parameters=[
                    {
                        "camera_output": "depth",
                        "truth_topic": TRUTH_TOPIC,
                        "attitude_topic": "/px4_1/fmu/out/vehicle_attitude",
                        "depth_topic": DEPTH_TOPIC,
                        "camera_rate_hz": LaunchConfiguration("camera_rate_hz"),
                        "depth_camera_fx": CAMERA_FX,
                        "depth_camera_fy": CAMERA_FY,
                        "depth_camera_cx": CAMERA_CX,
                        "depth_camera_cy": CAMERA_CY,
                        "depth_camera_width": CAMERA_WIDTH,
                        "depth_camera_height": CAMERA_HEIGHT,
                        "depth_horizontal_half_fov_rad": 0.6458,
                        "depth_vertical_half_fov_rad": 0.3927,
                        # 三区深度（架构 v2 §1）
                        "depth_camera_maximum_depth_m": 16.0,
                        "depth_reliable_max_m": 15.0,
                        "depth_valid_max_m": LaunchConfiguration(
                            "depth_valid_max_m"
                        ),
                        "depth_unstable_valid_probability": 0.4,
                        "depth_unstable_noise_multiplier": 4.0,
                        "pixel_noise_stddev_px": 1.5,
                        "depth_noise_stddev_m": 0.10,
                        # 姿态采样容差放宽（默认 0.05s 在 Gazebo 里容易 miss）
                        "attitude_sync_tolerance_s": 0.2,
                    }
                ],
            ),
            # ---- IMM 估计器 ----
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
                        "attitude_topic": "/px4_1/fmu/out/vehicle_attitude",
                        "camera_fx": CAMERA_FX,
                        "camera_fy": CAMERA_FY,
                        "camera_cx": CAMERA_CX,
                        "camera_cy": CAMERA_CY,
                        "camera_width": CAMERA_WIDTH,
                        "camera_height": CAMERA_HEIGHT,
                        # bearing-only 更新（2026-10-14）：检测在而深度
                        # 不在的观测退化为纯方位量测（横向修正、距离大
                        # 方差）；火控侧因位置方差上限自然闭门
                        "bearing_only_update_enabled": True,
                        "bearing_only_nominal_range_m": 12.0,
                        "bearing_only_range_sigma_m": 30.0,
                        "divergence_reset_period_s": LaunchConfiguration(
                            "divergence_reset_period_s"
                        ),
                        # 安装四元数（实物安装 2026-10-13 定案）：宽轴
                        # （H74°/u 轴）沿机头。须与相机模拟器
                        # default_upward_optical_quaternion() 严格一致，
                        # 正反变换错位会悄悄污染 LOS 方向
                        "camera_q_body_to_optical_w": 0.0,
                        "camera_q_body_to_optical_x": 1.0,
                        "camera_q_body_to_optical_y": 0.0,
                        "camera_q_body_to_optical_z": 0.0,
                        # 姿态采样容差放宽：与相机同款问题（0.05s 默认在 Gazebo
                        # 姿态流抖动下会拒收全部观测 → 滤波器永不初始化，
                        # strike 侧表现为 est=0/range=0）
                        "attitude_sync_tolerance_s": 0.2,
                        # 起步加速瞬态鲁棒性（onset bag 定位）：防拒收锁死
                        "innovation_gate_mahalanobis": 25.0,
                        "acceleration_noise_stddev_mps2": 4.0,
                    }
                ],
            ),
            # ---- 制导节点（三状态 + 前馈 + 方位伺服，替代 simple_below_tracker） ----
            Node(
                package="netgun_intercept",
                executable="netgun_guidance_node",
                namespace="pursuer",
                name="netgun_guidance_node",
                output="screen",
                parameters=[
                    {
                        "relative_estimate_topic": ESTIMATE_TOPIC,
                        "local_position_topic": "/px4_1/fmu/out/vehicle_local_position",
                        "status_topic": "/px4_1/fmu/out/vehicle_status",
                        "takeoff_height_m": 5.8,
                        "minimum_height_m": 1.6,
                        # 机动上限（2026-10-14 实机数据修订：80% 油门=
                        # 200A 电气上限、推重比 3 → 推力不再是约束，改由
                        # 相机 FOV 决定：倾角 35°/加速度 6.5（u 轴半视场
                        # 37° 留余量）；速度上限维持 10）
                        "horizontal_position_gain": 2.0,
                        "maximum_horizontal_speed_mps": 10.0,
                        "maximum_horizontal_acceleration_mps2": 6.5,
                        "altitude_position_gain": 1.0,
                        "maximum_vertical_speed_mps": 1.5,
                        "feedforward_gain": 1.0,
                        # engage 模式（§八-9 实机）：auto/rc_gated
                        "engage_mode": LaunchConfiguration("engage_mode"),
                        "allow_auto_engage":
                            LaunchConfiguration("allow_auto_engage"),
                        # 航向伺服（交接清单 §八-1）：机头=相机宽轴对准
                        # 目标航线；锁定模式保持（估计失效/目标静止不转）
                        "yaw_servo_enabled": True,
                        "yaw_course_offset_rad": LaunchConfiguration(
                            "yaw_course_offset_rad"
                        ),
                        "yaw_max_rate_rad_s": 3.0,
                        "yaw_engage_speed_mps": 0.5,
                        # BEARING_HOLD（§八-3②）：估计失效→对准最后方位
                        # +慢扫描+逼近（复用 approach_speed_mps），确认回跟踪
                        "bearing_hold_enabled": True,
                        "bearing_hold_sweep_rad": 0.44,
                        "bearing_hold_sweep_period_s": 8.0,
                        "bearing_hold_recover_confirm_s": 0.3,
                        # 减速感知交接：恢复须实际速度<1m/s（或 2s 兜底）
                        "bearing_hold_recover_speed_mps": 1.0,
                        "bearing_hold_recover_timeout_s": 2.0,
                        "bearing_hold_max_duration_s": 20.0,
                        "bearing_hold_intercept_nav": LaunchConfiguration(
                            "bearing_hold_intercept_nav"
                        ),
                        "intercept_speed_mps": LaunchConfiguration(
                            "intercept_speed_mps"
                        ),
                        # 方位伺服（§八-3 并入）：瞄准线死区外速度偏置
                        # （k=0.25 → a≈g·xy/h，收敛中提前倾斜对准）
                        "bearing_servo_enabled": True,
                        "bearing_servo_gain": 0.25,
                        "bearing_servo_deadband_m": 0.15,
                        # 摆动抑制（§八-4）：反馈用估计低通 + 积分 0.3→0.1
                        "estimate_lowpass_tau_s": 0.3,
                        "horizontal_integral_gain": 0.1,
                        # AIM 主动瞄准（§八-0 完整版，B 路线加速度前馈）：
                        # 稳定跟踪+授权后主动倾斜，2° 实测门验收真实指向
                        "aim_mode_enabled": True,
                        "aim_enter_delay_s": 2.0,
                        "aim_max_tilt_rad": 0.35,
                        "aim_timeout_s": 0.8,
                        "aim_max_target_speed_mps": 100.0,
                        "aim_tilt_compensation": 1.15,
                        "weapon_authorization_topic": "/tracking/weapon_authorization",
                        # 开火后 360° 旋转标志（人工核验已激发）：
                        # 先悬停制动（SETTLE）停稳确认再起旋
                        "spin_flag_enabled": True,
                        "spin_rate_rad_s": 1.8,
                        "spin_total_rad": 6.2832,
                        "spin_settle_speed_mps": 0.25,
                        "spin_settle_confirm_s": 0.3,
                        "spin_settle_timeout_s": 4.0,
                        "search_spin_enabled": LaunchConfiguration(
                            "search_spin_enabled"
                        ),
                    }
                ],
            ),
            # ---- 火控节点 ----
            Node(
                package="netgun_intercept",
                executable="netgun_strike_node",
                namespace="pursuer",
                name="netgun_strike_node",
                output="screen",
                parameters=[
                    {
                        "relative_estimate_topic": ESTIMATE_TOPIC,
                        "attitude_topic": "/px4_1/fmu/out/vehicle_attitude",
                        "status_topic": "/px4_1/fmu/out/vehicle_status",
                        "local_position_topic": "/px4_1/fmu/out/vehicle_local_position",
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
                        # 授权对齐镜像 SETTLE_S：目标在就位信号后 delay 10s 起跑，
                        # 火控授权同样延时 10s（自 TRACK 起），交战发生在机动中
                        "attack_enable_delay_s": LaunchConfiguration(
                            "attack_enable_delay_s"
                        ),
                        "stable_track_time_s": LaunchConfiguration(
                            "stable_track_time_s"
                        ),
                        "engagement_timeout_s": 0.0,
                        "pursuer_velocity_compensation": False,
                        # 武器授权单一真源（§八-0）：guidance AIM 进入条件
                        "weapon_authorization_topic": "/tracking/weapon_authorization",
                    }
                ],
            ),
            # ---- 裁判（3×3 即时覆盖） ----
            Node(
                package="virtual_interception_referee",
                executable="virtual_projectile_referee",
                namespace="evaluation",
                name="virtual_projectile_referee",
                output="screen",
                parameters=[
                    {
                        "shot_topic": SHOT_TOPIC,
                        "result_topic": "/evaluation/virtual_hit_result",
                        "truth_topic": TRUTH_TOPIC,
                        "task_origin_topic": "/tracking/task_origin_ned",
                        "pursuer_local_topic": "/px4_1/fmu/out/vehicle_local_position",
                        "pursuer_attitude_topic": "/px4_1/fmu/out/vehicle_attitude",
                        "instant_coverage_mode": True,
                        "projectile_speed_mps": 55.0,
                        "maximum_flight_time_s": 0.5,
                        "coverage_half_extent_m": 1.5,
                        "effective_range_min_m": 4.0,
                        "effective_range_max_m": 10.0,
                        "required_hits": 1,
                    }
                ],
            ),
        ]
    )
