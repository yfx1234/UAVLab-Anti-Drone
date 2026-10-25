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
  - 开火（I-3，2026-10-21）：fire_mode=auto_manual——火控门满足即自动单发，
    同时开放面板"发射网枪"按钮作人工兜底（自动一直不发时用；物理安全门：
    飞控新鲜+已解锁+高度+倾角≤45°）。两路共享单发闭锁，全任务只发一次。
    开火事件 /tracking/virtual_shot_event；实物击发经 trigger_command
    （PWM 脉冲脚本，异步执行，环境变量 NETGUN_SHOT_ID/NETGUN_SHOT_REASON）
  - 飞控 PX4 v1.15.x：状态话题 /fmu/out/vehicle_status（v1.16+ 才有 _v1）
  - 相机外参（I-3 安装约定，2026-10-23 按用户口径更新）：ZED X Mini 朝天、基线
    沿机头、右目在前，相机中心在网枪轴线右侧 6 cm、相机顶面与网枪顶面同高，
    飞控在网枪顶面下方 18 cm → 左目光心（ZED 坐标原点，顶面下 2 cm）相对飞控
    在 body FRD (-0.025, +0.06, -0.16)，枪口相对飞控 (0, 0, -0.18)；
    光轴与枪轴平行、无夹角，boresight 固定 0（不再做枪轴核对）
  - 安装参数（2026-10-23 修订）：相机顶面/RTK/飞控相对网枪顶面中心的位置与
    右目朝向在面板「拦截任务 → 安装参数」修改，存板上
    ~/netgun_config/mounting_geometry.json；run_interceptor_stack.sh start
    经 deploy/mounting_geometry.py launch-args 换算成下列 camera_* /
    muzzle_* 参数传入（重启拦截链后生效）。下面的默认值 = 未保存时的默认
    安装；RTK 天线偏置只给飞控参数建议值，不经本链

调试：四节点自动发布 /debug/{guidance,strike,estimator,bridge}_state
（2Hz/1Hz，key=value），用法见 docs/实物调试指南.md 与
src/zed_perception_bridge/README.md。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ESTIMATE_TOPIC = "/tracking/relative/estimate"
# PX4 v1.15.x 的状态话题（I-3：I-2 的 _v1 是 v1.16+ 兼容话题，本机固件没有）
STATUS_TOPIC_DEFAULT = "/fmu/out/vehicle_status"
DEPTH_TOPIC = "/perception/depth/observation"
SHOT_TOPIC = "/tracking/virtual_shot_event"


def _float_arg(name):
    """launch 参数按 float 传给节点（安装几何均为米/弧度）。"""
    return ParameterValue(LaunchConfiguration(name), value_type=float)


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
            # R0-4：AIM 与火控共用同一组枪口/展开参数（统一瞄准口径）
            DeclareLaunchArgument("muzzle_speed_mps", default_value="55.0"),
            DeclareLaunchArgument("deployment_time_s", default_value="0.055"),
            # I-3：开火模式 auto_manual=自动单发+手动兜底按钮（实机默认）；
            # auto=仅自动；manual=仅按钮（I-1 口径）
            DeclareLaunchArgument("fire_mode", default_value="auto_manual"),
            # 手动兜底门：false=只看物理安全门（解锁/高度/倾角）；true=再要求
            # 武器授权+有效估计
            DeclareLaunchArgument(
                "manual_fire_require_authorization", default_value="false"),
            # 实物击发钩子（PWM 脉冲脚本），空=仅虚拟事件（接网枪时装订）
            DeclareLaunchArgument("trigger_command", default_value=""),
            DeclareLaunchArgument("status_topic", default_value=STATUS_TOPIC_DEFAULT),
            # 相机（ZED 左目光心）相对机体原点（飞控）的平移，body FRD 米；
            # 2026-10-22 起由面板安装参数生成（mounting_geometry.py）
            DeclareLaunchArgument("camera_translation_body_x_m", default_value="-0.025"),
            DeclareLaunchArgument("camera_translation_body_y_m", default_value="0.06"),
            DeclareLaunchArgument("camera_translation_body_z_m", default_value="-0.16"),
            # 安装四元数 body→optical（相机朝天；默认 = 右目朝机头）
            DeclareLaunchArgument("camera_q_body_to_optical_w", default_value="0.0"),
            DeclareLaunchArgument("camera_q_body_to_optical_x", default_value="1.0"),
            DeclareLaunchArgument("camera_q_body_to_optical_y", default_value="0.0"),
            DeclareLaunchArgument("camera_q_body_to_optical_z", default_value="0.0"),
            # 网枪顶面中心（枪口）相对机体原点（飞控）的杠杆臂；枪轴与光轴
            # 平行（2026-10-23 用户确认），boresight 固定 0，仅留作工程覆盖
            DeclareLaunchArgument("muzzle_offset_body_x_m", default_value="0.0"),
            DeclareLaunchArgument("muzzle_offset_body_y_m", default_value="0.0"),
            DeclareLaunchArgument("muzzle_offset_body_z_m", default_value="-0.18"),
            DeclareLaunchArgument("boresight_pitch_rad", default_value="0.0"),
            DeclareLaunchArgument("boresight_yaw_rad", default_value="0.0"),
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
                        # 安装四元数（相机朝天；默认宽轴沿机头、右目在前）：须与
                        # 机加实物一致，面板安装参数「右目朝向」生成
                        "camera_q_body_to_optical_w": _float_arg(
                            "camera_q_body_to_optical_w"),
                        "camera_q_body_to_optical_x": _float_arg(
                            "camera_q_body_to_optical_x"),
                        "camera_q_body_to_optical_y": _float_arg(
                            "camera_q_body_to_optical_y"),
                        "camera_q_body_to_optical_z": _float_arg(
                            "camera_q_body_to_optical_z"),
                        # I-3：左目（ZED 坐标原点）杠杆臂——估计输出改为相对
                        # 机体原点（飞控），相机右偏不再混进相对位置
                        "camera_translation_body_x_m": _float_arg(
                            "camera_translation_body_x_m"),
                        "camera_translation_body_y_m": _float_arg(
                            "camera_translation_body_y_m"),
                        "camera_translation_body_z_m": _float_arg(
                            "camera_translation_body_z_m"),
                        "bearing_only_update_enabled": True,
                        "bearing_only_nominal_range_m": 12.0,
                        "bearing_only_range_sigma_m": 30.0,
                        # 秒单位，与相机帧率无关（实机 15Hz 零改参）
                        "divergence_reset_period_s": 1.5,
                        # 姿态采样容差：收紧到代码默认 0.05（R1-5；旧值 0.2s
                        # 在 1 rad/s 角速度下即 ~11° 姿态误差）。首次接真
                        # 飞控后按 estimator 诊断确认
                        "attitude_sync_tolerance_s": 0.05,
                        # PX4 与 ROS 时钟域判定容差：默认 5.0 过松，先收 0.5
                        #（UXRCE_DDS_SYNCT 开启时应走直通分支；实接后核对）
                        "attitude_clock_domain_tolerance_s": 0.5,
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
                        "status_topic": LaunchConfiguration("status_topic"),
                        "engage_mode": "rc_gated",
                        # F-1：本地位置 0.5s / vehicle_status 1.5s（PX4 实际
                        # 2Hz）；过期停发超过 1.0s 才复位；开火后不再追击
                        "local_position_timeout_s": 0.5,
                        "vehicle_status_timeout_s": 1.5,
                        "fcu_stale_reset_s": 1.0,
                        "post_shot_reengage": False,
                        # R0-4：AIM 与火控统一瞄准口径（与 strike 同参）
                        "muzzle_speed_mps": LaunchConfiguration(
                            "muzzle_speed_mps"),
                        "deployment_time_s": LaunchConfiguration(
                            "deployment_time_s"),
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
                        # 击发后安全后撤：网弹竖直上抛落回机体附近，先沿
                        # 机体后向水平退 post_shot_retreat_m 再稳定/旋转/
                        # 结束；0=旧行为（击发点原地保持）
                        "post_shot_retreat_m": 3.0,
                        "retreat_timeout_s": 4.5,
                        # 初始搜索慢自旋：无目标时原地转机头扫整圆天顶
                        "search_spin_enabled": True,
                        "search_spin_rate_rad_s": 0.42,
                        "weapon_authorization_topic": "/tracking/weapon_authorization",
                    }
                ],
            ),
            # ---- 火控（自动单发 + 手动兜底；实物击发经 trigger_command） ----
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
                        "status_topic": LaunchConfiguration("status_topic"),
                        "local_position_topic": "/fmu/out/vehicle_local_position",
                        "shot_topic": SHOT_TOPIC,
                        "frame_id": "map_ned",
                        "muzzle_speed_mps": LaunchConfiguration(
                            "muzzle_speed_mps"),
                        "coverage_half_extent_m": 1.5,
                        "deployment_time_s": LaunchConfiguration(
                            "deployment_time_s"),
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
                        # F-1：姿态/位置 1.0s，vehicle_status 单独 1.5s（2Hz）
                        "vehicle_status_timeout_s": 1.5,
                        # H-5：枪口相对机体原点的杠杆臂（body FRD, m）与枪管-
                        # 光轴安装角（rad）。I-3：网枪装在拦截器中轴线上 → x/y=0；
                        # 估计已扣相机杠杆臂（相对机体原点），这里不再含相机偏置。
                        # 2026-10-23：枪口偏置由面板安装参数（飞控位置）生成
                        "muzzle_offset_body_x_m": _float_arg("muzzle_offset_body_x_m"),
                        "muzzle_offset_body_y_m": _float_arg("muzzle_offset_body_y_m"),
                        "muzzle_offset_body_z_m": _float_arg("muzzle_offset_body_z_m"),
                        "boresight_pitch_rad": _float_arg("boresight_pitch_rad"),
                        "boresight_yaw_rad": _float_arg("boresight_yaw_rad"),
                        # I-3：自动单发 + 手动兜底（/netgun/fire_command，面板
                        # 按钮）；单发闭锁两路共享。仿真矩阵沿用 auto
                        "fire_mode": ParameterValue(
                            LaunchConfiguration("fire_mode"), value_type=str),
                        "manual_fire_require_authorization": ParameterValue(
                            LaunchConfiguration("manual_fire_require_authorization"),
                            value_type=bool),
                        # 开火倾角门（枪管轴偏离竖直上限，自动/手动共用）
                        "fire_max_tilt_rad": 0.7853981633974483,
                        # 实物击发钩子（PWM 脉冲命令，异步执行）；空=纯虚拟事件
                        "trigger_command": ParameterValue(
                            LaunchConfiguration("trigger_command"), value_type=str),
                        # 实机口径 2s（仿真 12s=与虚拟目标起跑对齐，仅仿真用）
                        "attack_enable_delay_s": LaunchConfiguration(
                            "attack_enable_delay_s"
                        ),
                        "stable_track_time_s": LaunchConfiguration(
                            "stable_track_time_s"
                        ),
                        "engagement_timeout_s": 0.0,
                        "pursuer_velocity_compensation": False,
                        "weapon_authorization_topic": "/tracking/weapon_authorization",
                    }
                ],
            ),
        ]
    )
