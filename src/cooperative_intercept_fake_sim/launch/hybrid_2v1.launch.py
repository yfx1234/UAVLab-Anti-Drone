"""2 虚 1 实混合协同拦截 launch：实机 14 m/s 统一拦截方案 + 两架 PX4 僚机包抄。

实机部分复用原项目 predictive_interception_sim.launch.py（零修改），
本 launch 仅追加混合节点：订阅目标/实机真值，用目标真值为 P2/P3
生成位置参考，由 PX4 instance 2/3 执行，并记录轨迹、指标与图册。

用法：
    ros2 launch cooperative_intercept_fake_sim hybrid_2v1.launch.py \
        target_scenario:=straight target_speed_mps:=8.0
可选参数（透传给原项目 14 m/s 方案）：
    target_scenario    straight|circle|s_curve|figure_eight|rectangle
    target_speed_mps   目标速度（默认 8.0）
    target_heading_rad 目标初始航向（默认 0）
    command_maximum_speed_mps  实机指令速度上限（默认 14.0）
    safety_maximum_speed_mps   实机安全速度上限（默认 16.0）
    config             僚机配置文件（默认包内 config/hybrid.yaml）
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 原项目 14 m/s 统一拦截方案 launch（不改动）
REAL_LAUNCH = os.path.join(
    get_package_share_directory('virtual_interception_referee'),
    'launch', 'predictive_interception_sim.launch.py')

# 默认参数与"成功迎头组"（165 轮扫描 head_on）一致；由 run_hybrid_2v1.sh
# 按 HEAD_ON_MODE 覆盖 bearing / attack 时序等。
PARAMS = {
    'target_scenario': 'straight',
    'target_speed_mps': '8.0',
    'target_heading_rad': '0.0',
    'target_motion_ramp_s': '1.0',
    'command_maximum_speed_mps': '14.0',
    'safety_maximum_speed_mps': '16.0',
    'known_target_direction_enabled': 'false',
    'known_target_course_enabled': 'false',
    'known_target_course_ned_rad': '0.0',
    'known_target_search_velocity_mps': '8.0',
    # ---- 迎头成功组时序参数（head_on 覆盖为下述值）----
    'known_target_search_hold_s': '12.0',
    'known_target_bearing_ned_rad': '0.0',
    'attack_enable_delay_s': '10.0',
    'estimate_stability_hold_s': '1.0',
    'first_attack_tracking_hold_s': '3.0',
    'terminal_alignment_tracking_hold_s': '2.0',
    'terminal_alignment_ramp_s': '1.0',
}


def generate_launch_description():
    config = LaunchConfiguration('config')
    args = []
    for name, default in PARAMS.items():
        args.append(DeclareLaunchArgument(name, default_value=default))
    args.append(DeclareLaunchArgument(
        'config', default_value=os.path.join(
            get_package_share_directory('cooperative_intercept_fake_sim'),
            'config', 'hybrid.yaml')))
    args.append(DeclareLaunchArgument(
        'wingman_enabled', default_value='true',
        description='僚机 offboard 使能（false = 僚机停地面，对照实验）'))
    args.append(DeclareLaunchArgument(
        'duration_s', default_value='45.0',
        description='hybrid 记录时长上限（实机 COMPLETE 后提前结束）'))

    # 透传给原项目方案的关键参数
    launch_args = [(name, LaunchConfiguration(name)) for name in PARAMS]

    args += [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(REAL_LAUNCH),
            launch_arguments=launch_args,
        ),
        Node(
            package='cooperative_intercept_fake_sim',
            executable='hybrid_2v1_sim',
            name='hybrid_2v1_sim',
            output='screen',
            parameters=[{'config_file': config,
                         'duration_s': LaunchConfiguration('duration_s')}],
        ),
        Node(
            package='cooperative_intercept_fake_sim',
            executable='wingman_offboard',
            name='wingman_offboard',
            output='screen',
            parameters=[{'config_file': config,
                         'wingman_enabled': LaunchConfiguration(
                             'wingman_enabled')}],
        ),
    ]
    return LaunchDescription(args)
