"""3 对 1 多无人机协同拦截虚拟仿真 launch 文件。

用法：
    ros2 launch cooperative_intercept_fake_sim cooperative_intercept.launch.py \
        scenario:=encirclement
可选参数：
    scenario    straight | s_curve | turn | encirclement（默认 encirclement）
    config      自定义 YAML 配置文件路径（默认使用包内 config/simulation.yaml）
    time_scale  墙钟加速比：1.0 实时，10.0 十倍速，0.0 尽可能快（默认 1.0）
    results_dir 结果输出目录（默认 <package>/results/时间戳/）
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('cooperative_intercept_fake_sim')
    default_config = os.path.join(pkg_share, 'config', 'simulation.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario', default_value='encirclement',
            description='场景: straight / s_curve / turn / encirclement'),
        DeclareLaunchArgument(
            'config', default_value=default_config,
            description='仿真参数 YAML 文件路径'),
        DeclareLaunchArgument(
            'time_scale', default_value='1.0',
            description='墙钟加速比（0 = 尽可能快，1 = 实时）'),
        DeclareLaunchArgument(
            'results_dir', default_value='',
            description='结果输出目录（默认 <package>/results/时间戳/）'),

        Node(
            package='cooperative_intercept_fake_sim',
            executable='cooperative_intercept_sim',
            name='cooperative_intercept_sim',
            output='screen',
            parameters=[{
                'scenario': LaunchConfiguration('scenario'),
                'config_file': LaunchConfiguration('config'),
                'time_scale': LaunchConfiguration('time_scale'),
                'results_dir': LaunchConfiguration('results_dir'),
            }],
        ),
    ])
