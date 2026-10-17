"""netgun_strike 打击节点骨架入口。

只启动打击节点本体；感知/估计/制导由既有链路（logical_camera_simulator
真值坐标变换虚拟相机 → relative_estimator → pure_interceptor_controller）
在各自入口中启动，仿真布置见 docs/双目8m网枪拦截方案.md §7。
"""

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    config = str(
        Path(get_package_share_directory("netgun_intercept"))
        / "config"
        / "netgun_strike.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="netgun_intercept",
                executable="netgun_strike_node",
                name="netgun_strike_node",
                output="screen",
                parameters=[config],
            ),
        ]
    )
