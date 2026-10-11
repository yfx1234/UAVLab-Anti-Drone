"""zed_perception_node 启动文件。

用法示例:
  ros2 launch zed_tracking zed_perception.launch.py
  ros2 launch zed_tracking zed_perception.launch.py record_enabled:=false
  ros2 launch zed_tracking zed_perception.launch.py \
      engine_path:=/path/to/exp_v_736x1280.engine
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    default_config = Path(
        get_package_share_directory("zed_tracking")
    ) / "config" / "zed_tracking.yaml"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "engine_path",
                default_value="/home/uavlab/Jetson_ROS2_V0/weights/candidates/"
                "platform_exp_20260909/exp_v_736x1280.engine",
                description="竖版 TensorRT 引擎路径",
            ),
            DeclareLaunchArgument(
                "record_enabled",
                default_value="true",
                description="是否开启 SVO 录像",
            ),
            Node(
                package="zed_tracking",
                executable="zed_perception_node",
                name="zed_perception_node",
                output="screen",
                parameters=[
                    str(default_config),
                    {
                        "engine_path": LaunchConfiguration("engine_path"),
                        "record_enabled": LaunchConfiguration("record_enabled"),
                    },
                ],
            ),
        ]
    )
