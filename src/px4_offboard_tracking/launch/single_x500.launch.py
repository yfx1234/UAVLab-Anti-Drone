from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("topic_prefix", default_value="/fmu"),
        DeclareLaunchArgument("takeoff_height_m", default_value="5.0"),
        Node(
            package="px4_offboard_tracking",
            executable="offboard_vehicle",
            name="x500_offboard",
            output="screen",
            parameters=[{
                "role": "hold",
                "topic_prefix": LaunchConfiguration("topic_prefix"),
                "target_system": 1,
                "takeoff_height_m": LaunchConfiguration("takeoff_height_m"),
            }],
        ),
    ])
