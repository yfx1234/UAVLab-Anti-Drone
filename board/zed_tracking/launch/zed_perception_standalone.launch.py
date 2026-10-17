from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Direct ROS launch is provided for development. Operational ownership/revision
    # checks and independent lifetime use tools/perception_supervisor.py instead.
    return LaunchDescription([Node(package='zed_tracking',executable='zed_perception_standalone',
        name='zed_perception_standalone',output='screen',parameters=[{'profile':'rgb'}])])
