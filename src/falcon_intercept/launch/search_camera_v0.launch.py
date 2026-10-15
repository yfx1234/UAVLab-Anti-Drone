from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    device_path = LaunchConfiguration("device_path")
    image_topic = LaunchConfiguration("image_topic")
    frame_id = LaunchConfiguration("frame_id")
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    fps = LaunchConfiguration("fps")
    fourcc = LaunchConfiguration("fourcc")
    capture_backend = LaunchConfiguration("capture_backend")
    publisher_backend = LaunchConfiguration("publisher_backend")
    device_id = LaunchConfiguration("device_id")
    reliability = LaunchConfiguration("reliability")

    return LaunchDescription(
        [
            DeclareLaunchArgument("device_path", default_value="/dev/video0"),
            DeclareLaunchArgument("image_topic", default_value="/search/image_raw"),
            DeclareLaunchArgument("frame_id", default_value="search_camera"),
            DeclareLaunchArgument("width", default_value="640"),
            DeclareLaunchArgument("height", default_value="480"),
            DeclareLaunchArgument("fps", default_value="30.0"),
            DeclareLaunchArgument("fourcc", default_value="MJPG"),
            DeclareLaunchArgument("capture_backend", default_value="auto"),
            DeclareLaunchArgument("publisher_backend", default_value="image_tools"),
            DeclareLaunchArgument("device_id", default_value="0"),
            DeclareLaunchArgument("reliability", default_value="best_effort"),
            Node(
                package="image_tools",
                executable="cam2image",
                name="search_camera_cpp_node",
                output="screen",
                condition=LaunchConfigurationEquals("publisher_backend", "image_tools"),
                remappings=[("image", image_topic)],
                parameters=[
                    {
                        "device_id": ParameterValue(device_id, value_type=int),
                        "width": ParameterValue(width, value_type=int),
                        "height": ParameterValue(height, value_type=int),
                        "frequency": ParameterValue(fps, value_type=float),
                        "frame_id": frame_id,
                        "show_camera": False,
                        "reliability": reliability,
                        "history": "keep_last",
                        "depth": 10,
                    }
                ],
            ),
            Node(
                package="falcon_intercept",
                executable="v4l2_camera_node",
                name="search_camera_node",
                output="screen",
                condition=LaunchConfigurationEquals("publisher_backend", "python"),
                parameters=[
                    {
                        "device_path": device_path,
                        "image_topic": image_topic,
                        "frame_id": frame_id,
                        "width": ParameterValue(width, value_type=int),
                        "height": ParameterValue(height, value_type=int),
                        "fps": ParameterValue(fps, value_type=float),
                        "fourcc": fourcc,
                        "capture_backend": capture_backend,
                    }
                ],
            ),
        ]
    )
