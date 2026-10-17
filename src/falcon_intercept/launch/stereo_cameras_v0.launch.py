from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    fps = LaunchConfiguration("fps")
    front_sensor_id = LaunchConfiguration("front_sensor_id")
    back_sensor_id = LaunchConfiguration("back_sensor_id")
    left_topic = LaunchConfiguration("left_topic")
    right_topic = LaunchConfiguration("right_topic")
    stereo_color_correction = LaunchConfiguration("stereo_color_correction")

    common = {
        "width": ParameterValue(width, value_type=int),
        "height": ParameterValue(height, value_type=int),
        "fps": ParameterValue(fps, value_type=float),
        "stereo_color_correction": ParameterValue(
            stereo_color_correction, value_type=bool
        ),
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument("width", default_value="640"),
            DeclareLaunchArgument("height", default_value="480"),
            DeclareLaunchArgument("fps", default_value="30.0"),
            DeclareLaunchArgument("front_sensor_id", default_value="1"),
            DeclareLaunchArgument("back_sensor_id", default_value="0"),
            DeclareLaunchArgument("left_topic", default_value="/stereo/front/image_raw"),
            DeclareLaunchArgument("right_topic", default_value="/stereo/back/image_raw"),
            DeclareLaunchArgument("stereo_color_correction", default_value="true"),
            Node(
                package="falcon_intercept_camera",
                executable="argus_camera_node",
                name="stereo_front_camera_node",
                output="screen",
                parameters=[
                    {
                        **common,
                        "sensor_id": ParameterValue(front_sensor_id, value_type=int),
                        "image_topic": left_topic,
                        "frame_id": "stereo_front_camera",
                    }
                ],
            ),
            Node(
                package="falcon_intercept_camera",
                executable="argus_camera_node",
                name="stereo_back_camera_node",
                output="screen",
                parameters=[
                    {
                        **common,
                        "sensor_id": ParameterValue(back_sensor_id, value_type=int),
                        "image_topic": right_topic,
                        "frame_id": "stereo_back_camera",
                    }
                ],
            ),
        ]
    )
