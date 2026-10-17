from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    search_device_path = LaunchConfiguration("search_device_path")
    stereo_width = LaunchConfiguration("stereo_width")
    stereo_height = LaunchConfiguration("stereo_height")
    stereo_fps = LaunchConfiguration("stereo_fps")
    front_sensor_id = LaunchConfiguration("front_sensor_id")
    back_sensor_id = LaunchConfiguration("back_sensor_id")
    search_width = LaunchConfiguration("search_width")
    search_height = LaunchConfiguration("search_height")
    search_fps = LaunchConfiguration("search_fps")
    search_sun_filter = LaunchConfiguration("search_sun_filter")
    search_exposure_time = LaunchConfiguration("search_exposure_time")
    search_gain = LaunchConfiguration("search_gain")
    search_brightness = LaunchConfiguration("search_brightness")
    search_backlight_compensation = LaunchConfiguration("search_backlight_compensation")
    stereo_color_correction = LaunchConfiguration("stereo_color_correction")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "search_device_path",
                default_value=(
                    "/dev/v4l/by-id/"
                    "usb-Sonix_Technology_Co.__Ltd._HJ_4K_Camera_SN0001-video-index0"
                ),
            ),
            # Hardware/source validation defaults. The YOLO launch keeps a separate
            # 640x480 inference profile until Jetson GPU throughput is measured.
            DeclareLaunchArgument("search_width", default_value="2560"),
            DeclareLaunchArgument("search_height", default_value="1440"),
            DeclareLaunchArgument("search_fps", default_value="25.0"),
            DeclareLaunchArgument("search_sun_filter", default_value="true"),
            DeclareLaunchArgument("search_exposure_time", default_value="5"),
            DeclareLaunchArgument("search_gain", default_value="0"),
            DeclareLaunchArgument("search_brightness", default_value="-8"),
            DeclareLaunchArgument("search_backlight_compensation", default_value="0"),
            DeclareLaunchArgument("stereo_width", default_value="3840"),
            DeclareLaunchArgument("stereo_height", default_value="2160"),
            DeclareLaunchArgument("stereo_fps", default_value="30.0"),
            DeclareLaunchArgument("front_sensor_id", default_value="1"),
            DeclareLaunchArgument("back_sensor_id", default_value="0"),
            DeclareLaunchArgument("stereo_color_correction", default_value="true"),
            Node(
                package="falcon_intercept_camera",
                executable="usb_mjpeg_camera_node",
                name="search_camera_cpp_node",
                output="screen",
                parameters=[
                    {
                        "device_path": search_device_path,
                        "image_topic": "/search/image_raw",
                        "width": ParameterValue(search_width, value_type=int),
                        "height": ParameterValue(search_height, value_type=int),
                        "fps": ParameterValue(search_fps, value_type=float),
                        "source_width": ParameterValue(search_width, value_type=int),
                        "source_height": ParameterValue(search_height, value_type=int),
                        "source_fps": ParameterValue(search_fps, value_type=float),
                        "search_sun_filter": ParameterValue(
                            search_sun_filter, value_type=bool
                        ),
                        "search_exposure_time": ParameterValue(
                            search_exposure_time, value_type=int
                        ),
                        "search_gain": ParameterValue(search_gain, value_type=int),
                        "search_brightness": ParameterValue(
                            search_brightness, value_type=int
                        ),
                        "search_backlight_compensation": ParameterValue(
                            search_backlight_compensation, value_type=int
                        ),
                        "frame_id": "search_camera",
                    }
                ],
            ),
            Node(
                package="falcon_intercept_camera",
                executable="argus_camera_node",
                name="stereo_front_camera_node",
                output="screen",
                parameters=[
                    {
                        "sensor_id": ParameterValue(front_sensor_id, value_type=int),
                        "image_topic": "/stereo/front/image_raw",
                        "frame_id": "stereo_front_camera",
                        "width": ParameterValue(stereo_width, value_type=int),
                        "height": ParameterValue(stereo_height, value_type=int),
                        "fps": ParameterValue(stereo_fps, value_type=float),
                        "stereo_color_correction": ParameterValue(
                            stereo_color_correction, value_type=bool
                        ),
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
                        "sensor_id": ParameterValue(back_sensor_id, value_type=int),
                        "image_topic": "/stereo/back/image_raw",
                        "frame_id": "stereo_back_camera",
                        "width": ParameterValue(stereo_width, value_type=int),
                        "height": ParameterValue(stereo_height, value_type=int),
                        "fps": ParameterValue(stereo_fps, value_type=float),
                        "stereo_color_correction": ParameterValue(
                            stereo_color_correction, value_type=bool
                        ),
                    }
                ],
            ),
        ]
    )
