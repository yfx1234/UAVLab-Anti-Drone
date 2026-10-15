from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    search_device_path = LaunchConfiguration("search_device_path")
    search_width = LaunchConfiguration("search_width")
    search_height = LaunchConfiguration("search_height")
    search_fps = LaunchConfiguration("search_fps")
    search_source_width = LaunchConfiguration("search_source_width")
    search_source_height = LaunchConfiguration("search_source_height")
    search_source_fps = LaunchConfiguration("search_source_fps")
    search_sun_filter = LaunchConfiguration("search_sun_filter")
    search_exposure_time = LaunchConfiguration("search_exposure_time")
    search_gain = LaunchConfiguration("search_gain")
    search_brightness = LaunchConfiguration("search_brightness")
    search_backlight_compensation = LaunchConfiguration("search_backlight_compensation")
    stereo_width = LaunchConfiguration("stereo_width")
    stereo_height = LaunchConfiguration("stereo_height")
    stereo_fps = LaunchConfiguration("stereo_fps")
    stereo_source_width = LaunchConfiguration("stereo_source_width")
    stereo_source_height = LaunchConfiguration("stereo_source_height")
    stereo_source_fps = LaunchConfiguration("stereo_source_fps")
    front_sensor_id = LaunchConfiguration("front_sensor_id")
    back_sensor_id = LaunchConfiguration("back_sensor_id")
    model_path = LaunchConfiguration("model_path")
    confidence = LaunchConfiguration("confidence")
    iou = LaunchConfiguration("iou")
    imgsz = LaunchConfiguration("imgsz")
    device = LaunchConfiguration("device")
    half = LaunchConfiguration("half")
    max_det = LaunchConfiguration("max_det")
    classes = LaunchConfiguration("classes")
    frame_stride = LaunchConfiguration("frame_stride")
    detector_backend = LaunchConfiguration("detector_backend")
    publish_debug_image = LaunchConfiguration("publish_debug_image")
    publish_vision_msgs = LaunchConfiguration("publish_vision_msgs")
    enable_search = LaunchConfiguration("enable_search")
    enable_stereo_front = LaunchConfiguration("enable_stereo_front")
    enable_stereo_back = LaunchConfiguration("enable_stereo_back")
    stereo_color_correction = LaunchConfiguration("stereo_color_correction")

    common_detector = {
        "input_mode": "ros_image",
        "detector_backend": detector_backend,
        "model_path": model_path,
        "confidence": ParameterValue(confidence, value_type=float),
        "iou": ParameterValue(iou, value_type=float),
        "imgsz": ParameterValue(imgsz, value_type=int),
        "device": device,
        "half": ParameterValue(half, value_type=bool),
        "max_det": ParameterValue(max_det, value_type=int),
        "classes": classes,
        "frame_stride": ParameterValue(frame_stride, value_type=int),
        "publish_debug_image": ParameterValue(publish_debug_image, value_type=bool),
        "publish_vision_msgs": ParameterValue(publish_vision_msgs, value_type=bool),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "search_device_path",
                default_value=(
                    "/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._HJ_4K_Camera_SN0001-video-index0"
                ),
            ),
            DeclareLaunchArgument("search_width", default_value="640"),
            DeclareLaunchArgument("search_height", default_value="480"),
            DeclareLaunchArgument("search_fps", default_value="25.0"),
            # YOLO profile: use the USB camera's native low-latency mode. The
            # 2560x1440@25 profile remains the recording/archive baseline.
            DeclareLaunchArgument("search_source_width", default_value="640"),
            DeclareLaunchArgument("search_source_height", default_value="480"),
            DeclareLaunchArgument("search_source_fps", default_value="25.0"),
            DeclareLaunchArgument("search_sun_filter", default_value="true"),
            DeclareLaunchArgument("search_exposure_time", default_value="5"),
            DeclareLaunchArgument("search_gain", default_value="0"),
            DeclareLaunchArgument("search_brightness", default_value="-8"),
            DeclareLaunchArgument("search_backlight_compensation", default_value="0"),
            DeclareLaunchArgument("stereo_width", default_value="640"),
            DeclareLaunchArgument("stereo_height", default_value="480"),
            DeclareLaunchArgument("stereo_fps", default_value="30.0"),
            DeclareLaunchArgument("stereo_source_width", default_value="3840"),
            DeclareLaunchArgument("stereo_source_height", default_value="2160"),
            DeclareLaunchArgument("stereo_source_fps", default_value="30.0"),
            DeclareLaunchArgument("front_sensor_id", default_value="1"),
            DeclareLaunchArgument("back_sensor_id", default_value="0"),
            DeclareLaunchArgument(
                "model_path",
                default_value=[FindPackageShare("falcon_intercept"), "/models/yolo26n.pt"],
            ),
            DeclareLaunchArgument("confidence", default_value="0.50"),
            DeclareLaunchArgument("iou", default_value="0.45"),
            DeclareLaunchArgument("imgsz", default_value="640"),
            DeclareLaunchArgument("device", default_value=""),
            DeclareLaunchArgument("half", default_value="false"),
            DeclareLaunchArgument("max_det", default_value="100"),
            DeclareLaunchArgument("classes", default_value=""),
            DeclareLaunchArgument("frame_stride", default_value="1"),
            DeclareLaunchArgument("detector_backend", default_value="ultralytics"),
            DeclareLaunchArgument("publish_debug_image", default_value="false"),
            DeclareLaunchArgument("publish_vision_msgs", default_value="false"),
            DeclareLaunchArgument("enable_search", default_value="true"),
            DeclareLaunchArgument("enable_stereo_front", default_value="true"),
            DeclareLaunchArgument("enable_stereo_back", default_value="true"),
            DeclareLaunchArgument("stereo_color_correction", default_value="true"),
            Node(
                package="falcon_intercept_camera",
                executable="usb_mjpeg_camera_node",
                name="search_camera_node",
                output="screen",
                condition=IfCondition(enable_search),
                parameters=[
                    {
                        "device_path": search_device_path,
                        "image_topic": "/search/image_raw",
                        "frame_id": "search_camera",
                        "width": ParameterValue(search_width, value_type=int),
                        "height": ParameterValue(search_height, value_type=int),
                        "fps": ParameterValue(search_fps, value_type=float),
                        "source_width": ParameterValue(search_source_width, value_type=int),
                        "source_height": ParameterValue(search_source_height, value_type=int),
                        "source_fps": ParameterValue(search_source_fps, value_type=float),
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
                    }
                ],
            ),
            Node(
                package="falcon_intercept_camera",
                executable="argus_camera_node",
                name="stereo_front_camera_node",
                output="screen",
                condition=IfCondition(enable_stereo_front),
                parameters=[
                    {
                        "sensor_id": ParameterValue(front_sensor_id, value_type=int),
                        "image_topic": "/stereo/front/image_raw",
                        "frame_id": "stereo_front_camera",
                        "width": ParameterValue(stereo_width, value_type=int),
                        "height": ParameterValue(stereo_height, value_type=int),
                        "fps": ParameterValue(stereo_fps, value_type=float),
                        "source_width": ParameterValue(stereo_source_width, value_type=int),
                        "source_height": ParameterValue(stereo_source_height, value_type=int),
                        "source_fps": ParameterValue(stereo_source_fps, value_type=float),
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
                condition=IfCondition(enable_stereo_back),
                parameters=[
                    {
                        "sensor_id": ParameterValue(back_sensor_id, value_type=int),
                        "image_topic": "/stereo/back/image_raw",
                        "frame_id": "stereo_back_camera",
                        "width": ParameterValue(stereo_width, value_type=int),
                        "height": ParameterValue(stereo_height, value_type=int),
                        "fps": ParameterValue(stereo_fps, value_type=float),
                        "source_width": ParameterValue(stereo_source_width, value_type=int),
                        "source_height": ParameterValue(stereo_source_height, value_type=int),
                        "source_fps": ParameterValue(stereo_source_fps, value_type=float),
                        "stereo_color_correction": ParameterValue(
                            stereo_color_correction, value_type=bool
                        ),
                    }
                ],
            ),
            Node(
                package="falcon_intercept",
                executable="detector_node",
                name="search_yolo_detector_node",
                output="screen",
                condition=IfCondition(enable_search),
                parameters=[
                    {
                        **common_detector,
                        "image_topic": "/search/image_raw",
                        "detections_topic": "/search/detections_json",
                        "vision_detections_topic": "/search/detections",
                        "debug_image_topic": "/search/image_yolo",
                    }
                ],
            ),
            Node(
                package="falcon_intercept",
                executable="detector_node",
                name="stereo_front_yolo_detector_node",
                output="screen",
                condition=IfCondition(enable_stereo_front),
                parameters=[
                    {
                        **common_detector,
                        "image_topic": "/stereo/front/image_raw",
                        "detections_topic": "/stereo/front/detections_json",
                        "vision_detections_topic": "/stereo/front/detections",
                        "debug_image_topic": "/stereo/front/image_yolo",
                    }
                ],
            ),
            Node(
                package="falcon_intercept",
                executable="detector_node",
                name="stereo_back_yolo_detector_node",
                output="screen",
                condition=IfCondition(enable_stereo_back),
                parameters=[
                    {
                        **common_detector,
                        "image_topic": "/stereo/back/image_raw",
                        "detections_topic": "/stereo/back/detections_json",
                        "vision_detections_topic": "/stereo/back/detections",
                        "debug_image_topic": "/stereo/back/image_yolo",
                    }
                ],
            ),
        ]
    )
