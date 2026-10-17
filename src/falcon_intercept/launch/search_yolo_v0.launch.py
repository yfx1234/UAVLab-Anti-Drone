from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    input_mode = LaunchConfiguration("input_mode")
    image_topic = LaunchConfiguration("image_topic")
    detections_topic = LaunchConfiguration("detections_topic")
    device_path = LaunchConfiguration("device_path")
    frame_id = LaunchConfiguration("frame_id")
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    fps = LaunchConfiguration("fps")
    fourcc = LaunchConfiguration("fourcc")
    detector_backend = LaunchConfiguration("detector_backend")
    model_path = LaunchConfiguration("model_path")
    confidence = LaunchConfiguration("confidence")
    iou = LaunchConfiguration("iou")
    imgsz = LaunchConfiguration("imgsz")
    device = LaunchConfiguration("device")
    half = LaunchConfiguration("half")
    max_det = LaunchConfiguration("max_det")
    classes = LaunchConfiguration("classes")
    frame_stride = LaunchConfiguration("frame_stride")
    class_name = LaunchConfiguration("class_name")
    publish_debug_image = LaunchConfiguration("publish_debug_image")
    publish_vision_msgs = LaunchConfiguration("publish_vision_msgs")

    return LaunchDescription(
        [
            DeclareLaunchArgument("input_mode", default_value="direct_camera"),
            DeclareLaunchArgument("image_topic", default_value="/search/image_raw"),
            DeclareLaunchArgument("detections_topic", default_value="/search/detections_json"),
            DeclareLaunchArgument("device_path", default_value="/dev/video0"),
            DeclareLaunchArgument("frame_id", default_value="search_camera"),
            DeclareLaunchArgument("width", default_value="640"),
            DeclareLaunchArgument("height", default_value="480"),
            DeclareLaunchArgument("fps", default_value="10.0"),
            DeclareLaunchArgument("fourcc", default_value="MJPG"),
            DeclareLaunchArgument("detector_backend", default_value="empty"),
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
            DeclareLaunchArgument("class_name", default_value=""),
            DeclareLaunchArgument("publish_debug_image", default_value="false"),
            DeclareLaunchArgument("publish_vision_msgs", default_value="false"),
            Node(
                package="falcon_intercept",
                executable="detector_node",
                name="search_yolo_detector_node",
                output="screen",
                parameters=[
                    {
                        "input_mode": input_mode,
                        "image_topic": image_topic,
                        "detections_topic": detections_topic,
                        "device_path": device_path,
                        "frame_id": frame_id,
                        "width": ParameterValue(width, value_type=int),
                        "height": ParameterValue(height, value_type=int),
                        "fps": ParameterValue(fps, value_type=float),
                        "fourcc": fourcc,
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
                        "class_name": class_name,
                        "publish_debug_image": ParameterValue(publish_debug_image, value_type=bool),
                        "publish_vision_msgs": ParameterValue(publish_vision_msgs, value_type=bool),
                    }
                ],
            ),
        ]
    )
