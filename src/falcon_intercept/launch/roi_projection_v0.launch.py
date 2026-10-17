from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "calibration_path",
                default_value=[
                    FindPackageShare("falcon_intercept"),
                    "/config/camera/stereo_calib.yaml",
                ],
            ),
            DeclareLaunchArgument(
                "extrinsics_path",
                default_value=[
                    FindPackageShare("falcon_intercept"),
                    "/config/camera/extrinsics.yaml",
                ],
            ),
            DeclareLaunchArgument(
                "search_detections_topic", default_value="/search/detections_json"
            ),
            DeclareLaunchArgument("rois_topic", default_value="/stereo/rois_json"),
            DeclareLaunchArgument("min_confidence", default_value="0.5"),
            DeclareLaunchArgument("class_id", default_value="-1"),
            DeclareLaunchArgument("class_name", default_value=""),
            DeclareLaunchArgument("depth_min_m", default_value="5.0"),
            DeclareLaunchArgument("depth_max_m", default_value="100.0"),
            DeclareLaunchArgument("margin_px", default_value="32.0"),
            DeclareLaunchArgument("max_source_age_s", default_value="0.20"),
            DeclareLaunchArgument("max_future_skew_s", default_value="0.05"),
            Node(
                package="falcon_intercept",
                executable="stereo_roi_node",
                name="stereo_roi_node",
                output="screen",
                parameters=[
                    {
                        "calibration_path": LaunchConfiguration("calibration_path"),
                        "extrinsics_path": LaunchConfiguration("extrinsics_path"),
                        "search_detections_topic": LaunchConfiguration("search_detections_topic"),
                        "rois_topic": LaunchConfiguration("rois_topic"),
                        "min_confidence": ParameterValue(
                            LaunchConfiguration("min_confidence"), value_type=float
                        ),
                        "class_id": ParameterValue(
                            LaunchConfiguration("class_id"), value_type=int
                        ),
                        "class_name": LaunchConfiguration("class_name"),
                        "depth_min_m": ParameterValue(
                            LaunchConfiguration("depth_min_m"), value_type=float
                        ),
                        "depth_max_m": ParameterValue(
                            LaunchConfiguration("depth_max_m"), value_type=float
                        ),
                        "margin_px": ParameterValue(
                            LaunchConfiguration("margin_px"), value_type=float
                        ),
                        "max_source_age_s": ParameterValue(
                            LaunchConfiguration("max_source_age_s"), value_type=float
                        ),
                        "max_future_skew_s": ParameterValue(
                            LaunchConfiguration("max_future_skew_s"), value_type=float
                        ),
                    }
                ],
            ),
        ]
    )
