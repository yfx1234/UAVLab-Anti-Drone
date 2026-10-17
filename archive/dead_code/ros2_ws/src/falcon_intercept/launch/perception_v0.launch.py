from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="falcon_intercept",
                executable="stereo_target_solver_node",
                name="stereo_target_solver_node",
                output="screen",
                parameters=[
                    {
                        "calibration_path": PathJoinSubstitution(
                            [
                                FindPackageShare("falcon_intercept"),
                                "config",
                                "camera",
                                "stereo_calib.yaml",
                            ]
                        )
                    }
                ],
            ),
            Node(
                package="falcon_intercept",
                executable="target_kf_node",
                name="target_kf_node",
                output="screen",
            ),
            Node(
                package="falcon_intercept",
                executable="search_bearing_node",
                name="search_bearing_node",
                output="screen",
            ),
        ]
    )
