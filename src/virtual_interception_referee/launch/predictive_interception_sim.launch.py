from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    tracking_launch = (
        Path(get_package_share_directory("px4_offboard_tracking"))
        / "launch"
        / "pure_interceptor_sim.launch.py"
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(tracking_launch))
            ),
            Node(
                package="virtual_interception_referee",
                executable="virtual_projectile_referee",
                namespace="evaluation",
                name="virtual_projectile_referee",
                output="screen",
                parameters=[
                    {
                        "shot_topic": "/tracking/virtual_shot_event",
                        "result_topic": "/evaluation/virtual_hit_result",
                        "solver_result_topic":
                            "/evaluation/virtual_solver_hit_result",
                        "truth_topic": "/evaluation/relative_truth",
                        "task_origin_topic": "/tracking/task_origin_ned",
                        "pursuer_local_topic":
                            "/px4_1/fmu/out/vehicle_local_position",
                        "pursuer_attitude_topic":
                            "/px4_1/fmu/out/vehicle_attitude",
                        "projectile_speed_mps": 400.0,
                        "hit_radius_m": 0.4,
                        "effect_reference_range_m": 35.0,
                        "effect_reference_diameter_m": 1.7,
                    }
                ],
            ),
        ]
    )
