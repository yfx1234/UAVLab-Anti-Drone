from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Field deployments pass the live vehicle namespace; the defaults
            # preserve the historical SITL (/px4_1) wiring.
            DeclareLaunchArgument(
                "pursuer_local_topic",
                default_value="/px4_1/fmu/out/vehicle_local_position",
            ),
            DeclareLaunchArgument(
                "pursuer_attitude_topic",
                default_value="/px4_1/fmu/out/vehicle_attitude",
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
                        "pursuer_local_topic": LaunchConfiguration(
                            "pursuer_local_topic"
                        ),
                        "pursuer_attitude_topic": LaunchConfiguration(
                            "pursuer_attitude_topic"
                        ),
                        "projectile_speed_mps": 400.0,
                        "hit_radius_m": 0.4,
                        "effect_reference_range_m": 35.0,
                        "effect_reference_diameter_m": 1.7,
                    }
                ],
            )
        ]
    )
