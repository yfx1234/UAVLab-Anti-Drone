"""Search-camera-only visual servo chain (search_servo_v0).

Full-chain fallback mode that skips the stereo depth pipeline entirely:
search detections -> bearing -> servo node -> upstream gate -> PX4 adapter.
Shares the production isolation boundary (upstream_gate_node +
px4_upstream_adapter_node), so arm/takeoff/preheat/takeover gating is
identical to real_strike_v0. A virtual projectile referee can be included
for scoring; its pursuer topics are mapped to the live vehicle namespace.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _truth(value: str) -> bool:
    return str(value).strip().lower() in {"true", "1", "yes"}


def _servo_actions(context):
    authorized = _truth(LaunchConfiguration("field_authorized").perform(context))
    if not authorized:
        raise RuntimeError(
            "SERVO CHAIN REFUSED: pass field_authorized:=true after pilot and field approval"
        )
    adapter_mode = (
        LaunchConfiguration("adapter_mode").perform(context).strip().lower()
    )
    allow_live = _truth(LaunchConfiguration("allow_live_control").perform(context))
    calibration_ready = _truth(
        LaunchConfiguration("calibration_ready").perform(context)
    )
    if adapter_mode == "live" and not (allow_live and calibration_ready):
        raise RuntimeError(
            "live servo mode requires allow_live_control:=true and calibration_ready:=true"
        )
    with_referee = _truth(LaunchConfiguration("with_referee").perform(context))

    nodes = [
        Node(
            package="falcon_intercept",
            executable="search_bearing_node",
            name="search_bearing_node",
            output="screen",
        ),
        Node(
            package="falcon_intercept",
            executable="perception_bridge_node",
            name="perception_bridge_node",
            output="screen",
            parameters=[
                {
                    # The search camera owns the 10 deg noseward boresight in
                    # this mode; no extra tilt on top of the roll correction.
                    "search_canonical_tilt_deg": 0.0,
                }
            ],
        ),
        Node(
            package="falcon_intercept",
            executable="search_servo_node",
            name="search_servo_node",
            output="screen",
            parameters=[
                {
                    "desired_range_m": LaunchConfiguration("desired_range_m"),
                    "reference_range_m": LaunchConfiguration("reference_range_m"),
                    "reference_bbox_px": LaunchConfiguration("reference_bbox_px"),
                    "minimum_height_m": LaunchConfiguration("minimum_height_m"),
                    "max_forward_mps": LaunchConfiguration("max_forward_mps"),
                    "fire_enable": LaunchConfiguration("fire_enable"),
                    "prediction_enabled": LaunchConfiguration("prediction_enabled"),
                    "prediction_control_delay_s": LaunchConfiguration(
                        "prediction_control_delay_s"
                    ),
                    "prediction_trigger_delay_s": LaunchConfiguration(
                        "prediction_trigger_delay_s"
                    ),
                    "prediction_maximum_horizon_s": LaunchConfiguration(
                        "prediction_maximum_horizon_s"
                    ),
                    "prediction_maximum_rate_rad_s": LaunchConfiguration(
                        "prediction_maximum_rate_rad_s"
                    ),
                    "prediction_maximum_offset_rad": LaunchConfiguration(
                        "prediction_maximum_offset_rad"
                    ),
                }
            ],
        ),
        Node(
            package="falcon_intercept",
            executable="upstream_gate_node",
            name="upstream_gate_node",
            output="screen",
        ),
        Node(
            package="falcon_intercept",
            executable="px4_upstream_adapter_node",
            name="px4_upstream_adapter",
            output="screen",
            parameters=[
                {
                    "mode": adapter_mode,
                    "allow_live_control": allow_live,
                    "calibration_ready": calibration_ready,
                    "rate_hz": 50.0,
                }
            ],
        ),
    ]
    if with_referee:
        nodes.append(
            Node(
                package="virtual_interception_referee",
                executable="virtual_projectile_referee",
                namespace="evaluation",
                name="virtual_projectile_referee",
                output="screen",
                parameters=[
                    {
                        "pursuer_local_topic": LaunchConfiguration(
                            "referee_pursuer_local_topic"
                        ),
                        "pursuer_attitude_topic": LaunchConfiguration(
                            "referee_pursuer_attitude_topic"
                        ),
                        # Widened effect cone so the servo's 4 deg fire gate
                        # matches the referee verdict at 5-10 m standoff:
                        # half angle = 0.5*5.0/35 = 4.1 deg (radius ~0.54 m at
                        # 7.5 m). One confirmed hit = mission success.
                        "effect_reference_diameter_m": 5.0,
                        "required_hits": 1,
                    }
                ],
            )
        )
    return [
        LogInfo(msg="SEARCH SERVO CHAIN AUTHORIZED: mode=%s" % adapter_mode)
    ] + nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("field_authorized", default_value="false"),
            DeclareLaunchArgument("adapter_mode", default_value="live"),
            DeclareLaunchArgument("allow_live_control", default_value="true"),
            DeclareLaunchArgument("calibration_ready", default_value="true"),
            DeclareLaunchArgument("with_referee", default_value="true"),
            DeclareLaunchArgument("desired_range_m", default_value="7.5"),
            DeclareLaunchArgument("reference_range_m", default_value="10.0"),
            DeclareLaunchArgument("reference_bbox_px", default_value="55.0"),
            DeclareLaunchArgument("minimum_height_m", default_value="3.0"),
            DeclareLaunchArgument("max_forward_mps", default_value="3.0"),
            DeclareLaunchArgument("fire_enable", default_value="true"),
            DeclareLaunchArgument("prediction_enabled", default_value="true"),
            DeclareLaunchArgument(
                "prediction_control_delay_s", default_value="0.08"
            ),
            DeclareLaunchArgument(
                "prediction_trigger_delay_s", default_value="0.02"
            ),
            DeclareLaunchArgument(
                "prediction_maximum_horizon_s", default_value="0.35"
            ),
            DeclareLaunchArgument(
                "prediction_maximum_rate_rad_s", default_value="0.70"
            ),
            DeclareLaunchArgument(
                "prediction_maximum_offset_rad", default_value="0.12"
            ),
            DeclareLaunchArgument(
                "referee_pursuer_local_topic",
                default_value="/fmu/out/vehicle_local_position",
            ),
            DeclareLaunchArgument(
                "referee_pursuer_attitude_topic",
                default_value="/fmu/out/vehicle_attitude",
            ),
            SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
            OpaqueFunction(function=_servo_actions),
        ]
    )
