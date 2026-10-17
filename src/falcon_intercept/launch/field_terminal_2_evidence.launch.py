"""Field terminal 2: timestamped evidence bag for the real closed loop."""

from datetime import datetime

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable


TOPICS = [
    "/search/detections_json",
    "/stereo/front/detections_json",
    "/stereo/back/detections_json",
    "/perception/search/bearing",
    "/perception/async_stereo_measurement",
    "/perception/target/camera_position",
    "/perception/search/observation",
    "/perception/depth/observation",
    "/tracking/relative/estimate",
    "/tracking/pure_interceptor/status",
    "/tracking/virtual_shot_event",
    "/tracking/mission/complete",
    "/tracking/offboard_session_started",
    "/intercept/upstream_px4_shadow",
    "/upstream/fmu/in/offboard_control_mode",
    "/upstream/fmu/in/trajectory_setpoint",
    "/upstream/fmu/in/vehicle_attitude_setpoint",
    "/fmu/in/offboard_control_mode",
    "/fmu/in/trajectory_setpoint",
    "/fmu/in/vehicle_attitude_setpoint",
    "/fmu/out/vehicle_status",
    "/fmu/out/vehicle_attitude",
    "/fmu/out/vehicle_local_position",
    "/fmu/out/vehicle_land_detected",
    # Search-servo mode (2026-09-06): servo state, referee verdicts, and the
    # servo's target estimate used as referee truth.
    "/tracking/servo/status",
    "/evaluation/virtual_hit_result",
    "/evaluation/mission_success",
    "/evaluation/relative_truth",
]


def generate_launch_description():
    output = "bags/real_closed_loop_live_" + datetime.now().strftime("%Y%m%d_%H%M%S")
    return LaunchDescription(
        [
            SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
            ExecuteProcess(
                cmd=["ros2", "bag", "record", "-o", output, *TOPICS],
                output="screen",
            ),
        ]
    )
