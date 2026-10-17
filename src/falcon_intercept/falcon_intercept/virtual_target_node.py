from __future__ import annotations

import json
import math
from pathlib import Path

from .virtual_target import (
    body_flu_to_ned,
    build_logical_detection_json,
    project_field_aligned_stereo,
    project_pinhole,
    relative_ned_to_camera_fru,
    sample_trajectory_offset,
)


def main(args=None) -> int:
    import rclpy
    import yaml
    from nav_msgs.msg import Odometry
    from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, VehicleStatus
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Bool, String

    class RealAircraftVirtualTargetNode(Node):
        def __init__(self) -> None:
            super().__init__("real_aircraft_virtual_target")
            self.declare_parameter("calibration_path", "")
            self.declare_parameter("px4_namespace", "/fmu")
            self.declare_parameter("target_scenario", "straight")
            self.declare_parameter("target_speed_mps", 2.0)
            self.declare_parameter("target_radius_m", 20.0)
            self.declare_parameter("target_path_width_m", 8.0)
            self.declare_parameter("target_path_length_m", 40.0)
            self.declare_parameter("target_motion_delay_s", 2.0)
            self.declare_parameter("target_motion_ramp_s", 2.0)
            self.declare_parameter("initial_forward_m", 25.0)
            self.declare_parameter("initial_front_m", 0.0)
            self.declare_parameter("initial_up_m", 0.0)
            self.declare_parameter("front_rate_hz", 30.0)
            self.declare_parameter("back_rate_hz", 30.0)
            self.declare_parameter("back_phase_offset_s", 0.012)
            self.declare_parameter("search_rate_hz", 25.0)
            self.declare_parameter("detection_mode", "all")
            self.declare_parameter("detection_start_delay_s", 0.0)
            self.declare_parameter("detection_dropout_start_s", -1.0)
            self.declare_parameter("detection_dropout_duration_s", 0.0)
            self.declare_parameter("state_timeout_s", 0.5)
            self.declare_parameter("require_armed", True)
            self.declare_parameter("require_offboard", True)
            self.declare_parameter("latch_on_offboard_exit", True)
            self.declare_parameter("camera_translation_body_flu_x_m", 0.124)
            self.declare_parameter("camera_translation_body_flu_y_m", 0.050)
            self.declare_parameter("camera_translation_body_flu_z_m", 0.0)

            calibration_path = str(self.get_parameter("calibration_path").value).strip()
            if not calibration_path or not Path(calibration_path).is_file():
                raise RuntimeError("a valid calibration_path is required")
            with Path(calibration_path).open("r", encoding="utf-8") as stream:
                calibration = yaml.safe_load(stream) or {}
            search = calibration["search"]
            stereo = calibration.get("stereo_front", calibration.get("stereo_left"))
            pair = calibration["stereo_pair"]
            # field_alignment is the legacy empirical correction; rectified-mode
            # production calibrations omit it, in which case an ideal stereo
            # pair (all zeros) is simulated.
            alignment = pair.get("field_alignment", {})
            self._search = {
                key: float(search[key]) for key in ("fx", "fy", "cx", "cy")
            }
            self._search_size = (
                int(search["image_width"]),
                int(search["image_height"]),
            )
            self._stereo = {
                key: float(stereo[key]) for key in ("fx", "fy", "cx", "cy")
            }
            self._stereo_size = (
                int(stereo["image_width"]),
                int(stereo["image_height"]),
            )
            self._baseline = float(pair["baseline_m"])
            self._alignment = {
                "vertical_offset_px": float(
                    alignment.get("vertical_offset_front_minus_back_px", 0.0)
                ),
                "vertical_slope": float(
                    alignment.get("vertical_slope_front_minus_back", 0.0)
                ),
                "vertical_reference_u_px": float(
                    alignment.get("vertical_reference_u_px", 0.0)
                ),
                "horizontal_zero_px": float(
                    alignment.get("horizontal_zero_disparity_offset_px", 0.0)
                ),
                "horizontal_sign": float(
                    alignment.get("horizontal_disparity_sign", 1.0)
                ),
            }
            self._scenario = str(self.get_parameter("target_scenario").value)
            self._speed = max(0.0, float(self.get_parameter("target_speed_mps").value))
            self._radius = max(0.1, float(self.get_parameter("target_radius_m").value))
            self._path_width = max(
                0.1, float(self.get_parameter("target_path_width_m").value)
            )
            self._path_length = max(
                0.1, float(self.get_parameter("target_path_length_m").value)
            )
            self._motion_delay = max(
                0.0, float(self.get_parameter("target_motion_delay_s").value)
            )
            self._motion_ramp = max(
                0.0, float(self.get_parameter("target_motion_ramp_s").value)
            )
            self._initial_flu = (
                float(self.get_parameter("initial_forward_m").value),
                float(self.get_parameter("initial_front_m").value),
                float(self.get_parameter("initial_up_m").value),
            )
            self._camera_translation = tuple(
                float(self.get_parameter(name).value)
                for name in (
                    "camera_translation_body_flu_x_m",
                    "camera_translation_body_flu_y_m",
                    "camera_translation_body_flu_z_m",
                )
            )
            self._periods = {
                "front": 1.0 / max(1.0, float(self.get_parameter("front_rate_hz").value)),
                "back": 1.0
                / max(1.0, float(self.get_parameter("back_rate_hz").value)),
                "search": 1.0
                / max(1.0, float(self.get_parameter("search_rate_hz").value)),
            }
            self._back_phase = max(
                0.0, float(self.get_parameter("back_phase_offset_s").value)
            )
            self._detection_mode = str(
                self.get_parameter("detection_mode").value
            ).strip().lower()
            if self._detection_mode not in ("all", "search_only", "stereo_only"):
                raise ValueError(
                    "detection_mode must be all, search_only, or stereo_only"
                )
            self._detection_start_delay = max(
                0.0, float(self.get_parameter("detection_start_delay_s").value)
            )
            self._dropout_start = float(
                self.get_parameter("detection_dropout_start_s").value
            )
            self._dropout_duration = max(
                0.0,
                float(self.get_parameter("detection_dropout_duration_s").value),
            )
            self._timeout = max(
                0.05, float(self.get_parameter("state_timeout_s").value)
            )
            self._require_armed = bool(self.get_parameter("require_armed").value)
            self._require_offboard = bool(self.get_parameter("require_offboard").value)
            self._latch_on_exit = bool(
                self.get_parameter("latch_on_offboard_exit").value
            )

            namespace = str(self.get_parameter("px4_namespace").value).rstrip("/")
            self._position = None
            self._attitude = None
            self._status = None
            self._position_time = self._attitude_time = self._status_time = -math.inf
            self._spawn_position = None
            self._spawn_q = None
            self._spawn_time = None
            self._offboard_seen = False
            self._stopped = False
            self._reset_counters = None
            self._next_publish = {}
            self._sequences = {"front": 0, "back": 0, "search": 0}

            self.create_subscription(
                VehicleLocalPosition,
                f"{namespace}/out/vehicle_local_position",
                self._on_position,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                VehicleAttitude,
                f"{namespace}/out/vehicle_attitude",
                self._on_attitude,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                VehicleStatus,
                f"{namespace}/out/vehicle_status",
                self._on_status,
                qos_profile_sensor_data,
            )
            self.create_subscription(Bool, "/virtual_target/reset", self._on_reset, 10)
            self._detection_pubs = {
                "front": self.create_publisher(
                    String, "/stereo/front/detections_json", qos_profile_sensor_data
                ),
                "back": self.create_publisher(
                    String, "/stereo/back/detections_json", qos_profile_sensor_data
                ),
                "search": self.create_publisher(
                    String, "/search/detections_json", qos_profile_sensor_data
                ),
            }
            self._truth_pub = self.create_publisher(
                Odometry, "/virtual_target/truth", qos_profile_sensor_data
            )
            self._status_pub = self.create_publisher(
                String, "/virtual_target/status", 10
            )
            self.create_timer(0.005, self._on_timer)
            self.get_logger().warning(
                "test-only real-aircraft virtual target ready; waits for manual "
                "arm/takeoff/Offboard and never writes PX4 input topics"
            )

        def _now(self) -> float:
            return self.get_clock().now().nanoseconds / 1_000_000_000.0

        def _on_position(self, message) -> None:
            if (
                not bool(message.xy_valid)
                or not bool(message.z_valid)
                or not all(math.isfinite(float(v)) for v in (message.x, message.y, message.z))
            ):
                return
            counters = (
                int(message.xy_reset_counter),
                int(message.z_reset_counter),
            )
            if self._reset_counters is not None and self._spawn_position is not None:
                north, east, down = self._spawn_position
                if counters[0] != self._reset_counters[0]:
                    north += float(message.delta_xy[0])
                    east += float(message.delta_xy[1])
                if counters[1] != self._reset_counters[1]:
                    down += float(message.delta_z)
                self._spawn_position = (north, east, down)
            self._reset_counters = counters
            self._position = (float(message.x), float(message.y), float(message.z))
            self._position_time = self._now()

        def _on_attitude(self, message) -> None:
            if len(message.q) < 4:
                return
            q = tuple(float(item) for item in message.q[:4])
            if all(math.isfinite(item) for item in q) and sum(item * item for item in q) > 1e-8:
                self._attitude = q
                self._attitude_time = self._now()

        def _on_status(self, message) -> None:
            previous_offboard = self._offboard()
            self._status = message
            self._status_time = self._now()
            current_offboard = self._offboard()
            if current_offboard:
                self._offboard_seen = True
            elif (
                previous_offboard
                and self._offboard_seen
                and self._latch_on_exit
                and self._armed()
            ):
                self._stopped = True
                self.get_logger().warning(
                    "manual Offboard exit detected; virtual detections latched off"
                )

        def _armed(self) -> bool:
            return self._status is not None and int(self._status.arming_state) == int(
                VehicleStatus.ARMING_STATE_ARMED
            )

        def _offboard(self) -> bool:
            return self._status is not None and int(self._status.nav_state) == int(
                VehicleStatus.NAVIGATION_STATE_OFFBOARD
            )

        def _on_reset(self, message) -> None:
            if bool(message.data) and not self._armed():
                self._spawn_position = self._spawn_q = self._spawn_time = None
                self._offboard_seen = False
                self._stopped = False
                self.get_logger().warning("virtual-target latch reset while disarmed")

        def _fresh_and_gated(self, now: float) -> bool:
            fresh = (
                self._position is not None
                and self._attitude is not None
                and self._status is not None
                and now - self._position_time <= self._timeout
                and now - self._attitude_time <= self._timeout
                and now - self._status_time <= self._timeout
            )
            return (
                fresh
                and not self._stopped
                and (not self._require_armed or self._armed())
                and (not self._require_offboard or self._offboard())
            )

        def _spawn(self, now: float) -> None:
            offset_ned = body_flu_to_ned(self._attitude, self._initial_flu)
            self._spawn_position = tuple(
                self._position[i] + offset_ned[i] for i in range(3)
            )
            self._spawn_q = self._attitude
            self._spawn_time = now
            self._next_publish = {
                "front": now,
                "back": now + self._back_phase,
                "search": now,
            }
            self.get_logger().warning(
                "virtual target spawned %.1f m forward; scenario=%s speed=%.1f m/s"
                % (self._initial_flu[0], self._scenario, self._speed)
            )

        def _target_state(self, now: float):
            elapsed = max(0.0, now - self._spawn_time - self._motion_delay)
            ramp = 1.0 if self._motion_ramp <= 0.0 else min(1.0, elapsed / self._motion_ramp)
            offset, velocity = sample_trajectory_offset(
                self._scenario,
                elapsed,
                self._speed * ramp,
                self._radius,
                self._path_width,
                self._path_length,
            )
            offset_ned = body_flu_to_ned(
                self._spawn_q, (offset[0], -offset[1], -offset[2])
            )
            velocity_ned = body_flu_to_ned(
                self._spawn_q, (velocity[0], -velocity[1], -velocity[2])
            )
            position = tuple(self._spawn_position[i] + offset_ned[i] for i in range(3))
            return position, velocity_ned

        def _publish_truth(self, now: float, position, velocity) -> None:
            message = Odometry()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = "px4_local_ned"
            message.child_frame_id = "virtual_target"
            message.pose.pose.position.x = position[0]
            message.pose.pose.position.y = position[1]
            message.pose.pose.position.z = position[2]
            message.pose.pose.orientation.w = 1.0
            message.twist.twist.linear.x = velocity[0]
            message.twist.twist.linear.y = velocity[1]
            message.twist.twist.linear.z = velocity[2]
            self._truth_pub.publish(message)
            status = String()
            status.data = json.dumps(
                {
                    "stamp_s": now,
                    "active": True,
                    "test_only": True,
                    "scenario": self._scenario,
                    "speed_mps": self._speed,
                },
                separators=(",", ":"),
            )
            self._status_pub.publish(status)

        def _publish_detection(self, stream: str, now: float, camera_point) -> None:
            if stream == "search":
                center = project_pinhole(camera_point, **self._search)
                size = self._search_size
                frame = "search_camera"
            else:
                projection = project_field_aligned_stereo(
                    camera_point,
                    **self._stereo,
                    baseline_m=self._baseline,
                    **self._alignment,
                )
                center = None if projection is None else getattr(projection, stream)
                size = self._stereo_size
                frame = f"stereo_{stream}_camera"
            message = String()
            message.data = build_logical_detection_json(
                frame_id=frame,
                stamp_s=now,
                image_width=size[0],
                image_height=size[1],
                center=center,
                sequence=self._sequences[stream],
            )
            self._sequences[stream] += 1
            self._detection_pubs[stream].publish(message)

        def _stream_enabled(self, stream: str, now: float) -> bool:
            elapsed = now - self._spawn_time
            if elapsed < self._detection_start_delay:
                return False
            if (
                self._dropout_start >= 0.0
                and self._dropout_start
                <= elapsed
                < self._dropout_start + self._dropout_duration
            ):
                return False
            if self._detection_mode == "search_only":
                return stream == "search"
            if self._detection_mode == "stereo_only":
                return stream in ("front", "back")
            return True

        def _on_timer(self) -> None:
            now = self._now()
            if not self._fresh_and_gated(now):
                return
            if self._spawn_time is None:
                self._spawn(now)
            position, velocity = self._target_state(now)
            relative = tuple(position[i] - self._position[i] for i in range(3))
            camera_point = relative_ned_to_camera_fru(
                relative, self._attitude, self._camera_translation
            )
            due = False
            for stream in ("front", "back", "search"):
                if now + 1e-9 >= self._next_publish[stream]:
                    if self._stream_enabled(stream, now):
                        self._publish_detection(stream, now, camera_point)
                        due = True
                    self._next_publish[stream] += self._periods[stream]
                    if self._next_publish[stream] < now - self._periods[stream]:
                        self._next_publish[stream] = now + self._periods[stream]
            if due:
                self._publish_truth(now, position, velocity)

    rclpy.init(args=args)
    node = None
    try:
        node = RealAircraftVirtualTargetNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
