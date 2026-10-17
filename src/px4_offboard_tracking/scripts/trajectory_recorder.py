#!/usr/bin/env python3
"""Record common-NED aircraft states and upward-camera observations to CSV."""

from __future__ import annotations

import csv
import math
from datetime import datetime
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleAttitude
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from px4_offboard_tracking.msg import (
    AlignmentFeasibility,
    CaptureWindowStatus,
    MissionStatus,
    RelativeEstimate,
    RelativeKinematics,
    SafetyStatus,
    TargetObservation,
    TrackingQuality,
)


CORE_FIELDS = (
    "time_s",
    "target_n_m",
    "target_e_m",
    "target_d_m",
    "target_vn_mps",
    "target_ve_mps",
    "target_vd_mps",
    "target_q_w",
    "target_q_x",
    "target_q_y",
    "target_q_z",
    "pursuer_n_m",
    "pursuer_e_m",
    "pursuer_d_m",
    "pursuer_vn_mps",
    "pursuer_ve_mps",
    "pursuer_vd_mps",
    "pursuer_q_w",
    "pursuer_q_x",
    "pursuer_q_y",
    "pursuer_q_z",
    "pursuer_yaw_rad",
    "pursuer_body_x_n",
    "pursuer_body_x_e",
    "pursuer_body_x_d",
    "pursuer_body_minus_z_n",
    "pursuer_body_minus_z_e",
    "pursuer_body_minus_z_d",
    "camera_x_m",
    "camera_y_m",
    "camera_z_m",
    "camera_u_px",
    "camera_v_px",
    "camera_depth_m",
    "camera_visible",
    "camera_outlier_injected",
    "estimate_valid",
    "estimate_n_m",
    "estimate_e_m",
    "estimate_d_m",
    "estimate_vn_mps",
    "estimate_ve_mps",
    "estimate_vd_mps",
    "estimate_innovation_n_m",
    "estimate_innovation_e_m",
    "estimate_innovation_d_m",
    "estimate_innovation_mahalanobis",
    "estimate_data_age_s",
)

ESTIMATE_COVARIANCE_FIELDS = tuple(
    f"estimate_covariance_{row}{column}" for row in range(6) for column in range(6)
)

DIAGNOSTIC_FIELDS = (
    "kinematics_valid",
    "kinematics_range_m",
    "kinematics_relative_vn_mps",
    "kinematics_relative_ve_mps",
    "kinematics_relative_vd_mps",
    "kinematics_closing_speed_mps",
    "kinematics_los_rate_n_rad_s",
    "kinematics_los_rate_e_rad_s",
    "kinematics_los_rate_d_rad_s",
    "kinematics_tcpa_s",
    "kinematics_predicted_miss_distance_m",
    "alignment_feasible",
    "alignment_commanded_tilt_rad",
    "alignment_commanded_thrust",
    "alignment_thrust_margin",
    "alignment_body_rate_rad_s",
    "alignment_altitude_loss_m",
    "alignment_range_m",
    "alignment_duration_s",
    "mission_state",
    "capture_count",
    "mission_complete",
    "quality_passed",
    "quality_covariance_ok",
    "quality_innovation_ok",
    "quality_visibility_ok",
    "quality_timestamp_ok",
    "quality_attitude_ok",
    "quality_alignment_ok",
    "capture_instantaneous_pass",
    "capture_held",
    "capture_pixel_ok",
    "capture_alignment_ok",
    "capture_range_ok",
    "capture_los_rate_ok",
    "capture_relative_speed_ok",
    "capture_prediction_ok",
    "capture_frame_count_ok",
    "capture_distinct_observation_frames",
    "capture_pixel_error_px",
    "capture_alignment_error_rad",
    "capture_los_rate_rad_s",
    "capture_relative_speed_mps",
    "capture_hold_elapsed_s",
    "safety_abort",
    "recovery_complete",
)

FIELDS = CORE_FIELDS + ESTIMATE_COVARIANCE_FIELDS + DIAGNOSTIC_FIELDS


class TrajectoryRecorder(Node):
    def __init__(self) -> None:
        super().__init__("trajectory_recorder")
        target_topic = self.declare_parameter(
            "target_state_topic", "/tracking/target/state"
        ).value
        relative_topic = self.declare_parameter(
            "relative_state_topic", "/tracking/relative/state"
        ).value
        observation_topic = self.declare_parameter(
            "observation_topic", "/tracking/camera/observation"
        ).value
        estimate_topic = self.declare_parameter(
            "estimate_topic", "/tracking/relative/estimate"
        ).value
        kinematics_topic = self.declare_parameter(
            "kinematics_topic", "/tracking/relative/kinematics"
        ).value
        alignment_topic = self.declare_parameter(
            "alignment_topic", "/tracking/alignment/feasibility"
        ).value
        mission_topic = self.declare_parameter(
            "mission_topic", "/tracking/mission/status"
        ).value
        quality_topic = self.declare_parameter(
            "quality_topic", "/tracking/quality_gate"
        ).value
        capture_topic = self.declare_parameter(
            "capture_topic", "/tracking/capture_window"
        ).value
        safety_topic = self.declare_parameter(
            "safety_topic", "/tracking/safety/status"
        ).value
        attitude_topic = self.declare_parameter(
            "pursuer_attitude_topic", "/px4_1/fmu/out/vehicle_attitude"
        ).value
        output_directory_value = str(
            self.declare_parameter("output_directory", "").value
        )
        output_directory = (
            Path(output_directory_value).expanduser()
            if output_directory_value
            else Path.home() / "UAV_ws" / "flight_records"
        )
        filename = str(self.declare_parameter("filename", "").value)
        record_rate_hz = float(self.declare_parameter("record_rate_hz", 50.0).value)
        self.max_state_age_s = float(self.declare_parameter("max_state_age_s", 0.5).value)
        if record_rate_hz <= 0.0 or self.max_state_age_s <= 0.0:
            raise ValueError("record rate and maximum state age must be positive")

        output_directory.mkdir(parents=True, exist_ok=True)
        if not filename:
            filename = f"tracking_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.output_path = output_directory / filename
        self.stream = self.output_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.stream, fieldnames=FIELDS)
        self.writer.writeheader()
        self.stream.flush()

        self.target = None
        self.target_receipt_ns = 0
        self.relative = None
        self.relative_receipt_ns = 0
        self.observation = None
        self.observation_receipt_ns = 0
        self.estimate_valid = False
        self.estimate = None
        self.kinematics = None
        self.alignment = None
        self.mission = None
        self.quality = None
        self.capture = None
        self.safety = None
        self.pursuer_yaw_rad = math.nan
        self.pursuer_q = (math.nan,) * 4
        self.pursuer_body_x_ned = (math.nan,) * 3
        self.pursuer_body_minus_z_ned = (math.nan,) * 3
        self.attitude_receipt_ns = 0

        self.create_subscription(Odometry, target_topic, self._on_target, 10)
        self.create_subscription(Odometry, relative_topic, self._on_relative, 10)
        self.create_subscription(TargetObservation, observation_topic, self._on_observation, 10)
        self.create_subscription(RelativeEstimate, estimate_topic, self._on_estimate, 10)
        self.create_subscription(
            RelativeKinematics, kinematics_topic, self._on_kinematics, 10
        )
        self.create_subscription(
            AlignmentFeasibility, alignment_topic, self._on_alignment, 10
        )
        self.create_subscription(MissionStatus, mission_topic, self._on_mission, 10)
        self.create_subscription(TrackingQuality, quality_topic, self._on_quality, 10)
        self.create_subscription(CaptureWindowStatus, capture_topic, self._on_capture, 10)
        self.create_subscription(SafetyStatus, safety_topic, self._on_safety, 10)
        self.create_subscription(
            VehicleAttitude,
            attitude_topic,
            self._on_pursuer_attitude,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(1.0 / record_rate_hz, self._record)
        self.get_logger().info(f"recording tracking data to {self.output_path}")

    def _receipt_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _on_target(self, message: Odometry) -> None:
        self.target = message
        self.target_receipt_ns = self._receipt_ns()

    def _on_relative(self, message: Odometry) -> None:
        self.relative = message
        self.relative_receipt_ns = self._receipt_ns()

    def _on_observation(self, message: TargetObservation) -> None:
        self.observation = message
        self.observation_receipt_ns = self._receipt_ns()

    def _on_estimate(self, message: RelativeEstimate) -> None:
        self.estimate = message
        self.estimate_valid = bool(message.valid)

    def _on_kinematics(self, message: RelativeKinematics) -> None:
        self.kinematics = message

    def _on_alignment(self, message: AlignmentFeasibility) -> None:
        self.alignment = message

    def _on_mission(self, message: MissionStatus) -> None:
        self.mission = message

    def _on_quality(self, message: TrackingQuality) -> None:
        self.quality = message

    def _on_capture(self, message: CaptureWindowStatus) -> None:
        self.capture = message

    def _on_safety(self, message: SafetyStatus) -> None:
        self.safety = message

    def _on_pursuer_attitude(self, message: VehicleAttitude) -> None:
        w, x, y, z = (float(value) for value in message.q)
        norm = math.sqrt(w * w + x * x + y * y + z * z)
        if not math.isfinite(norm) or norm <= 1e-9:
            return
        w, x, y, z = (value / norm for value in (w, x, y, z))
        self.pursuer_q = (w, x, y, z)
        self.pursuer_yaw_rad = math.atan2(
            2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)
        )
        self.pursuer_body_x_ned = (
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y + w * z),
            2.0 * (x * z - w * y),
        )
        self.pursuer_body_minus_z_ned = (
            -2.0 * (x * z + w * y),
            -2.0 * (y * z - w * x),
            -(1.0 - 2.0 * (x * x + y * y)),
        )
        self.attitude_receipt_ns = self._receipt_ns()

    def _fresh(self, receipt_ns: int, now_ns: int) -> bool:
        return receipt_ns > 0 and 0 <= now_ns - receipt_ns <= self.max_state_age_s * 1e9

    def _record(self) -> None:
        now_ns = self._receipt_ns()
        nan = math.nan
        row = {name: nan for name in FIELDS}
        row["time_s"] = now_ns * 1e-9
        target_position = (nan, nan, nan)
        target_velocity = (nan, nan, nan)
        if self.target is not None and self._fresh(self.target_receipt_ns, now_ns):
            target_position = (
                self.target.pose.pose.position.x,
                self.target.pose.pose.position.y,
                self.target.pose.pose.position.z,
            )
            target_velocity = (
                self.target.twist.twist.linear.x,
                self.target.twist.twist.linear.y,
                self.target.twist.twist.linear.z,
            )
            target_q = self.target.pose.pose.orientation
            row.update(
                {
                    "target_n_m": target_position[0],
                    "target_e_m": target_position[1],
                    "target_d_m": target_position[2],
                    "target_vn_mps": target_velocity[0],
                    "target_ve_mps": target_velocity[1],
                    "target_vd_mps": target_velocity[2],
                    "target_q_w": target_q.w,
                    "target_q_x": target_q.x,
                    "target_q_y": target_q.y,
                    "target_q_z": target_q.z,
                }
            )
        if (
            self.relative is not None
            and self._fresh(self.relative_receipt_ns, now_ns)
            and all(math.isfinite(value) for value in target_position + target_velocity)
        ):
            relative_position = self.relative.pose.pose.position
            relative_velocity = self.relative.twist.twist.linear
            pursuer_position = (
                target_position[0] - relative_position.x,
                target_position[1] - relative_position.y,
                target_position[2] - relative_position.z,
            )
            pursuer_velocity = (
                target_velocity[0] - relative_velocity.x,
                target_velocity[1] - relative_velocity.y,
                target_velocity[2] - relative_velocity.z,
            )
            row.update(
                {
                    "pursuer_n_m": pursuer_position[0],
                    "pursuer_e_m": pursuer_position[1],
                    "pursuer_d_m": pursuer_position[2],
                    "pursuer_vn_mps": pursuer_velocity[0],
                    "pursuer_ve_mps": pursuer_velocity[1],
                    "pursuer_vd_mps": pursuer_velocity[2],
                }
            )

        if self._fresh(self.attitude_receipt_ns, now_ns):
            row.update(
                {
                    "pursuer_q_w": self.pursuer_q[0],
                    "pursuer_q_x": self.pursuer_q[1],
                    "pursuer_q_y": self.pursuer_q[2],
                    "pursuer_q_z": self.pursuer_q[3],
                    "pursuer_yaw_rad": self.pursuer_yaw_rad,
                    "pursuer_body_x_n": self.pursuer_body_x_ned[0],
                    "pursuer_body_x_e": self.pursuer_body_x_ned[1],
                    "pursuer_body_x_d": self.pursuer_body_x_ned[2],
                    "pursuer_body_minus_z_n": self.pursuer_body_minus_z_ned[0],
                    "pursuer_body_minus_z_e": self.pursuer_body_minus_z_ned[1],
                    "pursuer_body_minus_z_d": self.pursuer_body_minus_z_ned[2],
                }
            )

        row["camera_visible"] = 0
        row["camera_outlier_injected"] = 0
        if self.observation is not None and self._fresh(self.observation_receipt_ns, now_ns):
            observation = self.observation
            row.update(
                {
                    "camera_x_m": observation.position_camera.x,
                    "camera_y_m": observation.position_camera.y,
                    "camera_z_m": observation.position_camera.z,
                    "camera_u_px": observation.pixel_u,
                    "camera_v_px": observation.pixel_v,
                    "camera_depth_m": observation.depth,
                    "camera_visible": int(observation.visible),
                    "camera_outlier_injected": int(observation.outlier_injected),
                }
            )

        row["estimate_valid"] = int(self.estimate_valid)
        if self.estimate is not None:
            estimate = self.estimate
            row.update(
                {
                    "estimate_n_m": estimate.position.x,
                    "estimate_e_m": estimate.position.y,
                    "estimate_d_m": estimate.position.z,
                    "estimate_vn_mps": estimate.velocity.x,
                    "estimate_ve_mps": estimate.velocity.y,
                    "estimate_vd_mps": estimate.velocity.z,
                    "estimate_innovation_n_m": estimate.innovation[0],
                    "estimate_innovation_e_m": estimate.innovation[1],
                    "estimate_innovation_d_m": estimate.innovation[2],
                    "estimate_innovation_mahalanobis": estimate.innovation_mahalanobis,
                    "estimate_data_age_s": estimate.data_age,
                }
            )
            for index, value in enumerate(estimate.covariance):
                row[ESTIMATE_COVARIANCE_FIELDS[index]] = value

        row["kinematics_valid"] = 0
        if self.kinematics is not None:
            kinematics = self.kinematics
            row.update(
                {
                    "kinematics_valid": int(kinematics.valid),
                    "kinematics_range_m": kinematics.range,
                    "kinematics_relative_vn_mps": kinematics.relative_velocity.x,
                    "kinematics_relative_ve_mps": kinematics.relative_velocity.y,
                    "kinematics_relative_vd_mps": kinematics.relative_velocity.z,
                    "kinematics_closing_speed_mps": kinematics.closing_speed,
                    "kinematics_los_rate_n_rad_s": kinematics.los_rate.x,
                    "kinematics_los_rate_e_rad_s": kinematics.los_rate.y,
                    "kinematics_los_rate_d_rad_s": kinematics.los_rate.z,
                    "kinematics_tcpa_s": kinematics.time_to_closest_approach,
                    "kinematics_predicted_miss_distance_m":
                        kinematics.predicted_miss_distance,
                }
            )

        row["alignment_feasible"] = 0
        if self.alignment is not None:
            alignment = self.alignment
            row.update(
                {
                    "alignment_feasible": int(alignment.feasible),
                    "alignment_commanded_tilt_rad": alignment.commanded_tilt,
                    "alignment_commanded_thrust": alignment.commanded_thrust,
                    "alignment_thrust_margin": alignment.thrust_margin,
                    "alignment_body_rate_rad_s": alignment.body_rate,
                    "alignment_altitude_loss_m": alignment.altitude_loss,
                    "alignment_range_m": alignment.range,
                    "alignment_duration_s": alignment.alignment_duration,
                }
            )

        if self.mission is not None:
            row.update(
                {
                    "mission_state": self.mission.state_name,
                    "capture_count": self.mission.capture_count,
                    "mission_complete": int(self.mission.mission_complete),
                }
            )
        if self.quality is not None:
            for field in (
                "passed", "covariance_ok", "innovation_ok", "visibility_ok",
                "timestamp_ok", "attitude_ok", "alignment_ok",
            ):
                row[f"quality_{field}"] = int(getattr(self.quality, field))
        if self.capture is not None:
            capture_mapping = {
                "instantaneous_pass": "instantaneous_pass",
                "held": "held",
                "pixel_ok": "pixel_ok",
                "alignment_ok": "alignment_ok",
                "range_ok": "range_ok",
                "los_rate_ok": "los_rate_ok",
                "relative_speed_ok": "relative_speed_ok",
                "prediction_ok": "prediction_ok",
                "frame_count_ok": "frame_count_ok",
            }
            for column_suffix, attribute in capture_mapping.items():
                row[f"capture_{column_suffix}"] = int(getattr(self.capture, attribute))
            row.update(
                {
                    "capture_distinct_observation_frames":
                        self.capture.distinct_observation_frames,
                    "capture_pixel_error_px": self.capture.pixel_error,
                    "capture_alignment_error_rad": self.capture.alignment_error,
                    "capture_los_rate_rad_s": self.capture.los_rate,
                    "capture_relative_speed_mps": self.capture.relative_speed,
                    "capture_hold_elapsed_s": self.capture.hold_elapsed,
                }
            )
        if self.safety is not None:
            row["safety_abort"] = int(self.safety.abort_requested)
            row["recovery_complete"] = int(self.safety.recovery_complete)

        self.writer.writerow(row)
        self.stream.flush()

    def destroy_node(self) -> bool:
        if not self.stream.closed:
            self.stream.flush()
            self.stream.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = TrajectoryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as error:
        # launch/timeout can tear down DDS readers while the executor is taking a sample.
        if "Unable to convert call argument" not in str(error):
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
