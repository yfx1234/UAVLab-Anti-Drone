#!/usr/bin/env python3
"""Publish synthetic PX4 output state to smoke-test the real-target ROS chain.

This is not SITL and never publishes under /fmu/in.  It exists only to exercise
the board-side virtual-camera/perception/decision graph while the flight
controller is unavailable.
"""

import argparse
import time

import rclpy
from falcon_intercept_msgs.msg import (
    CaptureWindowStatus,
    MissionStatus,
    SafetyStatus,
    SearchCommand,
    TargetTrack,
)
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import String


class ChainSmoke(Node):
    def __init__(self, profile):
        super().__init__("virtual_target_chain_smoke")
        self.profile = profile
        self.counts = {
            "front": 0,
            "back": 0,
            "search": 0,
            "track": 0,
            "fire": 0,
            "hit": 0,
        }
        self.maximum_state = MissionStatus.STATE_INIT
        self.seen_states = set()
        self.valid_track_seen = False
        self.search_active_seen = False
        self.search_cued_seen = False
        self.last_track = None
        self.last_mission = None
        self.last_capture = None
        self.last_safety = None
        px4_output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._attitude_pub = self.create_publisher(
            VehicleAttitude, "/fmu/out/vehicle_attitude", px4_output_qos
        )
        self._position_pub = self.create_publisher(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            px4_output_qos,
        )
        self._status_pub = self.create_publisher(
            VehicleStatus, "/fmu/out/vehicle_status", px4_output_qos
        )
        for key, topic in (
            ("front", "/stereo/front/detections_json"),
            ("back", "/stereo/back/detections_json"),
            ("search", "/search/detections_json"),
            ("fire", "/intercept/fire_event"),
            ("hit", "/virtual_target/hit_result"),
        ):
            self.create_subscription(
                String,
                topic,
                lambda _message, stream=key: self._increment(stream),
                qos_profile_sensor_data if key in ("front", "back", "search") else 10,
            )
        self.create_subscription(
            TargetTrack, "/target/track", self._on_track, qos_profile_sensor_data
        )
        self.create_subscription(
            MissionStatus, "/decision/mission_status", self._on_mission, 10
        )
        self.create_subscription(
            SearchCommand, "/decision/search_command", self._on_search, 10
        )
        self.create_subscription(
            CaptureWindowStatus,
            "/decision/capture_window",
            lambda message: setattr(self, "last_capture", message),
            10,
        )
        self.create_subscription(
            SafetyStatus,
            "/decision/safety_status",
            lambda message: setattr(self, "last_safety", message),
            10,
        )
        self.create_timer(0.02, self._publish_state)

    def _increment(self, key):
        self.counts[key] += 1

    def _on_track(self, message):
        self.counts["track"] += 1
        self.valid_track_seen |= bool(message.valid)
        self.last_track = message

    def _on_mission(self, message):
        self.maximum_state = max(self.maximum_state, int(message.state))
        self.seen_states.add(int(message.state))
        self.last_mission = message

    def _on_search(self, message):
        self.search_active_seen |= bool(message.active)
        self.search_cued_seen |= bool(message.target_cued)

    def _publish_state(self):
        timestamp = self.get_clock().now().nanoseconds // 1000
        attitude = VehicleAttitude()
        attitude.timestamp = timestamp
        attitude.q = [1.0, 0.0, 0.0, 0.0]
        self._attitude_pub.publish(attitude)

        position = VehicleLocalPosition()
        position.timestamp = timestamp
        position.x = 0.0
        position.y = 0.0
        position.z = -5.0
        position.vx = 0.0
        position.vy = 0.0
        position.vz = 0.0
        position.xy_valid = True
        position.z_valid = True
        position.v_xy_valid = True
        position.v_z_valid = True
        self._position_pub.publish(position)

        status = VehicleStatus()
        status.timestamp = timestamp
        status.arming_state = VehicleStatus.ARMING_STATE_ARMED
        status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        status.failsafe = False
        status.takeoff_time = 1
        self._status_pub.publish(status)

    def passed(self, forbidden):
        common = forbidden == 0 and MissionStatus.STATE_SEARCH in self.seen_states
        if self.profile == "search":
            return (
                common
                and self.counts["search"] > 20
                and self.counts["front"] == 0
                and self.counts["back"] == 0
                and self.search_active_seen
                and self.search_cued_seen
                and not self.valid_track_seen
                and self.counts["fire"] == 0
                and self.counts["hit"] == 0
            )
        if self.profile == "tracking":
            return (
                common
                and min(self.counts["front"], self.counts["back"]) > 20
                and self.counts["search"] == 0
                and self.valid_track_seen
                and MissionStatus.STATE_TRACK in self.seen_states
                and MissionStatus.STATE_APPROACH in self.seen_states
                and self.counts["fire"] == 0
                and self.counts["hit"] == 0
            )
        if self.profile == "strike":
            return (
                common
                and self.valid_track_seen
                and self.counts["fire"] >= 1
                and self.counts["hit"] >= 1
                and MissionStatus.STATE_FIRE in self.seen_states
                and MissionStatus.STATE_EGRESS in self.seen_states
                and MissionStatus.STATE_RECOVER in self.seen_states
                and MissionStatus.STATE_DONE in self.seen_states
            )
        if self.profile == "full":
            required_states = {
                MissionStatus.STATE_SEARCH,
                MissionStatus.STATE_TRACK,
                MissionStatus.STATE_APPROACH,
                MissionStatus.STATE_FIRE,
                MissionStatus.STATE_EGRESS,
                MissionStatus.STATE_RECOVER,
                MissionStatus.STATE_DONE,
            }
            return (
                common
                and min(
                    self.counts["front"],
                    self.counts["back"],
                    self.counts["search"],
                )
                > 20
                and self.valid_track_seen
                and required_states.issubset(self.seen_states)
                and self.counts["fire"] >= 2
                and self.counts["hit"] >= 2
            )
        return (
            min(
                self.counts["front"],
                self.counts["back"],
                self.counts["search"],
            )
            > 20
            and self.counts["track"] > 10
            and self.valid_track_seen
            and self.maximum_state >= MissionStatus.STATE_APPROACH
            and forbidden == 0
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--profile",
        choices=("legacy", "search", "tracking", "strike", "full"),
        default="legacy",
    )
    parser.add_argument("--duration", type=float, default=12.0)
    args = parser.parse_args()
    rclpy.init()
    node = ChainSmoke(args.profile)
    started = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - started < max(1.0, args.duration):
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        forbidden = sum(
            len(node.get_publishers_info_by_topic(topic))
            for topic in (
                "/fmu/in/offboard_control_mode",
                "/fmu/in/trajectory_setpoint",
                "/fmu/in/vehicle_command",
            )
        )
        print(
            "VIRTUAL_CHAIN_SMOKE profile=%s counts=%s valid_track=%s "
            "states=%s max_state=%d search_active=%s search_cued=%s "
            "px4_input_publishers=%d mission_reason=%r track=%s capture=%s safety=%s"
            % (
                node.profile,
                node.counts,
                node.valid_track_seen,
                sorted(node.seen_states),
                node.maximum_state,
                node.search_active_seen,
                node.search_cued_seen,
                forbidden,
                None if node.last_mission is None else node.last_mission.reason,
                None
                if node.last_track is None
                else {
                    "valid": node.last_track.valid,
                    "hits": node.last_track.consecutive_hits,
                    "position": (
                        node.last_track.position_camera.x,
                        node.last_track.position_camera.y,
                        node.last_track.position_camera.z,
                    ),
                    "velocity": (
                        node.last_track.velocity_camera.x,
                        node.last_track.velocity_camera.y,
                        node.last_track.velocity_camera.z,
                    ),
                    "timing_delta": node.last_track.timing_delta_s,
                    "reprojection": node.last_track.reprojection_error_px,
                    "geometry": node.last_track.measurement_geometry_score,
                    "range_m": node.last_track.range_m,
                    "range_sigma_m": node.last_track.range_sigma_m,
                },
                None
                if node.last_capture is None
                else {
                    "quality": node.last_capture.quality_ok,
                    "alignment": node.last_capture.alignment_ok,
                    "range": node.last_capture.range_ok,
                    "los_rate": node.last_capture.los_rate_ok,
                    "relative_speed": node.last_capture.relative_speed_ok,
                    "prediction": node.last_capture.prediction_ok,
                    "frames": node.last_capture.frame_count_ok,
                    "instantaneous": node.last_capture.instantaneous_pass,
                    "held": node.last_capture.held,
                    "alignment_error": node.last_capture.alignment_error_rad,
                    "range_m": node.last_capture.range_m,
                },
                None
                if node.last_safety is None
                else {
                    "quality": node.last_safety.tracking_quality_passed,
                    "output": node.last_safety.output_allowed,
                    "reason": node.last_safety.reason,
                },
            )
        , flush=True)
        passed = node.passed(forbidden)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
