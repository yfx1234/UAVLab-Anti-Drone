#!/usr/bin/env python3
"""Fault-path runtime checks for typed mission nodes; never connects to PX4."""

import sys
import time

import rclpy
from falcon_intercept_msgs.msg import GuidanceCommand, MissionStatus, TargetTrack, VehicleInterfaceStatus
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String


class FaultSmoke(Node):
    def __init__(self, scenario):
        super().__init__(f"real_flight_{scenario}_smoke")
        self.scenario = scenario
        self.started = time.monotonic()
        self.sequence = 1
        self.state = MissionStatus.STATE_INIT
        self.approach_seen = False
        self.search_seen = False
        self.search_returned_at = None
        self.abort_seen_at = None
        self.land_seen_at = None
        self.latest_command = None
        self.fire_count = 0
        self.vehicle_pub = self.create_publisher(VehicleInterfaceStatus, "/control/px4_status", 10)
        self.track_pub = self.create_publisher(TargetTrack, "/target/track", 10)
        self.reset_pub = self.create_publisher(Bool, "/intercept/reset", 10)
        self.create_subscription(MissionStatus, "/decision/mission_status", self.on_status, 10)
        self.create_subscription(GuidanceCommand, "/intercept/guidance_command", self.on_command, 10)
        self.create_subscription(String, "/intercept/fire_event", self.on_fire, 10)
        self.create_timer(0.05, self.tick)

    def on_status(self, msg):
        self.state = msg.state
        self.search_seen |= msg.state == MissionStatus.STATE_SEARCH
        self.approach_seen |= msg.state == MissionStatus.STATE_APPROACH
        if self.approach_seen and msg.state == MissionStatus.STATE_SEARCH and self.search_returned_at is None:
            self.search_returned_at = time.monotonic()
        if msg.state == MissionStatus.STATE_ABORT and self.abort_seen_at is None:
            self.abort_seen_at = time.monotonic()
        if msg.state == MissionStatus.STATE_LAND and self.land_seen_at is None:
            self.land_seen_at = time.monotonic()

    def on_command(self, msg):
        self.latest_command = msg

    def on_fire(self, _msg):
        self.fire_count += 1

    def publish_vehicle(self, *, armed=True, landed=False, offboard=True, takeover=False, land_ack=False):
        msg = VehicleInterfaceStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.sequence = self.sequence
        msg.px4_msgs_available = True
        msg.attitude_fresh = msg.local_position_fresh = msg.vehicle_status_fresh = True
        msg.armed, msg.landed, msg.offboard = armed, landed, offboard
        msg.takeover_latched = takeover
        msg.ready_for_mission = armed and not landed and offboard and not takeover
        msg.land_command_sent = land_ack
        msg.land_ack_received = land_ack
        msg.land_ack_accepted = land_ack
        msg.land_ack_result = 0 if land_ack else 255
        self.vehicle_pub.publish(msg)

    def publish_track(self):
        msg = TargetTrack()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.sequence = self.sequence
        msg.position_camera.z = 3.0
        msg.covariance[0] = msg.covariance[7] = msg.covariance[14] = 0.01
        msg.innovation_nis = 1.0
        msg.confidence = 0.9
        msg.age_s = 0.01
        msg.consecutive_hits = 10
        msg.valid = True
        self.track_pub.publish(msg)

    def tick(self):
        elapsed = time.monotonic() - self.started
        if self.scenario == "track_timeout":
            self.publish_vehicle()
            if not self.approach_seen:
                self.publish_track()
        elif self.scenario == "takeover_reset":
            if not self.search_seen:
                self.publish_vehicle()
            elif self.abort_seen_at is None or time.monotonic() - self.abort_seen_at < 0.6:
                self.publish_vehicle(armed=True, landed=False, offboard=False, takeover=True)
                if self.abort_seen_at is not None:
                    self.reset_pub.publish(Bool(data=True))  # Must be ignored while airborne.
            else:
                self.publish_vehicle(armed=False, landed=True, offboard=False, takeover=True)
                self.reset_pub.publish(Bool(data=True))
        elif self.scenario == "search_timeout":
            if self.state == MissionStatus.STATE_LAND:
                # Native PX4 Land leaves Offboard before touchdown.
                # Report landed before ACK once to prove FSM waits for both.
                ack = self.land_seen_at is not None and time.monotonic() - self.land_seen_at > 0.6
                self.publish_vehicle(armed=False, landed=True, offboard=False, land_ack=ack)
            else:
                self.publish_vehicle()
        self.sequence += 1


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in ("track_timeout", "takeover_reset", "search_timeout"):
        print("usage: real_flight_fault_smoke.py track_timeout|takeover_reset|search_timeout")
        return 64
    rclpy.init()
    node = FaultSmoke(sys.argv[1])
    deadline = time.monotonic() + 8.0
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.scenario == "track_timeout" and node.search_returned_at is not None:
                cmd = node.latest_command
                if cmd is not None and cmd.mission_state == MissionStatus.STATE_SEARCH:
                    if any(abs(v) > 1e-6 for v in (
                        cmd.velocity_body_flu.x, cmd.velocity_body_flu.y, cmd.velocity_body_flu.z)):
                        print("FAIL stale target did not produce zero translation")
                        return 2
                    print("PASS stopped TargetTrack stream returned APPROACH to SEARCH with zero translation")
                    return 0
            if node.scenario == "takeover_reset" and node.abort_seen_at is not None and node.state == MissionStatus.STATE_INIT:
                if time.monotonic() - node.abort_seen_at < 0.6:
                    print("FAIL airborne reset cleared ABORT")
                    return 3
                print("PASS takeover latched ABORT; only landed/disarmed explicit reset cleared it")
                return 0
            if node.scenario == "search_timeout" and node.state == MissionStatus.STATE_DONE:
                if node.fire_count != 0:
                    print(f"FAIL search timeout emitted FIRE count={node.fire_count}")
                    return 5
                if node.latest_command is None:
                    print("FAIL no guidance command observed")
                    return 6
                if node.land_seen_at is None or time.monotonic() - node.land_seen_at < 0.6:
                    print("FAIL FSM reached DONE before accepted Land ACK")
                    return 7
                print("PASS no-target search timeout requested native LAND and reached DONE without FIRE")
                return 0
        print(f"FAIL timeout scenario={node.scenario} state={node.state}")
        return 4
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
