#!/usr/bin/env python3
"""Runtime smoke for the typed four-node mission chain; never connects to PX4."""

import sys
import time

import rclpy
from falcon_intercept_msgs.msg import MissionStatus, TargetTrack, VehicleInterfaceStatus
from rclpy.node import Node
from std_msgs.msg import String


class Smoke(Node):
    def __init__(self):
        super().__init__("real_flight_decision_smoke")
        self.started = time.monotonic()
        self.sequence = 1
        self.state = MissionStatus.STATE_INIT
        self.fire_count = 0
        self.vehicle_pub = self.create_publisher(VehicleInterfaceStatus, "/control/px4_status", 10)
        self.track_pub = self.create_publisher(TargetTrack, "/target/track", 10)
        self.create_subscription(MissionStatus, "/decision/mission_status", self.on_status, 10)
        self.create_subscription(String, "/intercept/fire_event", self.on_fire, 10)
        self.create_timer(0.05, self.tick)

    def on_status(self, msg):
        self.state = msg.state

    def on_fire(self, _msg):
        self.fire_count += 1

    def tick(self):
        elapsed = time.monotonic() - self.started
        vehicle = VehicleInterfaceStatus()
        vehicle.header.stamp = self.get_clock().now().to_msg()
        vehicle.sequence = self.sequence
        vehicle.px4_msgs_available = True
        vehicle.attitude_fresh = vehicle.local_position_fresh = vehicle.vehicle_status_fresh = True
        landed = self.state in (MissionStatus.STATE_LAND, MissionStatus.STATE_DONE) and elapsed > 2.0
        vehicle.armed = not landed
        vehicle.landed = landed
        # PX4 normally leaves Offboard for AUTO_LAND before touchdown; this must
        # not be mistaken for a human takeover after the mission requested Land.
        vehicle.offboard = not landed and self.state != MissionStatus.STATE_LAND
        vehicle.ready_for_mission = vehicle.offboard
        vehicle.land_command_sent = self.state in (MissionStatus.STATE_LAND, MissionStatus.STATE_DONE)
        vehicle.land_ack_received = vehicle.land_command_sent
        vehicle.land_ack_accepted = vehicle.land_command_sent
        vehicle.land_ack_result = 0
        vehicle.yaw_ned = 0.0
        self.vehicle_pub.publish(vehicle)

        if elapsed > 0.5 and not landed:
            track = TargetTrack()
            track.header.stamp = self.get_clock().now().to_msg()
            track.header.frame_id = "stereo_camera_optical"
            track.sequence = self.sequence
            track.position_camera.z = 1.7 if elapsed > 1.2 else 3.0
            track.covariance[0] = track.covariance[7] = track.covariance[14] = 0.01
            track.innovation_nis = 1.0
            track.confidence = 0.9
            track.age_s = 0.01
            track.consecutive_hits = 10
            track.valid = True
            track.reason = "smoke"
            self.track_pub.publish(track)
        self.sequence += 1


def main():
    rclpy.init()
    node = Smoke()
    deadline = time.monotonic() + 8.0
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.state == MissionStatus.STATE_DONE:
                if node.fire_count != 1:
                    print(f"FAIL fire_count={node.fire_count}")
                    return 2
                print("PASS typed mission chain reached DONE with exactly one FIRE")
                return 0
        print(f"FAIL timeout state={node.state} fire_count={node.fire_count}")
        return 3
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
