#!/usr/bin/env python3
"""Observe PX4 GPS and EKF readiness without publishing any ROS topics."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import rclpy
from px4_msgs.msg import EstimatorStatusFlags, SensorGps, VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class Monitor(Node):
    def __init__(self) -> None:
        super().__init__("px4_gps_gate_monitor")
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.gps_count = 0
        self.max_satellites = 0
        self.max_fix_type = 0
        self.latest_gps = None
        self.latest_ekf = None
        self.latest_local = None
        self.latest_status = None
        self.create_subscription(
            SensorGps, "/fmu/out/vehicle_gps_position", self.on_gps, qos
        )
        self.create_subscription(
            EstimatorStatusFlags,
            "/fmu/out/estimator_status_flags",
            lambda message: setattr(self, "latest_ekf", message),
            qos,
        )
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            lambda message: setattr(self, "latest_local", message),
            qos,
        )
        self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            lambda message: setattr(self, "latest_status", message),
            qos,
        )

    def on_gps(self, message: SensorGps) -> None:
        self.gps_count += 1
        self.latest_gps = message
        self.max_satellites = max(self.max_satellites, int(message.satellites_used))
        self.max_fix_type = max(self.max_fix_type, int(message.fix_type))

    def result(self, duration_sec: float) -> dict:
        gps = self.latest_gps
        ekf = self.latest_ekf
        local = self.latest_local
        status = self.latest_status
        return {
            "duration_sec": duration_sec,
            "gps_messages": self.gps_count,
            "max_satellites_used": self.max_satellites,
            "max_fix_type": self.max_fix_type,
            "latest_gps": {
                "fix_type": int(gps.fix_type),
                "satellites_used": int(gps.satellites_used),
                "eph_m": float(gps.eph),
                "epv_m": float(gps.epv),
                "hdop": float(gps.hdop),
                "vdop": float(gps.vdop),
                "vel_ned_valid": bool(gps.vel_ned_valid),
                "heading_finite": float(gps.heading) == float(gps.heading),
            }
            if gps is not None
            else None,
            "ekf": {
                "cs_gps": bool(ekf.cs_gps),
                "cs_gps_hgt": bool(ekf.cs_gps_hgt),
                "cs_gps_yaw": bool(ekf.cs_gps_yaw),
                "cs_fake_pos": bool(ekf.cs_fake_pos),
                "cs_inertial_dead_reckoning": bool(ekf.cs_inertial_dead_reckoning),
            }
            if ekf is not None
            else None,
            "local_position": {
                "xy_valid": bool(local.xy_valid),
                "v_xy_valid": bool(local.v_xy_valid),
                "heading_good_for_control": bool(local.heading_good_for_control),
                "xy_global": bool(local.xy_global),
            }
            if local is not None
            else None,
            "vehicle_status": {
                "pre_flight_checks_pass": bool(status.pre_flight_checks_pass),
                "failsafe": bool(status.failsafe),
                "arming_state": int(status.arming_state),
            }
            if status is not None
            else None,
        }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration-sec", type=float, default=45.0)
    parser.add_argument("--json-output", required=True, type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = Monitor()
    started = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - started < args.duration_sec:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        duration = time.monotonic() - started
        result = node.result(duration)
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(
            json.dumps(result, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(result, ensure_ascii=False, indent=2))
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
