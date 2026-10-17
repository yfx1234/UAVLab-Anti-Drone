#!/usr/bin/env python3
"""Record RC axis ranges and PX4 mode transitions without publishing commands."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import rclpy
from px4_msgs.msg import FailsafeFlags, ManualControlSetpoint, VehicleStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


FIELDS = ("roll", "pitch", "yaw", "throttle", "aux1", "aux2", "aux3", "aux4", "aux5", "aux6")


class Monitor(Node):
    def __init__(self) -> None:
        super().__init__("px4_rc_mapping_monitor")
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.samples = 0
        self.valid_samples = 0
        self.ranges = {field: [float("inf"), float("-inf")] for field in FIELDS}
        self.nav_states: list[int] = []
        self.arming_states: list[int] = []
        self.manual_signal_lost_values: list[bool] = []
        self.failsafe_values: list[bool] = []
        self.create_subscription(
            ManualControlSetpoint,
            "/fmu/out/manual_control_setpoint",
            self.on_manual,
            qos,
        )
        self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status", self.on_status, qos
        )
        self.create_subscription(
            FailsafeFlags, "/fmu/out/failsafe_flags", self.on_failsafe, qos
        )

    def on_manual(self, message: ManualControlSetpoint) -> None:
        self.samples += 1
        self.valid_samples += int(bool(message.valid))
        for field in FIELDS:
            value = float(getattr(message, field))
            self.ranges[field][0] = min(self.ranges[field][0], value)
            self.ranges[field][1] = max(self.ranges[field][1], value)

    def on_status(self, message: VehicleStatus) -> None:
        nav_state = int(message.nav_state)
        arming_state = int(message.arming_state)
        if not self.nav_states or self.nav_states[-1] != nav_state:
            self.nav_states.append(nav_state)
        if not self.arming_states or self.arming_states[-1] != arming_state:
            self.arming_states.append(arming_state)
        self.failsafe_values.append(bool(message.failsafe))

    def on_failsafe(self, message: FailsafeFlags) -> None:
        self.manual_signal_lost_values.append(bool(message.manual_control_signal_lost))

    def result(self, duration_sec: float) -> dict:
        ranges = {}
        for field, (minimum, maximum) in self.ranges.items():
            ranges[field] = {
                "min": None if minimum == float("inf") else minimum,
                "max": None if maximum == float("-inf") else maximum,
            }
        return {
            "duration_sec": duration_sec,
            "manual_samples": self.samples,
            "valid_samples": self.valid_samples,
            "all_manual_samples_valid": self.samples > 0
            and self.valid_samples == self.samples,
            "ranges": ranges,
            "nav_state_transitions": self.nav_states,
            "arming_state_transitions": self.arming_states,
            "manual_signal_lost_ever": any(self.manual_signal_lost_values),
            "failsafe_ever": any(self.failsafe_values),
        }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration-sec", type=float, default=30.0)
    parser.add_argument("--json-output", required=True, type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = Monitor()
    started = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - started < args.duration_sec:
            rclpy.spin_once(node, timeout_sec=0.1)
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
