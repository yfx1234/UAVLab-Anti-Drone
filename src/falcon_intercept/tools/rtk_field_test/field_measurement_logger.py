#!/usr/bin/env python3
"""Log interceptor-side measurements for the RTK truth field test.

Runs on the interceptor Jetson (which stays on the ground). Subscribes:

- own RTK:    /fmu/out/vehicle_gps_position (px4_msgs/SensorGps)
- stereo:     /perception/async_stereo_measurement (AsyncStereoMeasurement)
- fallback:   /perception/target/camera_position (TargetCameraPosition)

and writes one JSON object per line to a single JSONL file. Every record
carries `wall` (Jetson unix time from the message header or receive time)
so the offline comparator can align with the MAVLink truth log. Own RTK
also carries `utc` (GPS-disciplined, from SensorGps.time_utc_usec) as an
independent clock-mapping cross-check.

Topic names are parameters so bench smoke tests can use test topics
without colliding with the real PX4 publishers.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path


def stamp_to_unix(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def build_node(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data

    from falcon_intercept_msgs.msg import AsyncStereoMeasurement, TargetCameraPosition
    from px4_msgs.msg import SensorGps

    sink = open(args.output, "a", encoding="utf-8")

    class MeasurementLogger(Node):
        def __init__(self) -> None:
            super().__init__("field_measurement_logger")
            self.counts = {"own_rtk": 0, "async_meas": 0, "cam_pos": 0}
            self.create_subscription(
                SensorGps, args.own_rtk_topic, self.on_own_rtk, qos_profile_sensor_data
            )
            self.create_subscription(
                AsyncStereoMeasurement, args.async_topic, self.on_async, qos_profile_sensor_data
            )
            self.create_subscription(
                TargetCameraPosition, args.cam_pos_topic, self.on_cam_pos, qos_profile_sensor_data
            )
            self.create_timer(15.0, self.report)

        def emit(self, record: dict) -> None:
            self.counts[record["kind"]] += 1
            sink.write(json.dumps(record) + "\n")
            sink.flush()

        def on_own_rtk(self, msg) -> None:
            utc = int(msg.time_utc_usec) / 1e6 if int(msg.time_utc_usec) > 0 else None
            self.emit(
                {
                    "kind": "own_rtk",
                    "wall": time.time(),
                    "utc": utc,
                    "lat": float(msg.latitude_deg),
                    "lon": float(msg.longitude_deg),
                    "alt": float(msg.altitude_msl_m),
                    "fix": int(msg.fix_type),
                    "eph": float(msg.eph) if math.isfinite(msg.eph) else None,
                    "epv": float(msg.epv) if math.isfinite(msg.epv) else None,
                    "sats": int(msg.satellites_used),
                }
            )

        def on_async(self, msg) -> None:
            p = msg.position_camera
            self.emit(
                {
                    "kind": "async_meas",
                    "wall": stamp_to_unix(msg.header.stamp),
                    "seq": int(msg.sequence),
                    "valid": bool(msg.valid),
                    "reason": str(msg.reason),
                    "position": [float(p.x), float(p.y), float(p.z)],
                    "covariance": [float(v) for v in msg.position_covariance],
                    "timing_delta": float(msg.timing_delta),
                    "timing_sigma": float(msg.timing_sigma),
                    "reprojection_error_px": float(msg.reprojection_error_px),
                    "range_observability": float(msg.range_observability),
                    "geometry_score": float(msg.measurement_geometry_score),
                    "confidence": float(msg.confidence),
                }
            )

        def on_cam_pos(self, msg) -> None:
            p = msg.position_camera
            self.emit(
                {
                    "kind": "cam_pos",
                    "wall": stamp_to_unix(msg.header.stamp),
                    "seq": int(msg.sequence),
                    "valid": bool(msg.valid),
                    "position": [float(p.x), float(p.y), float(p.z)],
                    "covariance": [float(v) for v in msg.position_covariance],
                    "confidence": float(msg.confidence),
                }
            )

        def report(self) -> None:
            self.get_logger().info(f"counts={self.counts}")

    return rclpy, MeasurementLogger


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--output", required=True, help="JSONL output path")
    parser.add_argument("--own-rtk-topic", default="/fmu/out/vehicle_gps_position")
    parser.add_argument("--async-topic", default="/perception/async_stereo_measurement")
    parser.add_argument("--cam-pos-topic", default="/perception/target/camera_position")
    args = parser.parse_args()

    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    rclpy, node_class = build_node(args)
    rclpy.init()
    node = node_class()
    print(f"[field_measurement_logger] writing {args.output}", flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
