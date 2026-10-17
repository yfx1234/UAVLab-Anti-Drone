#!/usr/bin/env python3
"""flight_status_watcher:飞行状态聚合监视器(板端常驻,只订阅不占相机)。

订阅链路与飞控关键话题,把每类最后一条消息以 1 Hz 落地为
/tmp/flight_status.json,供本机诊断面板经 SSH 一次读取。
任何话题缺失(如飞控未上电)只影响对应字段,不影响整体。
"""

from __future__ import annotations

import json
import math
import os
import threading
import time

OUTPUT = "/tmp/flight_status.json"


def main():
    import rclpy
    from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    from std_msgs.msg import String

    class FlightStatusWatcher(Node):
        def __init__(self):
            super().__init__("flight_status_watcher")
            self._lock = threading.Lock()
            self._started = time.time()
            self._latest = {
                "perception": None, "track": None, "control": None,
                "vehicle_status": None, "vehicle_local": None,
            }
            sensor_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE)

            def on_json(field):
                def handler(message):
                    try:
                        payload = json.loads(message.data)
                    except ValueError:
                        return
                    with self._lock:
                        self._latest[field] = {
                            "stamp_s": time.time(), "data": payload}
                return handler

            self.create_subscription(
                String, "/zed_perception_node/status",
                on_json("perception"), sensor_qos)
            self.create_subscription(
                String, "/zed_tracking/track_json", on_json("track"), sensor_qos)
            self.create_subscription(
                String, "/zed_tracking/track_hold_status",
                on_json("control"), sensor_qos)
            self.create_subscription(
                VehicleStatus, "/fmu/out/vehicle_status",
                self._on_status, sensor_qos)
            self.create_subscription(
                VehicleLocalPosition, "/fmu/out/vehicle_local_position",
                self._on_local, sensor_qos)
            self.create_timer(1.0, self._flush)
            self.get_logger().warning("flight status watcher ready")

        def _on_status(self, message):
            with self._lock:
                self._latest["vehicle_status"] = {
                    "stamp_s": time.time(),
                    "data": {
                        # PX4 v1.15:arming 2=ARMED;nav 14=OFFBOARD。
                        "arming_state": int(message.arming_state),
                        "nav_state": int(message.nav_state),
                        "armed": int(message.arming_state) == 2,
                        "offboard": int(message.nav_state) == 14,
                        "failsafe": bool(message.failsafe),
                    },
                }

        def _on_local(self, message):
            speed = math.hypot(
                float(message.vx or 0.0), float(message.vy or 0.0))
            with self._lock:
                self._latest["vehicle_local"] = {
                    "stamp_s": time.time(),
                    "data": {
                        "x": float(message.x), "y": float(message.y),
                        "z": float(message.z),
                        "vx": float(message.vx), "vy": float(message.vy),
                        "vz": float(message.vz),
                        "speed_xy": round(speed, 2),
                        "heading_deg": round(
                            math.degrees(float(message.heading or 0.0)), 1),
                        "xy_valid": bool(message.xy_valid),
                        "z_valid": bool(message.z_valid),
                    },
                }

        def _flush(self):
            with self._lock:
                snapshot = dict(self._latest)
            payload = {"written_s": time.time(),
                       "uptime_s": round(time.time() - self._started, 1),
                       "topics": snapshot}
            temporary = OUTPUT + ".tmp"
            try:
                with open(temporary, "w", encoding="utf-8") as handle:
                    json.dump(payload, handle, ensure_ascii=False)
                os.replace(temporary, OUTPUT)
            except OSError:
                pass

    rclpy.init()
    node = FlightStatusWatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            os.remove(OUTPUT)
        except OSError:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
