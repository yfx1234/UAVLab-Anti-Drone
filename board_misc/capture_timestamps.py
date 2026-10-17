#!/usr/bin/env python3
"""Sample detections_json frames and report stamp vs host_receive per camera."""
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

TOPICS = {
    "front": "/stereo/front/detections_json",
    "back": "/stereo/back/detections_json",
    "search": "/search/detections_json",
}
TARGET = int(sys.argv[1]) if len(sys.argv) > 1 else 10


def dict_ns(d):
    return float(d["sec"]) + float(d["nanosec"]) / 1e9


class Capture(Node):
    def __init__(self):
        super().__init__("timestamp_capture")
        self.rows = {name: [] for name in TOPICS}
        self.done = {name: 0 for name in TOPICS}
        self.subs = {}
        for name, topic in TOPICS.items():
            self.subs[name] = self.create_subscription(
                String, topic, self.make_cb(name), qos_profile_sensor_data
            )

    def make_cb(self, name):
        def cb(msg):
            if self.done[name] >= TARGET:
                return
            local_now = time.time()
            try:
                payload = json.loads(msg.data)
            except (ValueError, TypeError):
                return
            stamp = dict_ns(payload["header"]["stamp"])
            host = dict_ns(payload["host_receive_time"])
            self.rows[name].append(
                {
                    "seq": payload.get("sequence"),
                    "stamp": stamp,
                    "host": host,
                    "local": local_now,
                    "recovered_ms": (host - stamp) * 1000.0,
                    "e2e_ms": (local_now - stamp) * 1000.0,
                    "source": payload.get("timestamp_source"),
                    "sigma": payload.get("timestamp_sigma_s"),
                }
            )
            self.done[name] += 1
        return cb


def main():
    rclpy.init()
    node = Capture()
    deadline = time.time() + 60.0
    while rclpy.ok() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if all(v >= TARGET for v in node.done.values()):
            break
    for name in TOPICS:
        rows = node.rows[name]
        print("==== %s (%d samples) ====" % (name, len(rows)))
        if not rows:
            print("  NO MESSAGES")
            continue
        rec = [r["recovered_ms"] for r in rows]
        e2e = [r["e2e_ms"] for r in rows]
        srcs = sorted({str(r["source"]) for r in rows})
        print("  timestamp_source: %s" % ", ".join(srcs))
        print("  seq range: %s .. %s" % (rows[0]["seq"], rows[-1]["seq"]))
        print("  host-stamp  (backdate recovered) ms: min %.1f med %.1f max %.1f"
              % (min(rec), sorted(rec)[len(rec)//2], max(rec)))
        print("  local-stamp (end-to-end)        ms: min %.1f med %.1f max %.1f"
              % (min(e2e), sorted(e2e)[len(e2e)//2], max(e2e)))
        for r in rows[:3]:
            print("    seq=%s stamp=%.3f recovered=%.1fms e2e=%.1fms src=%s"
                  % (r["seq"], r["stamp"], r["recovered_ms"], r["e2e_ms"], r["source"]))
    # inter-camera stamp offset at matching sequences
    fr = {r["seq"]: r["stamp"] for r in node.rows["front"]}
    bk = {r["seq"]: r["stamp"] for r in node.rows["back"]}
    common = sorted(set(fr) & set(bk))
    if common:
        offs = [(fr[s] - bk[s]) * 1000.0 for s in common]
        print("==== front-back same-sequence stamp offset ms (n=%d):" % len(common))
        print("  min %.2f med %.2f max %.2f" % (min(offs), sorted(offs)[len(offs)//2], max(offs)))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
