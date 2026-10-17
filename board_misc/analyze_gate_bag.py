#!/usr/bin/env python3
"""Dump first-attack gate telemetry from a status bag."""
import sys
import sqlite3
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag = Path(sys.argv[1])
db = sorted(bag.glob("*.db3"))[0]
conn = sqlite3.connect(str(db))
topics = {
    str(name): (int(tid), str(tname))
    for tid, name, tname in conn.execute("SELECT id, name, type FROM topics")
}
tid, tname = topics["/tracking/pure_interceptor/status"]
msg_type = get_message(tname)
rows = []
for (data,) in conn.execute(
    "SELECT data FROM messages WHERE topic_id = ? ORDER BY id", (tid,)
):
    rows.append(deserialize_message(data, msg_type))

print("total statuses:", len(rows))
# print rows in first-attack window: from first DEPTH_ACQUIRE until first fire + 1s
t0 = None
fired = None
for r in rows:
    if t0 is None and r.state in (4, 5, 6, 9, 10) and r.continuous_tracking_duration_s > 0:
        t0 = r.header.stamp.sec + r.header.stamp.nanosec * 1e-9
    if fired is None and r.fire_authorized:
        fired = r.header.stamp.sec + r.header.stamp.nanosec * 1e-9
print("window start:", t0, "first fire_authorized:", fired)

def ts(r):
    return r.header.stamp.sec + r.header.stamp.nanosec * 1e-9

prev_dur = 0.0
n_false = 0
for r in rows:
    t = ts(r)
    if t0 is not None and t0 - 1.0 <= t <= (fired or t0 + 12) + 1.0:
        dur = r.continuous_tracking_duration_s
        if not r.depth_visible:
            n_false += 1
        interesting = (
            r.state in (4, 5, 6, 9, 10)
            or not r.depth_visible
            or dur < prev_dur - 1e-9
            or r.fire_authorized
        )
        if interesting:
            print(
                "t=%10.3f st=%-14s depth_vis=%d est_vis=%d age=%.3f dur=%6.3f "
                "remain=%6.3f fire=%d" % (
                    t, r.state_name, r.depth_visible, r.estimate_valid,
                    r.depth_observation_age_s, dur,
                    r.first_attack_tracking_hold_remaining_s, r.fire_authorized))
        prev_dur = dur
print("depth_visible=false samples in window:", n_false)
