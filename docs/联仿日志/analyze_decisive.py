import math, sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from px4_offboard_tracking.msg import RelativeEstimate
from nav_msgs.msg import Odometry
from virtual_interception_interfaces.msg import VirtualShotEvent, VirtualHitResult

uri = sys.argv[1] if len(sys.argv) > 1 else '/home/key/netgun_logs/decisive'
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=uri, storage_id='sqlite3'),
       rosbag2_py.ConverterOptions('', ''))
shots, ests, truths, phys, solv = [], [], [], [], []
while r.has_next():
    topic, data, t = r.read_next()
    try:
        if topic == '/tracking/virtual_shot_event':
            shots.append(deserialize_message(data, VirtualShotEvent))
        elif topic == '/tracking/relative/estimate':
            ests.append(deserialize_message(data, RelativeEstimate))
        elif topic == '/evaluation/relative_truth':
            truths.append(deserialize_message(data, Odometry))
        elif topic == '/evaluation/virtual_hit_result':
            phys.append(deserialize_message(data, VirtualHitResult))
        elif topic == '/evaluation/virtual_solver_hit_result':
            solv.append(deserialize_message(data, VirtualHitResult))
    except Exception as e:
        print('skip', topic, e)
print(f"counts: shots={len(shots)} est={len(ests)} truth={len(truths)} phys={len(phys)} solv={len(solv)}")

def stamp_s(h): return h.stamp.sec + h.stamp.nanosec*1e-9
def v3(p): return (p.x, p.y, p.z)
def norm(a): return math.sqrt(sum(x*x for x in a))
def sub(a,b): return tuple(x-y for x,y in zip(a,b))
def dot(a,b): return sum(x*y for x,y in zip(a,b))
def ang(a,b):
    c = max(-1.0, min(1.0, dot(a,b)/(norm(a)*norm(b))))
    return math.degrees(math.acos(c))

for s in shots:
    ts = stamp_s(s.header)
    print(f"\n=== SHOT id={s.shot_id} t={ts:.3f} tof={s.predicted_time_of_flight_s:.3f} range={s.estimated_range_m:.2f}")
    d = v3(s.shot_direction_ned); ep = v3(s.estimated_target_position_ned); ev = v3(s.estimated_target_velocity_ned)
    pip = v3(s.predicted_intercept_point_ned)
    print(f"shot_dir=({d[0]:+.3f},{d[1]:+.3f},{d[2]:+.3f}) est_pos=({ep[0]:+.3f},{ep[1]:+.3f},{ep[2]:+.3f}) est_vel=({ev[0]:+.2f},{ev[1]:+.2f},{ev[2]:+.2f}) pip=({pip[0]:+.3f},{pip[1]:+.3f},{pip[2]:+.3f})")
    e = min(ests, key=lambda m: abs(stamp_s(m.header)-ts))
    tr = min(truths, key=lambda m: abs(stamp_s(m.header)-ts))
    tp = v3(tr.pose.pose.position)
    de = sub(v3(e.position), tp)
    print(f"@fire  est=({e.position.x:+.3f},{e.position.y:+.3f},{e.position.z:+.3f}) truth=({tp[0]:+.3f},{tp[1]:+.3f},{tp[2]:+.3f})")
    print(f"       est-truth=({de[0]:+.3f},{de[1]:+.3f},{de[2]:+.3f}) |xy|={math.hypot(de[0],de[1]):.3f} |3d|={norm(de):.3f} est_vel=({e.velocity.x:+.2f},{e.velocity.y:+.2f},{e.velocity.z:+.2f})")
    print(f"angles: aim-vs-est={ang(d, ep):.2f}deg aim-vs-truth={ang(d, tp):.2f}deg")
for tag, arr in (('PHYSICAL', phys), ('SOLVER', solv)):
    for m in arr:
        print(f"{tag} shot={m.shot_id} hit={int(m.hit)} miss={m.minimum_miss_distance_m:.3f} valid={int(m.result!=4 if hasattr(m,'result') else 1)} reason={m.reason}")
