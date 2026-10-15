import math, sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
from nav_msgs.msg import Odometry
from px4_offboard_tracking.msg import RelativeEstimate
from anti_drone_interfaces.msg import DepthObservation

uri = sys.argv[1] if len(sys.argv) > 1 else '/home/key/netgun_logs/yaw_ab/bag_v2a'
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=uri, storage_id='sqlite3'),
       rosbag2_py.ConverterOptions('', ''))
att, lp, truths, ests, obss = [], [], [], [], []
while r.has_next():
    topic, data, t = r.read_next()
    try:
        if topic == '/px4_1/fmu/out/vehicle_attitude':
            att.append((t * 1e-9, deserialize_message(data, VehicleAttitude)))
        elif topic == '/px4_1/fmu/out/vehicle_local_position':
            lp.append((t * 1e-9, deserialize_message(data, VehicleLocalPosition)))
        elif topic == '/evaluation/relative_truth':
            truths.append((t * 1e-9, deserialize_message(data, Odometry)))
        elif topic == '/tracking/relative/estimate':
            ests.append((t * 1e-9, deserialize_message(data, RelativeEstimate)))
        elif topic == '/perception/depth/observation':
            obss.append((t * 1e-9, deserialize_message(data, DepthObservation)))
    except Exception as e:
        print('skip', topic, e)
print(f"counts att={len(att)} lp={len(lp)} truth={len(truths)} est={len(ests)} obs={len(obss)}")

def q_to_R(q):
    w, x, y, z = q
    return [
        [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
        [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]]

def Rv(R, v):
    return [R[i][0]*v[0] + R[i][1]*v[1] + R[i][2]*v[2] for i in range(3)]

def nearest(arr, ts):
    # arr: list of (t, msg) sorted by t
    lo, hi = 0, len(arr) - 1
    best, bd = 0, 1e18
    while lo <= hi:
        mid = (lo + hi) // 2
        d = abs(arr[mid][0] - ts)
        if d < bd: bd, best = d, mid
        if arr[mid][0] < ts: lo = mid + 1
        else: hi = mid - 1
    return arr[best][1], bd

U_LIM, V_LIM = 37.0, 22.5  # deg half-FOV (H74/V45)
# 目标起跑检测：真值水平速度 > 1 m/s 的首个时刻
t0 = None
for ts, m in truths:
    v = m.twist.twist.linear
    if math.hypot(v.x, v.y) > 1.0:
        t0 = ts; break
print(f"target start t0={t0:.2f}" if t0 else "target never moved")

print("t_rel  hdg(deg)  tgtspd  rel_xy(m)      u_ang   v_ang   FOV  est_valid  obs_vis")
for k in range(0, 26):
    ts = t0 + 0.25 * k
    tr, _ = nearest(truths, ts)
    rel = tr.pose.pose.position
    lpm, _ = nearest(lp, ts)
    hdg = math.degrees(lpm.heading)
    v = tr.twist.twist.linear
    spd = math.hypot(v.x, v.y)
    atm, _ = nearest(att, ts)
    R = q_to_R((atm.q[0], atm.q[1], atm.q[2], atm.q[3]))
    pb = Rv([[R[j][i] for j in range(3)] for i in range(3)], (rel.x, rel.y, rel.z))  # R^T * v = ned->body
    # 新安装 q=(0,1,0,0): optical = [body_x, -body_y, -body_z]
    po = (pb[0], -pb[1], -pb[2])
    u_ang = math.degrees(math.atan2(po[0], po[2])) if po[2] > 0 else float('nan')
    v_ang = math.degrees(math.atan2(po[1], po[2])) if po[2] > 0 else float('nan')
    fov = "IN " if (abs(u_ang) <= U_LIM and abs(v_ang) <= V_LIM) else "OUT"
    em, _ = nearest(ests, ts)
    ov = sum(1 for t2, m2 in obss if abs(t2 - ts) < 0.25 and m2.visible)
    print(f"{0.25*k:5.2f}  {hdg:7.1f}  {spd:5.2f}  ({rel.x:+5.2f},{rel.y:+5.2f})  {u_ang:+6.1f}  {v_ang:+6.1f}  {fov}  {int(em.valid)}          {ov}")
