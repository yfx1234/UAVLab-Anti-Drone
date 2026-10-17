import math, sys, json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from anti_drone_interfaces.msg import DepthObservation
from px4_msgs.msg import VehicleAttitude
from px4_offboard_tracking.msg import RelativeEstimate
from nav_msgs.msg import Odometry

BAG = sys.argv[1]        # bag path
STRIP_M = float(sys.argv[2])   # >此深度剥掉 depth_valid（负数 = 不剥 = 基线）
OUT = sys.argv[3]        # 结果 json 路径

r = SequentialReader()
r.open(StorageOptions(uri=BAG, storage_id='sqlite3'), ConverterOptions('', ''))
msgs = []   # (t, kind, msg)
truths = [] # (t, Odometry)
while r.has_next():
    topic, data, t = r.read_next()
    ts = t * 1e-9
    if topic == '/perception/depth/observation':
        msgs.append((ts, 'obs', deserialize_message(data, DepthObservation)))
    elif topic == '/px4_1/fmu/out/vehicle_attitude':
        msgs.append((ts, 'att', deserialize_message(data, VehicleAttitude)))
    elif topic == '/evaluation/relative_truth':
        truths.append((ts, deserialize_message(data, Odometry)))
msgs.sort(key=lambda x: x[0])

class Replay(Node):
    def __init__(self):
        super().__init__('replay_bearing')
        sq = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
        aq = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
        self.obs_pub = self.create_publisher(DepthObservation, '/replay/obs', sq)
        self.att_pub = self.create_publisher(VehicleAttitude, '/replay/att', aq)
        self.ests = []
        self.create_subscription(RelativeEstimate, '/replay/est',
                                 lambda m: self.ests.append(m), 10)

rclpy.init()
node = Replay()
import threading
spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
spin.start()
import time
t0_wall = time.time()
t0_bag = msgs[0][0]
nan = float('nan')
n_stripped = n_visible = 0
w0 = w1 = None  # 可见观测发布的墙钟窗口（估计戳也是墙钟域）
for ts, kind, m in msgs:
    lag = (ts - t0_bag) - (time.time() - t0_wall)
    if lag > 0:
        time.sleep(lag)
    elif lag < -2.0:
        # 掉队超过 2s：重锚定墙钟，防止累计蠕变
        t0_wall = time.time() - (ts - t0_bag)
    if kind == 'att':
        node.att_pub.publish(m)
    else:
        # 重打戳到当前墙钟：估计器姿态缓冲按接收时刻键控，
        # bag 原戳是录制日的墙钟（相差数小时）→ 不重打会全部 attmiss
        m.header.stamp = node.get_clock().now().to_msg()
        if m.visible:
            n_visible += 1
            now_w = time.time()
            if w0 is None: w0 = now_w
            w1 = now_w
            if STRIP_M >= 0 and m.depth_z_m > STRIP_M:
                m.depth_valid = False
                m.depth_z_m = nan
                m.range_m = nan
                m.position_optical.x = nan
                m.position_optical.y = nan
                m.position_optical.z = nan
                n_stripped += 1
        node.obs_pub.publish(m)
time.sleep(1.0)

def nearest_truth(t):
    lo, hi, best, bd = 0, len(truths) - 1, 0, 1e18
    while lo <= hi:
        mid = (lo + hi) // 2
        d = abs(truths[mid][0] - t)
        if d < bd: bd, best = d, mid
        if truths[mid][0] < t: lo = mid + 1
        else: hi = mid - 1
    return truths[best][1]

lat_errs, z_errs, valid_n, total_n = [], [], 0, 0
for e in node.ests:
    t = e.header.stamp.sec + e.header.stamp.nanosec * 1e-9
    # 只统计可见观测窗口内（±0.5s）——窗口外的 coast 外推不是本测试
    # 的对象（bag 后段目标源复位、观测停流，coast 必然发散）
    if w0 is None or not (w0 - 0.5 <= t <= w1 + 0.5):
        continue
    total_n += 1
    valid_n += int(e.valid)
    tr = nearest_truth(t)
    tp = tr.pose.pose.position
    dx, dy, dz = e.position.x - tp.x, e.position.y - tp.y, e.position.z - tp.z
    lat_errs.append(math.hypot(dx, dy))
    z_errs.append(abs(dz))
if not lat_errs:
    print("NO_WINDOW_ESTIMATES")
    raise SystemExit(0)
lat_errs_s = sorted(lat_errs)
res = {
    'strip_m': STRIP_M, 'n_visible': n_visible, 'n_stripped': n_stripped,
    'est_total': total_n, 'est_valid_frac': valid_n / max(1, total_n),
    'lat_mean': sum(lat_errs) / len(lat_errs),
    'lat_p50': lat_errs_s[len(lat_errs)//2],
    'lat_max': max(lat_errs),
    'zerr_mean': sum(z_errs) / len(z_errs),
    'zerr_max': max(z_errs),
}
with open(OUT, 'w') as fh:
    json.dump(res, fh, indent=1)
print(json.dumps(res, indent=1))
rclpy.shutdown()
