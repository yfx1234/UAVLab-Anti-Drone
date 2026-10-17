#!/usr/bin/env bash
# 悬停标定参考数据采集（外场免照看版，2026-08-12）
# 用法（起飞前执行一次即可，之后无需任何操作）：
#   bash capture_hover_reference.sh [总时长秒, 默认150]
# 特性：自动后台脱离 SSH 会话；定时 SIGINT 干净关包；自动提取最稳悬停窗；
#       结果写入结果文件并 sync 落盘；落地后直接断电不影响已落盘数据。
set -Eeuo pipefail

duration="${1:-150}"
ws=/home/uavlab/Jetson_ROS2_V0/ros2_ws
out=~/codex_work/悬停标定_$(date +%Y%m%d_%H%M%S)
mkdir -p "${out}"

# 自后台脱离：前台启动时立刻转为 setsid+nohup，SSH 断开不影响
if [[ -z "${HOVER_REF_DETACHED:-}" ]]; then
  HOVER_REF_DETACHED=1 setsid nohup bash "$0" "${duration}" >"${out}/run.log" 2>&1 &
  echo "[hover-ref] 已后台启动，输出目录: ${out}（${duration}s 后自动完成并落盘）"
  exit 0
fi

set +u
source /opt/ros/humble/setup.bash
source "${ws}/install/setup.bash"
set -u
export ROS2CLI_NO_DAEMON=1

echo "[hover-ref] $(date --iso-8601=seconds) 开始录制 ${duration}s"
timeout -s INT "${duration}" ros2 bag record -o "${out}/bag" \
  /fmu/out/vehicle_local_position \
  /fmu/out/vehicle_attitude \
  /fmu/out/vehicle_status \
  /fmu/out/battery_status \
  /fmu/out/manual_control_setpoint \
  >>"${out}/run.log" 2>&1 || true
sync

{
python3 - "${out}/bag" <<'PY'
import glob, sqlite3, sys, math, statistics

dbs = sorted(glob.glob(sys.argv[1] + "/*.db3"))
if not dbs:
    print("[hover-ref] 未录到数据"); raise SystemExit(1)
con = sqlite3.connect(f"file:{dbs[0]}?mode=ro", uri=True)
tid = {n: i for i, n in con.execute("SELECT id, name FROM topics")}
t0 = con.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]

def rows(topic):
    if topic not in tid: return []
    return con.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid[topic],)).fetchall()

from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude, BatteryStatus, VehicleStatus
from rclpy.serialization import deserialize_message

# 解锁状态时间线
armed = []   # (t, armed_bool)
for ts, d in rows("/fmu/out/vehicle_status"):
    m = deserialize_message(d, VehicleStatus)
    armed.append(((ts - t0) / 1e9, m.arming_state == 2))

def is_armed(t):
    cur = False
    for ta, a in armed:
        if ta <= t: cur = a
        else: break
    return cur

# 空中段样本
vel, att = [], []
for ts, d in rows("/fmu/out/vehicle_local_position"):
    t = (ts - t0) / 1e9
    m = deserialize_message(d, VehicleLocalPosition)
    if is_armed(t) and m.z < -1.5:
        vel.append((t, m.vz, m.z))
att_map = []
for ts, d in rows("/fmu/out/vehicle_attitude"):
    t = (ts - t0) / 1e9
    m = deserialize_message(d, VehicleAttitude)
    w, x, y, z = m.q
    att_map.append((t, math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))))

volts, amps = [], []
for ts, d in rows("/fmu/out/battery_status"):
    t = (ts - t0) / 1e9
    m = deserialize_message(d, BatteryStatus)
    if m.voltage_v > 1.0 and is_armed(t):
        volts.append(m.voltage_v); amps.append(m.current_a)

if len(vel) < 200:
    print(f"[hover-ref] 空中样本不足（{len(vel)} 点），未检测到有效悬停段"); raise SystemExit(1)

# 滑动窗（20s, 步进5s）选 vz 标准差最小且倾角最小的窗口
best = None
starts = [vel[0][0] + 5 * i for i in range(int((vel[-1][0] - vel[0][0] - 20) / 5) + 1)]
for s in starts:
    wv = [v for v in vel if s <= v[0] < s + 20]
    if len(wv) < 150: continue
    wa = [a for a in att_map if s <= a[0] < s + 20]
    vz_std = statistics.pstdev([v[1] for v in wv])
    tilt_mean = statistics.mean([a[1] for a in wa]) if wa else 99.0
    score = vz_std + 0.02 * tilt_mean
    if best is None or score < best[0]: best = (score, s, wv, wa)

if best is None:
    print("[hover-ref] 未找到满 20s 的空中窗口"); raise SystemExit(1)

_, s, wv, wa = best
vzs = [v[1] for v in wv]; zs = [v[2] for v in wv]
tilts = [a[1] for a in wa]
print(f"[hover-ref] 最稳悬停窗: {s:.1f}s ~ {s+20:.1f}s（bag 相对时间）")
print(f"[hover-ref] vz(m/s): 均值{statistics.mean(vzs):.3f} 标准差{statistics.pstdev(vzs):.3f}")
print(f"[hover-ref] 高度z(m): 均值{statistics.mean(zs):.2f} 峰谷{max(zs)-min(zs):.2f}")
print(f"[hover-ref] 倾角(deg): 均值{statistics.mean(tilts):.2f} 最大{max(tilts):.2f}")
print(f"[hover-ref] 电压(V): 均值{statistics.mean(volts):.2f} 最低{min(volts):.2f}" if volts else "[hover-ref] 电压: 无数据")
print(f"[hover-ref] 电流(A): 均值{statistics.mean(amps):.2f} 最大{max(amps):.2f}" if amps else "[hover-ref] 电流: 无数据")
ok = statistics.pstdev(vzs) < 0.25 and statistics.mean(tilts) < 5.0
print(f"[hover-ref] 悬停质量判定: {'CLEAN（可采信HTE回写值）' if ok else 'NOISY（建议重飞更稳一段）'}")
PY
} > "${out}/悬停统计.txt" 2>&1
sync
echo "[hover-ref] $(date --iso-8601=seconds) 完成，结果: ${out}/悬停统计.txt" >> "${out}/run.log"
sync
