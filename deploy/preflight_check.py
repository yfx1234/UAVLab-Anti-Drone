#!/usr/bin/env python3
"""起飞前只读自检（R2-3，2026-10-17；F-4 重写 2026-10-18）。

用法（板上，拦截链已启动、相机+YOLO 运行、飞控上电）：
  source /opt/ros/humble/setup.bash 等 ROS 环境
  python3 ~/interceptor_ws/deploy/preflight_check.py

全部检查在进程内用 rclpy 完成（F-4：ros2 topic hz 输出到管道时块缓
冲、永不自退；topic info/echo 依赖 CLI daemon 的图视图——都不可靠）。
输出 PASS/FAIL/WARN 列表，退出码 0=无 FAIL（WARN 不计入）：
  1. /fmu/out/vehicle_attitude       >=20 Hz
  2. /fmu/out/vehicle_local_position >=10 Hz
  3. /fmu/out/vehicle_status         >=1.5 Hz（PX4 Commander 实际 2Hz，
     F-1——不要按 10Hz 要求）
  4. /perception/observations_json   13-17 Hz
  5. /debug/bridge_state             in_hz 13-17 且 last_stamp_age_ms<300
  6. /debug/estimator_state         3s 窗 Δatt(姿态拒收) << Δobs；
     Δobs=0（视场内无目标）判 WARN 不判 FAIL
  7. /fmu/in/vehicle_command         发布者数=0（rc_gated 红线，进程内
     count_publishers 查询，不经 daemon）
  8. timedatectl                     NTP 已同步——仅 WARN（外场离线）

环境门（G-2）：开头打印 ROS_DOMAIN_ID/ROS_LOCALHOST_ONLY；domain≠0 或
LOCALHOST_ONLY=1 直接退出 1（与拦截链发现配置不一致时结果不可信）。
"""

import os
import re
import subprocess
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, VehicleStatus
    from std_msgs.msg import String
except ImportError as exc:
    print("ROS 2 环境不可用（%s）：请先 source /opt/ros/humble/setup.bash 等" % exc)
    sys.exit(2)

T_ATT = "/fmu/out/vehicle_attitude"
T_LP = "/fmu/out/vehicle_local_position"
T_ST = "/fmu/out/vehicle_status"
T_OBS = "/perception/observations_json"
# mock 全链路（fullchain）里 synthetic 发 /test/ 前缀话题、桥经
# bridge_input_topic 参数指过去——生产布局没有该话题，两个都订，
# 哪个有数据用哪个（板上实测 2026-10-18）
T_OBS_ALT = "/test/perception/observations_json"
T_BRIDGE = "/debug/bridge_state"
T_EST = "/debug/estimator_state"
T_CMD = "/fmu/in/vehicle_command"

results = []


def check(name, status, detail=""):
    results.append((name, status, detail))


def kv(text):
    """key=value 单行 → dict（只取数值字段）。"""
    out = {}
    for key, value in re.findall(r"(\w+)=(-?[\d.]+)", text):
        try:
            out[key] = float(value)
        except ValueError:
            pass
    return out


def spin_for(node, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)


class PreflightNode(Node):
    """只订阅不发布；各话题独立计数并保留最后一条负载。"""

    def __init__(self):
        super().__init__("preflight_check")
        self.counts = {}
        self.last = {}
        for topic, msg_type in (
            (T_ATT, VehicleAttitude),
            (T_LP, VehicleLocalPosition),
            (T_ST, VehicleStatus),
            (T_OBS, String),
            (T_OBS_ALT, String),
            (T_BRIDGE, String),
            (T_EST, String),
        ):
            self.counts[topic] = 0
            self.last[topic] = None
            self.create_subscription(
                msg_type, topic, self._make_cb(topic), qos_profile_sensor_data)

    def _make_cb(self, topic):
        def cb(msg):
            self.counts[topic] += 1
            self.last[topic] = msg.data if hasattr(msg, "data") else msg
        return cb


def measure_all_hz(node, topics, window_s=5.0):
    """同一时间窗内测多个话题频率（各自计数差 / 窗长）。"""
    start = {t: node.counts[t] for t in topics}
    t0 = time.monotonic()
    spin_for(node, window_s)
    dt = time.monotonic() - t0
    return {t: (node.counts[t] - start[t]) / dt for t in topics}


def main():
    # G-2：先亮明并核对自身 DDS 环境——domain 非 0 或 localhost-only 时
    # 本脚本看到的是另一个 DDS 世界，后面所有结果都不可信
    domain = os.environ.get("ROS_DOMAIN_ID", "0")
    localhost = os.environ.get("ROS_LOCALHOST_ONLY", "未设置(=0)")
    print("DDS 环境：ROS_DOMAIN_ID=%s ROS_LOCALHOST_ONLY=%s（拦截链要求 0 / 非 1）"
          % (domain, localhost))
    if localhost.strip() == "1":
        print("环境不一致：ROS_LOCALHOST_ONLY=1 与拦截链(0)不同，检测结果不可信；"
              "source deploy/px4_ros2_environment.sh 后重跑")
        return 1
    if domain.strip() not in ("0", ""):
        print("环境不一致：ROS_DOMAIN_ID=%s 与拦截链(0)不同，检测结果不可信" % domain)
        return 1
    rclpy.init()
    node = PreflightNode()
    try:
        # 1-4. 飞控数据流 + 感知观测频率（共用 5s 窗）
        hz = measure_all_hz(
            node, [T_ATT, T_LP, T_ST, T_OBS, T_OBS_ALT], 5.0)
        check("飞控姿态 %s" % T_ATT, "PASS" if hz[T_ATT] >= 20.0 else "FAIL",
              "%.1f Hz" % hz[T_ATT])
        check("飞控位置 %s" % T_LP, "PASS" if hz[T_LP] >= 10.0 else "FAIL",
              "%.1f Hz" % hz[T_LP])
        # PX4 1.15.4 Commander 2Hz 发布（F-1），阈值 1.5 容忍丢一包
        check("飞控状态 %s" % T_ST, "PASS" if hz[T_ST] >= 1.5 else "FAIL",
              "%.1f Hz" % hz[T_ST])
        # 感知观测：生产 /perception/...，mock 全链路 /test/perception/...
        if hz[T_OBS] > 0.0 or hz[T_OBS_ALT] <= 0.0:
            obs_topic, obs_hz = T_OBS, hz[T_OBS]
        else:
            obs_topic, obs_hz = T_OBS_ALT, hz[T_OBS_ALT]
        check("感知观测 %s" % obs_topic,
              "PASS" if 13.0 <= obs_hz <= 17.0 else "FAIL",
              "%.1f Hz" % obs_hz)

        # 5. 桥状态（1Hz，窗口内应已收到）
        bridge = kv(node.last[T_BRIDGE]) if node.last[T_BRIDGE] else {}
        if not bridge:
            check("桥状态 %s" % T_BRIDGE, "FAIL", "无数据")
        else:
            in_hz = bridge.get("in_hz", -1.0)
            stamp_age = bridge.get("last_stamp_age_ms", 1e9)
            check("桥输入频率", "PASS" if 13.0 <= in_hz <= 17.0 else "FAIL",
                  "%.1f Hz" % in_hz)
            check("观测时间戳新鲜", "PASS" if stamp_age < 300.0 else "FAIL",
                  "%.0f ms" % stamp_age)

        # 6. 估计器姿态接受（Δatt=拒收增量 << Δobs=观测增量，3s 窗）
        est0 = kv(node.last[T_EST]) if node.last[T_EST] else {}
        spin_for(node, 3.0)
        est1 = kv(node.last[T_EST]) if node.last[T_EST] else {}
        if not est0 or not est1:
            check("估计器状态 %s" % T_EST, "FAIL", "无数据")
        else:
            d_obs = est1.get("obs", 0.0) - est0.get("obs", 0.0)
            d_att = est1.get("att", 0.0) - est0.get("att", 0.0)
            if d_obs <= 0:
                # G-2：起飞前目标常不在视场，桥不转发观测——无法判定，
                # 姿态链路健康以第 1-3 项的飞控频率为准
                check("估计器姿态接受（Δatt<<Δobs）", "WARN",
                      "无目标观测（Δobs=%.0f），姿态接受无法判定" % d_obs)
            else:
                # 允许启动期少量正常拒收：Δatt <= max(2, 10%·Δobs)
                ok = d_att <= max(2.0, 0.1 * d_obs)
                check("估计器姿态接受（Δatt<<Δobs）",
                      "PASS" if ok else "FAIL",
                      "dobs=%.0f datt=%.0f" % (d_obs, d_att))

        # 7. VehicleCommand 无发布者（rc_gated 红线，进程内图查询）
        spin_for(node, 2.0)  # 留 DDS 发现收敛时间
        n_pub = node.count_publishers(T_CMD)
        check("VehicleCommand 发布者数为 0",
              "PASS" if n_pub == 0 else "FAIL", str(n_pub))

        # 8. 时钟同步（外场离线常态，WARN 不计 FAIL）
        try:
            out = subprocess.run(["timedatectl"], capture_output=True,
                                 text=True, timeout=8).stdout
            synced = "synchronized: yes" in out
            check("系统时钟已同步（NTP）",
                  "PASS" if synced else "WARN", "" if synced else "未同步")
        except (OSError, subprocess.TimeoutExpired):
            check("时钟同步检查", "WARN", "timedatectl 不可用")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    print("=" * 56)
    failures = 0
    for name, status, detail in results:
        if status == "FAIL":
            failures += 1
        print("[%s] %s %s" % (status, name, ("· " + detail) if detail else ""))
    print("=" * 56)
    warns = sum(1 for _, s, _ in results if s == "WARN")
    print("结果：%d/%d 通过（%d WARN）" %
          (len(results) - failures - warns, len(results), warns))
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
