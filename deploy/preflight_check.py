#!/usr/bin/env python3
"""起飞前只读自检（R2-3，2026-10-17）。

用法（板上，拦截链已启动、相机+YOLO 运行、飞控上电）：
  python3 ~/interceptor_ws/deploy/preflight_check.py

只读检查项（输出 PASS/FAIL 列表，退出码 0=全过）：
  1. /fmu/out/{vehicle_attitude,vehicle_local_position,vehicle_status} 有数据
  2. /perception/observations_json 频率 ≈15Hz
  3. /debug/bridge_state：in_hz≈15 且 last_stamp_age_ms < 300
  4. /debug/estimator_state：attmiss 不增长（姿态被接受）
  5. /fmu/in/vehicle_command 发布者数为 0（rc_gated 红线）
  6. timedatectl 时钟已同步（NTP）
"""

import json
import re
import subprocess
import sys
import time

results = []


def check(name, ok, detail=""):
    results.append((name, bool(ok), detail))


def ros_env_ready():
    try:
        subprocess.run(["ros2", "--help"], capture_output=True, timeout=10)
        return True
    except (OSError, subprocess.TimeoutExpired):
        return False


def topic_hz(topic, timeout=6.0):
    """用一次性 rclpy 订阅测频率（比 ros2 topic hz 快且稳）。"""
    code = (
        "import rclpy,time,sys\n"
        "from rclpy.qos import qos_profile_sensor_data\n"
        "rclpy.init()\n"
        f"node=rclpy.create_node('pf')\n"
        f"n=[0]\n"
        f"node.create_subscription(sys.argv[1] if False else __import__('rosidl_runtime_py.utilities').__class__, 'x', lambda m: None, 1)\n"
    )
    return None  # 占位：实际用下方 subprocess 方式


def hz_via_cli(topic, timeout=8.0):
    try:
        out = subprocess.run(
            ["ros2", "topic", "hz", topic], capture_output=True,
            text=True, timeout=timeout).stderr + ""
        return out
    except subprocess.TimeoutExpired as exc:
        return (exc.stderr or "") + (exc.stdout or "")


def cli(*args, timeout=15):
    return subprocess.run(["ros2", *args], capture_output=True,
                          text=True, timeout=timeout)


def main():
    if not ros_env_ready():
        print("ROS 2 环境不可用：请先 source /opt/ros/humble/setup.bash 等")
        return 2
    # 1. 飞控数据流
    for topic in ("/fmu/out/vehicle_attitude", "/fmu/out/vehicle_local_position",
                  "/fmu/out/vehicle_status"):
        hz = hz_via_cli(topic)
        ok = "average rate" in hz
        rate = ""
        m = __import__("re").search(r"average rate: ([\d.]+)", hz)
        if m:
            rate = m.group(1) + " Hz"
        check("飞控数据 " + topic, ok, rate or "无数据")
    # 2. 感知 JSON 频率
    hz = hz_via_cli("/perception/observations_json")
    m = __import__("re").search(r"average rate: ([\d.]+)", hz)
    rate = float(m.group(1)) if m else 0.0
    check("感知观测 /perception/observations_json", 13.0 <= rate <= 17.0,
          ("%.1f Hz" % rate) if rate else "无数据")
    # 3. 桥状态
    try:
        out = cli("topic", "echo", "/debug/bridge_state", "--once",
                  "--field", "data", timeout=12).stdout
        in_hz = __import__("re").search(r"in_hz=([\d.]+)", out)
        stamp = __import__("re").search(r"last_stamp_age_ms=(-?[\d.]+)", out)
        check("桥输入频率", in_hz and 13.0 <= float(in_hz.group(1)) <= 17.0,
              in_hz.group(1) + " Hz" if in_hz else "无")
        check("观测时间戳新鲜", stamp and float(stamp.group(1)) < 300,
              (stamp.group(1) + " ms") if stamp else "无")
    except subprocess.TimeoutExpired:
        check("桥状态读取", False, "超时")
    # 4. 估计器姿态接受
    try:
        out = cli("topic", "echo", "/debug/estimator_state", "--once",
                  "--field", "data", timeout=12).stdout
        miss = __import__("re").search(r"attmiss=(\d+)", out)
        obs = __import__("re").search(r"obs=(\d+)", out)
        ok = miss and obs and int(miss.group(1)) == 0
        check("估计器姿态全部接受（attmiss=0）", bool(ok),
              "attmiss=" + (miss.group(1) if miss else "?"))
    except subprocess.TimeoutExpired:
        check("估计器状态读取", False, "超时")
    # 5. VehicleCommand 无发布者（rc_gated 红线）
    try:
        out = cli("topic", "info", "/fmu/in/vehicle_command", timeout=12).stdout
        m = __import__("re").search(r"Publisher count: (\d+)", out)
        n = int(m.group(1)) if m else -1
        check("VehicleCommand 发布者数为 0", n == 0, str(n))
    except subprocess.TimeoutExpired:
        check("VehicleCommand 发布者检查", False, "超时")
    # 6. 时钟同步
    try:
        out = subprocess.run(["timedatectl"], capture_output=True, text=True,
                             timeout=8).stdout
        synced = "synchronized: yes" in out
        check("系统时钟已同步（NTP）", synced)
    except (OSError, subprocess.TimeoutExpired):
        check("时钟同步检查", False, "timedatectl 不可用")

    failures = 0
    print("=" * 56)
    for name, ok, detail in results:
        mark = "PASS" if ok else "FAIL"
        if not ok:
            failures += 1
        print("[%s] %s %s" % (mark, name, ("· " + detail) if detail else ""))
    print("=" * 56)
    print("结果：%d/%d 通过" % (len(results) - failures, len(results)))
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
