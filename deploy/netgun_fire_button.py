#!/usr/bin/env python3
"""网枪发射按钮的板上助手（I-1，2026-10-19）。

用法（PC 面板经 SSH 调用；也可手动执行）：
  python3 ~/interceptor_ws/deploy/netgun_fire_button.py status
  python3 ~/interceptor_ws/deploy/netgun_fire_button.py fire

status：采样 /debug/strike_state（2s），输出单行 JSON：
  {"ok":true,"running":..,"fire_mode":"manual|auto","ready":..,"latched":..,
   "auth":..,"est":..,"mq":..,"range_m":..,"axis_deg":..,"shots":..,"dage":..,
   "reason":"..."}
fire：ready 时向 /netgun/fire_command 发布一次 true（strike 在授权/估计/
深度硬门内执行单发；未就绪则拒绝并给出原因）。单发闭锁在火控侧，
重复按键天然安全。

需先 source ROS 环境（面板侧 board_client 已拼接）。
"""

import json
import re
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String

STRIKE_TOPIC = "/debug/strike_state"
FIRE_TOPIC = "/netgun/fire_command"
# 本板 FastDDS 晚加入订阅者匹配延迟 1-2s+（H-4/preflight 同款经验），
# 采样窗取 8s：先等匹配，再取最新一条
SAMPLE_SECONDS = 8.0


def parse_state(text):
    fields = {}
    for match in re.finditer(r"(\w+)=([\w.+-]+)", text):
        key, raw = match.group(1), match.group(2)
        try:
            fields[key] = int(raw) if re.fullmatch(r"[+-]?\d+", raw) \
                else float(raw)
        except ValueError:
            fields[key] = raw
    return fields


def sample_state(node):
    seen = []
    node.create_subscription(
        String, STRIKE_TOPIC,
        lambda msg: seen.append(parse_state(msg.data)),
        qos_profile_sensor_data)
    deadline = time.monotonic() + SAMPLE_SECONDS
    while time.monotonic() < deadline and not seen:
        rclpy.spin_once(node, timeout_sec=0.1)
    return seen[-1] if seen else None


def emit(payload):
    print(json.dumps(payload, ensure_ascii=False))


def main():
    action = sys.argv[1] if len(sys.argv) > 1 else "status"
    if action not in ("status", "fire"):
        emit({"ok": False, "reason": "用法: netgun_fire_button.py status|fire"})
        return 2
    rclpy.init()
    node = Node("netgun_fire_button")
    try:
        state = sample_state(node)
        if not state:
            emit({"ok": False, "running": False,
                  "reason": "未收到火控状态（拦截链未运行或未就绪）"})
            return 1
        # 手动就绪 = 火控 diag 的 fready=1（授权+估计+深度硬门与单发闭锁
        # 在 strike 侧逐拍计算）；旧格式无 fready 时按 auth&est 近似。
        fire_mode = state.get("fmode", "auto")
        if "fready" in state:
            ready = state.get("fready") == 1
        else:
            ready = state.get("auth") == 1 and state.get("est") == 1
        payload = {
            "ok": True,
            "running": True,
            "fire_mode": fire_mode,
            "ready": bool(ready),
            "latched": state.get("shots", 0) >= 1,
            "auth": state.get("auth"),
            "est": state.get("est"),
            "mq": state.get("mq"),
            "range_m": state.get("range"),
            "axis_deg": state.get("axis_err"),
            "shots": state.get("shots", 0),
            "dage": state.get("dage"),
        }
        if action == "status":
            payload["reason"] = (
                "已发射" if payload["latched"] else
                ("就绪，可发射" if ready else
                 "未就绪：需 armed+offboard+高度+稳定跟踪+有效估计（含深度）"))
            emit(payload)
            return 0
        if fire_mode != "manual":
            emit({"ok": False, "running": True, "fire_mode": fire_mode,
                  "reason": "当前为 auto 模式，火控自动开火，按钮不生效"})
            return 1
        if payload["latched"]:
            emit({"ok": False, "running": True, "fire_mode": fire_mode,
                  "latched": True, "reason": "已发射过（单发），按钮不再生效"})
            return 1
        if not ready:
            payload["reason"] = "发射条件未就绪（见 auth/est/dage 字段）"
            emit(payload)
            return 1
        pub = node.create_publisher(Bool, FIRE_TOPIC, 1)
        # 等一小段发现时间再发，确保订阅方（strike）已匹配
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        msg = Bool()
        msg.data = True
        pub.publish(msg)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        payload["fired"] = "requested"
        payload["reason"] = "发射指令已发出（单发闭锁生效）"
        emit(payload)
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
