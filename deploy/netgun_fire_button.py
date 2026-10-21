#!/usr/bin/env python3
"""网枪发射按钮的板上助手（I-1 2026-10-19；I-3 2026-10-21 改为自动+手动兜底）。

用法（PC 面板经 SSH 调用；也可手动执行）：
  python3 ~/interceptor_ws/deploy/netgun_fire_button.py status
  python3 ~/interceptor_ws/deploy/netgun_fire_button.py fire

I-3 起实机默认 fire_mode=auto_manual：火控门满足即自动单发；按钮是人工
兜底（自动一直不发时用）。手动门由 strike 逐拍计算并随 /debug/strike_state
发布：fready=1 即此刻按键会发射，mblk=被拦下的首因（NONE/DISABLED/LATCHED/
FCU_STALE/NOT_ARMED/LOW_HEIGHT/TILT/NOT_AUTHORIZED）。

status：输出单行 JSON
  {"ok","running","fire_mode","manual_enabled","auto_enabled","ready",
   "latched","block","auth","est","range_m","tilt_deg","shots","reason"}
fire：fready=1 时向 /netgun/fire_command 发布一次 true，再等火控状态确认
  shots 增加（"fired": true）；未就绪则不发布并给出中文原因。单发闭锁在
  火控侧，重复按键天然安全。

需先 source ROS 环境（面板侧 board_client 已拼接）。
"""

import json
import re
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String

STRIKE_TOPIC = "/debug/strike_state"
FIRE_TOPIC = "/netgun/fire_command"
STRIKE_NODE = "netgun_strike_node"
# 本板 FastDDS 晚加入订阅者匹配延迟 1-2s+（H-4/preflight 同款经验）：
# 首条火控状态最长等 8s；发布者与订阅同时创建，发现过程并行
SAMPLE_SECONDS = 8.0
MATCH_SECONDS = 3.0     # 等火控订阅匹配发射话题
CONFIRM_SECONDS = 3.0   # 发布后等 shots 增加（状态 2Hz）
MANUAL_MODES = ("manual", "auto_manual")
AUTO_MODES = ("auto", "auto_manual")

BLOCK_REASONS = {
    "NONE": "就绪，可发射",
    "DISABLED": "fire_mode=auto（仅自动开火），按钮未启用",
    "LATCHED": "已发射过（单发），按钮不再生效",
    "FCU_STALE": "飞控数据缺失或过期（姿态/状态/本地位置）",
    "NOT_ARMED": "飞机未解锁",
    "LOW_HEIGHT": "高度低于最低开火高度（或本地高度无效）",
    "TILT": "机体倾角过大（枪管偏离竖直超过上限）",
    "NOT_AUTHORIZED": "严格口径：需武器授权（offboard+就位+稳定跟踪）与有效估计",
}


def production_launch_pids(timeout_s=5.0):
    """I-3（2026-10-21）：只认生产域（ROS_DOMAIN_ID 为空或 0）的
    netgun_real.launch 进程——隔离域测试栈（fullchain/rehearsal 跑在
    domain 42）不算"拦截链运行中"，免得面板误报运行、误点停止把测试栈
    一并 SIGINT（与 run_interceptor_stack.sh production_launch_pids 同判据）。"""
    try:
        out = subprocess.run(
            ["pgrep", "-f", "netgun_real[.]launch"],
            capture_output=True, text=True, timeout=timeout_s,
        ).stdout.split()
    except (OSError, ValueError, subprocess.SubprocessError):
        return []
    pids = []
    for token in out:
        try:
            pid = int(token)
            with open("/proc/%d/environ" % pid, "rb") as f:
                env = f.read().split(b"\0")
        except (OSError, ValueError):
            continue
        domain = b""
        for item in env:
            if item.startswith(b"ROS_DOMAIN_ID="):
                domain = item.split(b"=", 1)[1].strip()
                break
        if domain in (b"", b"0"):
            pids.append(pid)
    return pids


def stack_running():
    """生产域拦截链 launch 是否存活（与 interceptor_status.py 同判据）。"""
    return bool(production_launch_pids())


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


def summarize(state):
    fire_mode = str(state.get("fmode", "auto"))
    try:
        shots = int(state.get("shots", 0) or 0)
    except (TypeError, ValueError):
        shots = 0
    if "fready" in state:
        ready = state.get("fready") == 1
    else:  # I-1 前旧格式：按 auth&est 近似
        ready = state.get("auth") == 1 and state.get("est") == 1
    block = state.get("mblk")
    if not isinstance(block, str):  # I-3 前旧格式无 mblk
        block = ("LATCHED" if shots >= 1 else
                 "NONE" if ready else
                 "DISABLED" if fire_mode not in MANUAL_MODES else
                 "NOT_AUTHORIZED")
    return {
        "ok": True,
        "running": True,
        "fire_mode": fire_mode,
        "manual_enabled": fire_mode in MANUAL_MODES,
        "auto_enabled": fire_mode in AUTO_MODES,
        "ready": bool(ready) and shots < 1,
        "latched": shots >= 1,
        "block": block,
        "auth": state.get("auth"),
        "est": state.get("est"),
        "range_m": state.get("range"),
        "tilt_deg": state.get("tilt"),
        "shots": shots,
    }


def reason_text(payload):
    if payload["latched"]:
        return "已发射（单发）"
    block = payload["block"]
    base = BLOCK_REASONS.get(block, "手动发射条件未满足（%s）" % block)
    if block == "NONE":
        return ("自动开火待命；手动兜底就绪，可发射" if payload["auto_enabled"]
                else "就绪，可发射")
    if payload["auto_enabled"] and block != "DISABLED":
        return "自动开火待命；手动未就绪：" + base
    return base


def spin_until(node, predicate, timeout_s):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        rclpy.spin_once(node, timeout_sec=0.05)
    return bool(predicate())


def strike_matched(node, pub):
    """发布端须与火控订阅真正匹配：仅 get_subscription_count()>0 不够——
    bag 录制器（record -a）等其他订阅者先匹配时会误判，单次发布丢失。
    图里要有火控订阅，且本发布端已匹配图中全部订阅者。"""
    infos = node.get_subscriptions_info_by_topic(FIRE_TOPIC)
    if not any(i.node_name == STRIKE_NODE for i in infos):
        return False
    return pub.get_subscription_count() >= len(infos)


def emit(payload):
    print(json.dumps(payload, ensure_ascii=False), flush=True)


def main():
    action = sys.argv[1] if len(sys.argv) > 1 else "status"
    if action not in ("status", "fire"):
        emit({"ok": False, "reason": "用法: netgun_fire_button.py status|fire"})
        return 2
    rclpy.init()
    node = Node("netgun_fire_button")
    states = []
    node.create_subscription(
        String, STRIKE_TOPIC, lambda msg: states.append(parse_state(msg.data)),
        qos_profile_sensor_data)
    pub = node.create_publisher(Bool, FIRE_TOPIC, 1) if action == "fire" else None
    try:
        if not spin_until(node, lambda: bool(states), SAMPLE_SECONDS):
            running = stack_running()
            emit({"ok": False, "running": running,
                  "reason": ("拦截链在运行但收不到火控状态（/debug/strike_state）"
                             "——飞控姿态/状态未到或 DDS 未匹配，无法发射"
                             if running else "拦截链未运行")})
            return 1
        payload = summarize(states[-1])
        if action == "status":
            payload["reason"] = reason_text(payload)
            emit(payload)
            return 0
        if not payload["manual_enabled"]:
            payload.update(ok=False, reason=BLOCK_REASONS["DISABLED"])
            emit(payload)
            return 1
        if payload["latched"]:
            payload.update(ok=False, reason=BLOCK_REASONS["LATCHED"])
            emit(payload)
            return 1
        if not payload["ready"]:
            payload.update(ok=False, reason="未发射：" + BLOCK_REASONS.get(
                payload["block"], payload["block"]))
            emit(payload)
            return 1
        if not spin_until(node, lambda: strike_matched(node, pub),
                          MATCH_SECONDS):
            # 兜底：有订阅者 QoS 不兼容时全匹配永远不成立——只要图中有火控
            # 订阅且已有匹配就照发（是否打出由 shots 确认，见下）
            has_strike = any(
                i.node_name == STRIKE_NODE
                for i in node.get_subscriptions_info_by_topic(FIRE_TOPIC))
            if not (has_strike and pub.get_subscription_count() > 0):
                payload.update(ok=False, reason="火控未订阅 /netgun/fire_command，"
                               "指令未发出")
                emit(payload)
                return 1
        seen_before = len(states)
        shots_before = payload["shots"]
        msg = Bool()
        msg.data = True
        pub.publish(msg)
        confirmed = spin_until(
            node,
            lambda: any(summarize(s)["shots"] > shots_before
                        for s in states[seen_before:]),
            CONFIRM_SECONDS)
        payload["fired"] = confirmed
        if confirmed:
            payload.update(latched=True, ready=False, shots=shots_before + 1,
                           block="LATCHED",
                           reason="已发射（火控确认 shots=%d）" % (shots_before + 1))
            emit(payload)
            return 0
        last = summarize(states[-1])
        payload.update(ok=False, reason="指令已发出但 %.0fs 内火控未确认（%s）"
                       % (CONFIRM_SECONDS, reason_text(last)))
        emit(payload)
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
