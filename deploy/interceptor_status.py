#!/usr/bin/env python3
"""拦截链状态读取（只读，PC 面板 2s 轮询用，2026-10-15）。

输出单行 JSON 到 stdout：栈进程存活、最新 launch 日志的估计器/火控诊断行、
制导 phase 序列、开火记录与日志尾部。飞控是否在线从估计器 attmiss 推断
（attmiss=obs 且持续增长 = 无姿态数据 = 飞控未上电/uxrce 未跑）；解锁与
offboard 从火控诊断 arm=/ob= 读取。

安装参数（2026-10-22）：geometry.saved = 板上已保存的安装参数（面板「安装参数」，
以网枪中心为基准，下次启动生效）；geometry.applied = 最新日志启动时实际使用的参数
（run_interceptor_stack.sh 写入的 NETGUN_GEOMETRY 行；旧日志/手动启动时缺省）。

任何内部错误都降级为 {"error": ...}，绝不向日志目录写东西。
"""

import glob
import json
import os
import re
import subprocess
import sys
import time

LOG_ROOT = os.path.expanduser("~/netgun_logs")
PID_FILE = os.path.join(LOG_ROOT, "interceptor_stack.pid")
STATUS_CMD_TIMEOUT_S = 5.0


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


def stack_pid():
    pid = None
    try:
        with open(PID_FILE) as f:
            pid = int(f.read().strip())
    except (OSError, ValueError):
        pid = None
    running = False
    if pid:
        try:
            os.kill(pid, 0)
            running = True
        except OSError:
            running = False
    if not running:
        # 兜底：PID 文件丢失/失效时按进程名找（模式含 [] 防自匹配；只认生产域）
        pids = production_launch_pids(STATUS_CMD_TIMEOUT_S)
        if pids:
            running = True
            pid = pids[0]
    return running, pid


def parse_estimator(line):
    fields = {}
    for key in ("obs", "inv", "attmiss", "acc", "bo", "reset", "ren", "init",
                "valid"):
        match = re.search(r"\b%s=(-?\d+)" % key, line)
        if match:
            fields[key] = int(match.group(1))
    return fields


def parse_strike(line):
    fields = {}
    for key in ("est", "auth", "fcu", "arm", "ob", "h", "shots"):
        match = re.search(r"(?:^|\s)%s=(-?\d+)" % key, line)
        if match:
            fields[key] = int(match.group(1))
    match = re.search(r"range=([\d.]+)", line)
    if match:
        fields["range_m"] = float(match.group(1))
    # diag 实际字段名为 axis_err=（旧正则 axis= 从未命中，I-3 顺手修正）
    match = re.search(r"\baxis(?:_err)?=([\d.]+)deg", line)
    if match:
        fields["axis_deg"] = float(match.group(1))
    # I-1：开火模式与手动发射就绪（netgun diag 尾部 fmode=auto|manual|auto_manual、
    # fready=0|1；旧格式无这两个字段时不产出，面板按缺省处理）
    match = re.search(r"fmode=(\w+)", line)
    if match:
        fields["fire_mode"] = match.group(1)
    match = re.search(r"\bfready=(\d)", line)
    if match:
        fields["fire_ready"] = int(match.group(1))
    # I-3：手动兜底被拦首因（NONE/DISABLED/LATCHED/FCU_STALE/NOT_ARMED/
    # LOW_HEIGHT/TILT/NOT_AUTHORIZED）与枪管倾角（度）
    match = re.search(r"\bmblk=(\w+)", line)
    if match:
        fields["manual_block"] = match.group(1)
    match = re.search(r"\btilt=(-?[\d.]+)", line)
    if match:
        fields["tilt_deg"] = float(match.group(1))
    return fields


def geometry_status(log_text):
    """已保存与本次启动实际使用的安装参数；任何异常只降级本字段。"""
    info = {}
    try:
        here = os.path.dirname(os.path.abspath(__file__))
        if here not in sys.path:
            sys.path.insert(0, here)
        import mounting_geometry  # noqa: PLC0415 —— 与本脚本同目录部署
        info["saved"] = mounting_geometry.describe()
    except Exception as exc:  # noqa: BLE001 —— 文件损坏也不能拖垮状态轮询
        info["saved"] = {"ok": False, "error": str(exc)}
    applied = re.findall(r"^NETGUN_GEOMETRY (\{.*\})$", log_text or "", re.MULTILINE)
    if applied:
        try:
            info["applied"] = json.loads(applied[-1])
        except ValueError:
            info["applied"] = None
    return info


def main():
    result = {"utc": time.time()}
    running, pid = stack_pid()
    result["running"] = running
    result["pid"] = pid
    logs = []
    try:
        logs = sorted(
            glob.glob(os.path.join(LOG_ROOT, "interceptor_stack_*.log")),
            key=os.path.getmtime)
    except OSError:
        pass
    if not logs:
        result["log"] = None
        result["geometry"] = geometry_status("")
        print(json.dumps(result, ensure_ascii=False))
        return
    log_path = logs[-1]
    result["log"] = os.path.basename(log_path)
    try:
        with open(log_path, errors="replace") as f:
            text = f.read()
    except OSError:
        text = ""

    stamps = [float(m) for m in re.findall(r"\[(\d{9,12}\.\d+)\]", text)
              if 1.6e9 < float(m) < 2.0e9]
    if stamps:
        result["last_line_age_s"] = max(0.0, time.time() - stamps[-1])

    estimator_lines = re.findall(r"estimator diag: (.+)", text)
    strike_lines = re.findall(r"netgun diag: (.+)", text)
    if estimator_lines:
        result["estimator"] = parse_estimator(estimator_lines[-1])
        result["estimator_raw"] = estimator_lines[-1]
    if strike_lines:
        result["strike"] = parse_strike(strike_lines[-1])
        result["strike_raw"] = strike_lines[-1]
    # guidance alive 行（2026-10-15）：当前 phase + 心跳/指令计数
    alive_lines = re.findall(r"guidance alive: (.+)", text)
    if alive_lines:
        alive = {}
        match = re.search(r"ob=(\d+)", alive_lines[-1])
        if match:
            alive["ob_sent"] = int(match.group(1))
        match = re.search(r"sp=(\d+)", alive_lines[-1])
        if match:
            alive["sp_sent"] = int(match.group(1))
        match = re.search(r"phase=(\w+)", alive_lines[-1])
        if match:
            alive["phase"] = match.group(1)
        result["guidance"] = alive
    result["phases"] = re.findall(r"phase → (\w+)", text)
    fires = re.findall(r"NETGUN FIRE (.+)", text)
    result["shots"] = len(fires)
    result["fire_last"] = fires[-1] if fires else None

    result["geometry"] = geometry_status(text)

    lines = [ln for ln in text.splitlines()
             if ln.strip() and not ln.startswith("NETGUN_GEOMETRY ")]
    result["tail"] = lines[-8:]
    print(json.dumps(result, ensure_ascii=False))


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:  # noqa: BLE001 —— 轮询路径必须永不抛栈
        print(json.dumps({"error": str(exc)}, ensure_ascii=False))
