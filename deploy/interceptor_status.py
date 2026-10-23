#!/usr/bin/env python3
"""拦截链状态读取（只读，PC 面板 2s 轮询用，2026-10-15）。

输出单行 JSON 到 stdout：栈进程存活、最新 launch 日志的估计器/火控诊断行、
制导 phase 序列、开火记录与日志尾部。飞控是否在线从估计器 attmiss 推断
（attmiss=obs 且持续增长 = 无姿态数据 = 飞控未上电/uxrce 未跑）；解锁与
offboard 从火控诊断 arm=/ob= 读取。

安装参数（2026-10-22）：geometry.saved = 板上已保存的安装参数（面板「安装参数」，
以网枪顶面中心为基准，下次启动生效）；geometry.applied = 最新日志启动时实际使用的参数
（run_interceptor_stack.sh 写入的 NETGUN_GEOMETRY 行；旧日志/手动启动时缺省）。

常驻发射通道与复盘记录（2026-10-23）：fire_link = 拦截链运行时经本机套接字 ping
netgun_fire_button.py serve 的结果（link.ready=按键直达火控；state=≤1.5s 内的
2Hz 火控状态，比 5s 一条的 diag 日志新）；review = 本次复盘目录、录制器是否
在录、目录大小与剩余空间（无当前记录时给最近一次目录名）。

实物击发（2026-10-23）：trigger = 本次启动装订的 trigger_command（运行日志 NETGUN_TRIGGER 行，
bound=""→仅虚拟；旧日志无此行→null）+ 常驻击发服务 netgun_pwm_link 状态（飞控心跳/解锁、
AUX5 参数核对、本次启动以来的最近一次击发 last_fire）。

任何内部错误都降级为 {"error": ...}，绝不向日志目录写东西。
"""

import glob
import json
import os
import re
import shutil
import subprocess
import sys
import time

LOG_ROOT = os.path.expanduser("~/netgun_logs")
PID_FILE = os.path.join(LOG_ROOT, "interceptor_stack.pid")
REVIEW_FILE = os.path.join(LOG_ROOT, "interceptor_review.current")
BAG_PID_FILE = os.path.join(LOG_ROOT, "interceptor_bag.pid")
STATUS_CMD_TIMEOUT_S = 5.0
PING_REPLY_S = 0.5


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


def _sibling(name):
    here = os.path.dirname(os.path.abspath(__file__))
    if here not in sys.path:
        sys.path.insert(0, here)
    return __import__(name)


def fire_link_status():
    """常驻发射通道 ping（本机套接字，无 ROS）；不可用时 ok=false 与原因。"""
    try:
        reply = _sibling("netgun_fire_button").channel_request(
            "ping", confirm_s=PING_REPLY_S)
    except Exception as exc:  # noqa: BLE001 —— 通道缺失/旧部署只降级本字段
        return {"ok": False, "error": str(exc)}
    info = {"ok": True, "link": reply.get("link") or {}}
    if isinstance(reply.get("state"), dict):
        info["state"] = reply["state"]
    return info


def trigger_status(log_text):
    """实物击发装订 + 击发服务状态（本机套接字，无 ROS）；服务不可用时 ok=false 与原因。"""
    bound = re.findall(r"^NETGUN_TRIGGER ?(.*)$", log_text or "", re.MULTILINE)
    info = {"bound": bound[-1].strip() if bound else None}
    try:
        link = _sibling("netgun_pwm_link").status(PING_REPLY_S)
    except Exception as exc:  # noqa: BLE001 —— 旧部署缺脚本只降级本字段
        link = {"ok": False, "error": str(exc)}
    info["ok"] = bool(link.get("ok"))
    if not info["ok"]:
        info["error"] = link.get("error") or "击发服务不可用"
        return info
    config = link.get("config") or {}
    info["ready"] = bool(link.get("ready"))
    info["link"] = link.get("link") or {}
    info["config"] = {key: config.get(key) for key in ("ok", "problems", "missing", "warnings")}
    info["options"] = link.get("options") or {}
    info["busy"] = bool(link.get("busy"))
    last = link.get("last_fire")
    try:
        started = os.path.getmtime(PID_FILE)  # 本次启动时刻：更早的击发（台架测试等）不算
    except OSError:
        started = None
    if isinstance(last, dict) and (started is None or float(last.get("time") or 0) >= started):
        info["last_fire"] = {key: last.get(key) for key in (
            "ok", "pending", "mode", "shot_id", "refused", "result_name", "ack_ms", "attempts",
            "elapsed_ms", "pwm_us", "reason_text", "time")}
    return info


def _pid_with_cmdline(pid_file, needle):
    try:
        with open(pid_file) as f:
            pid = int(f.read().strip())
        with open("/proc/%d/cmdline" % pid, "rb") as f:
            cmdline = f.read().replace(b"\0", b" ").decode(errors="replace")
    except (OSError, ValueError):
        return None
    return pid if needle in cmdline else None


def _dir_size_mb(path):
    total = 0
    for root, _, files in os.walk(path):
        for name in files:
            try:
                total += os.path.getsize(os.path.join(root, name))
            except OSError:
                continue
    return round(total / 1048576.0, 1)


def review_status():
    """本次复盘记录（run_interceptor_stack.sh start 建立，stop 收尾）。"""
    info = {}
    try:
        info["free_mb"] = int(shutil.disk_usage(LOG_ROOT).free / 1048576)
    except OSError:
        pass
    try:
        with open(REVIEW_FILE) as f:
            current = f.read().strip()
    except OSError:
        current = ""
    if not current or not os.path.isdir(current):
        dirs = sorted(d for d in glob.glob(os.path.join(LOG_ROOT, "review_*"))
                      if os.path.isdir(d))
        if dirs:
            info["last"] = os.path.basename(dirs[-1])
        return info
    info["dir"] = os.path.basename(current)
    info["recording"] = _pid_with_cmdline(BAG_PID_FILE, "bag record") is not None
    info["size_mb"] = _dir_size_mb(current)
    try:
        with open(os.path.join(current, "bag_record.log"), errors="replace") as f:
            bag_log = f.read()
    except OSError:
        bag_log = ""
    info["topics"] = len(set(re.findall(r"Subscribed to topic '([^']+)'", bag_log)))
    notes = []
    try:
        with open(os.path.join(current, "run_info.txt"), errors="replace") as f:
            notes = [ln.strip() for ln in f if ln.startswith("WARN")]
    except OSError:
        pass
    if not info["recording"]:
        # 启动时的告警（如磁盘余量不足不录 bag）优先，其次录制器最后一条错误
        notes += [ln.strip() for ln in bag_log.splitlines()
                  if re.search(r"ERROR|error|Error|Traceback", ln)][-1:]
    if notes:
        info["note"] = notes[0][:300]
    return info


def main():
    result = {"utc": time.time()}
    running, pid = stack_pid()
    result["running"] = running
    result["pid"] = pid
    result["review"] = review_status()
    if running:
        result["fire_link"] = fire_link_status()
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
    if running:
        result["trigger"] = trigger_status(text)

    lines = [ln for ln in text.splitlines()
             if ln.strip() and not ln.startswith(("NETGUN_GEOMETRY ", "NETGUN_REVIEW ", "NETGUN_TRIGGER "))]
    result["tail"] = lines[-8:]
    print(json.dumps(result, ensure_ascii=False))


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:  # noqa: BLE001 —— 轮询路径必须永不抛栈
        print(json.dumps({"error": str(exc)}, ensure_ascii=False))
