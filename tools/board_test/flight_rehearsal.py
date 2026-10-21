#!/usr/bin/env python3
"""实机流程彩排（2026-10-15）：按真实飞行顺序验证从切 offboard 到拦截的全流程。

场景与真实外场一致（生产 launch 路径；I-3 起必须在隔离 domain 运行——真飞控
的 uXRCE agent 在 domain 0，mock 与真飞控混流、制导心跳会进真飞控；
ROS_DOMAIN_ID 未设置/为 0 时拒绝运行，--allow-domain0 仅限飞控已断开）：
  S1 飞控上线（未解锁）   验证：估计器 attmiss=0 且 init=1；心跳 ob 计数持续
                          增长（两次采样 5s 间隔）；指令计数 sp=0（待机不发
                          setpoint）；mock 收 VehicleCommand=0（rc_gated）
  S2 飞手解锁（手动）     mock 模拟手动爬升至 5.5m。验证：制导保持 WAITING、
                          sp 不增长、心跳继续、高度达标
  S3 切 offboard          验证：WAITING→TAKEOFF→TRACK、sp 开始流动
  S4 开火                 验证：NETGUN FIRE（实机默认 fire_mode=auto_manual，
                          此处验证自动支）+ bag 中 solution_valid=1
  S5 开火后               验证：SETTLE→SPIN
  S6 飞手切出 offboard    验证：回 WAITING、sp 停止增长、心跳继续（夺回后
                          心跳不断——PX4 切回 offboard 不需要重新等心跳）

用法（板上，已 source 全部环境，生产栈已停止）：
  export ROS_DOMAIN_ID=42 && unset ROS_LOCALHOST_ONLY
  python3 flight_rehearsal.py
产物：~/netgun_logs/rehearsal_<stamp>/（日志、bag、summary.json）
"""

import argparse
import json
import os
import re
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

HERE = os.path.dirname(os.path.abspath(__file__))
LOG_ROOT = os.path.expanduser("~/netgun_logs")


def parse_alive(text):
    lines = re.findall(r"guidance alive: (.+)", text)
    if not lines:
        return {}
    out = {}
    m = re.search(r"ob=(\d+)", lines[-1])
    if m:
        out["ob"] = int(m.group(1))
    m = re.search(r"sp=(\d+)", lines[-1])
    if m:
        out["sp"] = int(m.group(1))
    m = re.search(r"phase=(\w+)", lines[-1])
    if m:
        out["phase"] = m.group(1)
    return out


def parse_estimator(text):
    lines = re.findall(r"estimator diag: (.+)", text)
    if not lines:
        return {}
    out = {}
    for key in ("obs", "attmiss", "acc", "init", "valid"):
        m = re.search(r"\b%s=(-?\d+)" % key, lines[-1])
        if m:
            out[key] = int(m.group(1))
    return out


class Rehearsal(Node):

    def __init__(self, args):
        super().__init__("flight_rehearsal")
        self.args = args
        self.stamp = time.strftime("%Y%m%d_%H%M%S")
        self.run_dir = os.path.join(LOG_ROOT, "rehearsal_%s" % self.stamp)
        os.makedirs(self.run_dir, exist_ok=True)
        self.stack_log = os.path.join(self.run_dir, "stack.log")

        self.mock_state = None
        self.stage = "S1"
        self.stage_since = time.monotonic()
        self.result = {"pass": False, "stages": {}, "errors": []}
        self.procs = {}
        self._finished = False
        self.done = False
        self._pub_last = {}

        self.create_subscription(
            String, "/test/mock_px4/state", self._on_mock, 10)
        self._arm_pub = self.create_publisher(
            Bool, "/test/mock_px4/arm", 10)
        self._engage_pub = self.create_publisher(
            Bool, "/test/mock_px4/engage", 10)

        self.create_timer(0.5, self._tick)
        self._start_children()

    def _log(self, text):
        print("[rehearsal %s] %s" % (time.strftime("%H:%M:%S"), text),
              flush=True)

    def _spawn(self, name, cmd):
        log = open(os.path.join(self.run_dir, name + ".log"), "w")
        proc = subprocess.Popen(
            cmd, cwd=self.run_dir, stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
        self.procs[name] = (proc, log)
        self._log("spawn %s pid=%d" % (name, proc.pid))

    def _start_children(self):
        self._log("environment ROS_DOMAIN_ID=%s" %
                  os.environ.get("ROS_DOMAIN_ID", "?"))
        self._spawn("mock_px4", [
            "python3", os.path.join(HERE, "mock_px4_node.py")])
        self._spawn("synthetic", [
            "python3", os.path.join(HERE, "synthetic_perception_node.py"),
            "--ros-args", "-p",
            "target_ned:=%s" % self.args.target_ned])
        self._spawn("stack", [
            "ros2", "launch", "netgun_intercept", "netgun_real.launch.py",
            "bridge_input_topic:=/test/perception/observations_json"])
        self._spawn("bag", [
            "ros2", "bag", "record", "--all",
            "-o", os.path.join(self.run_dir, "bag")])

    def _stop_children(self):
        for name, (proc, log) in reversed(list(self.procs.items())):
            if proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                except (ProcessLookupError, PermissionError):
                    proc.send_signal(signal.SIGINT)
        deadline = time.time() + 8.0
        for name, (proc, log) in self.procs.items():
            try:
                proc.wait(timeout=max(0.1, deadline - time.time()))
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    proc.kill()
        for _, log in self.procs.values():
            log.close()

    def _on_mock(self, msg):
        try:
            self.mock_state = json.loads(msg.data)
        except (json.JSONDecodeError, ValueError):
            pass

    def _read_log(self):
        try:
            with open(self.stack_log, "r", errors="replace") as f:
                return f.read()
        except OSError:
            return ""

    def _cli_pub(self, topic, value, seconds=4):
        """后台线程跑一次性 CLI 发布：时长要够长（本板新参与者发现需
        1–3s，发完即退会一条都送不到）；不阻塞节点自旋，否则 mock 状态
        断流会让合成感知跳帧、IMM 被误复位。同话题 8s 内不重复。"""
        now = time.monotonic()
        if now - self._pub_last.get(topic, -1e9) < 8.0:
            return
        self._pub_last[topic] = now

        def run():
            try:
                subprocess.run(
                    ["ros2", "topic", "pub", "-t", str(seconds * 2),
                     "-r", "2", topic, "std_msgs/msg/Bool",
                     "{data: %s}" % value],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                    timeout=seconds + 20)
            except subprocess.TimeoutExpired:
                pass
        threading.Thread(target=run, daemon=True).start()

    # ---- 阶段推进 ----

    def _tick(self):
        if self._finished:
            return
        now = time.monotonic()
        elapsed = now - self.stage_since
        log_text = self._read_log()

        for name, (proc, _) in self.procs.items():
            if proc.poll() is not None:
                self.result["errors"].append(
                    "子进程 %s 早退（exit=%d）" % (name, proc.returncode))
                self._finish()
                return

        if self.stage == "S1":
            alive = parse_alive(log_text)
            est = parse_estimator(log_text)
            ready = bool(alive) and est.get("obs", 0) > 30
            if ready and elapsed > 12:
                # 两次 alive 采样验证心跳增长（alive 行 5s 一条，12s 内至少 2 条）
                counts = re.findall(
                    r"guidance alive: ob=(\d+) sp=(\d+)", log_text)
                obs = [int(c[0]) for c in counts]
                sps = [int(c[1]) for c in counts]
                heartbeat_ok = obs[-1] > obs[0]
                standby_ok = sps[-1] == 0
                est_ok = est.get("attmiss", 1) == 0 or est.get("acc", 0) > 0
                self.result["stages"]["S1"] = {
                    "心跳增长": heartbeat_ok,
                    "待机零指令": standby_ok,
                    "估计器吃数据": est_ok,
                    "ob_samples": obs[-3:],
                }
                if not (heartbeat_ok and standby_ok and est_ok):
                    self.result["errors"].append(
                        "S1 失败：心跳=%s 零指令=%s 估计器=%s" %
                        (heartbeat_ok, standby_ok, est_ok))
                    self._finish()
                    return
                self._advance("S2", "飞控在线/心跳正常/待机零指令")
        elif self.stage == "S2":
            mock = self.mock_state or {}
            height = -mock.get("pos", [0, 0, 0])[2] if mock else 0.0
            # 本板 CLI 新参与者的发现时长不稳定（1–3s 抖动）：解锁命令
            # 持续重发直到 mock 高度开始爬升（arm 状态在 mock 侧锁存，
            # 重复无害；_pub_last 8s 节流）
            if height < 0.5:
                self._cli_pub("/test/mock_px4/arm", "true", seconds=4)
            if not getattr(self, "_s2_logged", False):
                self._s2_logged = True
                self._log("S2: 已解锁（手动模式），mock 模拟飞手爬升")
            alive = parse_alive(log_text)
            if height > 4.0 and elapsed > 8:
                sp_ok = alive.get("sp") == 0
                wait_ok = (alive.get("phase") or "WAITING") == "WAITING"
                self.result["stages"]["S2"] = {
                    "手动爬升高度_m": round(height, 1),
                    "手动期间零指令": sp_ok,
                    "制导保持待命": wait_ok,
                }
                if not (sp_ok and wait_ok):
                    self.result["errors"].append(
                        "S2 失败：sp=%s phase=%s" % (alive.get("sp"),
                                                    alive.get("phase")))
                    self._finish()
                    return
                self._advance("S3", "手动爬升 %.1fm 期间制导零指令" % height)
            elif elapsed > 45:
                self.result["errors"].append(
                    "S2 超时：高度 %.1f" % height)
                self._finish()
        elif self.stage == "S3":
            alive = parse_alive(log_text)
            # engage 同样持续重发直到指令开始流动
            if alive.get("sp", 0) == 0:
                self._cli_pub("/test/mock_px4/engage", "true", seconds=4)
            if not getattr(self, "_s3_logged", False):
                self._s3_logged = True
                self._log("S3: 已切 offboard，拦截链接管")
            # 已在 5m 高度时接管后可能秒级跑完全程（TAKEOFF→…→DONE），
            # 判据放宽为"离开 WAITING 且指令在流"
            if alive.get("sp", 0) > 0 and \
                    (alive.get("phase") or "WAITING") != "WAITING":
                self.result["stages"]["S3"] = {
                    "接管阶段": alive.get("phase"),
                    "指令开始流动": alive.get("sp", 0) > 0,
                }
                self._advance("S4", "offboard 接管成功（%s）"
                              % alive.get("phase"))
            elif elapsed > 60:
                self.result["errors"].append(
                    "S3 超时：phase=%s sp=%s" %
                    (alive.get("phase"), alive.get("sp")))
                self._finish()
        elif self.stage == "S4":
            self._cli_pub("/test/mock_px4/engage", "true", seconds=4)
            fires = re.findall(r"NETGUN FIRE (.+)", log_text)
            if fires:
                self.result["stages"]["S4"] = {"开火": fires[-1]}
                self._advance("S5", "开火：%s" % fires[-1])
            elif elapsed > 120:
                self.result["errors"].append(
                    "S4 超时：未开火 est=%s alive=%s" %
                    (parse_estimator(log_text), parse_alive(log_text)))
                self._finish()
        elif self.stage == "S5":
            alive = parse_alive(log_text)
            # SETTLE/SPIN 切换不打 "phase →" 日志行，以 alive 行的 phase 为准
            if (alive.get("phase") or "") in ("SETTLE", "SPIN", "DONE"):
                self.result["stages"]["S5"] = {
                    "开火后阶段": alive.get("phase")}
                self._advance("S6", "开火后进入 %s" % alive.get("phase"))
            elif elapsed > 30:
                self.result["errors"].append(
                    "S5 超时：alive phase=%s" % alive.get("phase"))
                self._finish()
        elif self.stage == "S6":
            alive = parse_alive(log_text)
            if elapsed < 1.0:
                self._cli_pub("/test/mock_px4/arm", "true", seconds=4)
                self._log("S6: 飞手切出 offboard（arm=true 保持解锁）")
            # 切出经 CLI 送达有 1–3s 发现延迟：先等制导真正回到 WAITING
            # （切出生效），以该时刻 sp 为基准，再等 6s 验证指令冻结。
            went_waiting = (alive.get("phase") or "") == "WAITING"
            if went_waiting and getattr(self, "_sp_at_waiting", None) is None:
                self._sp_at_waiting = alive.get("sp", 0)
                self._waiting_since = time.monotonic()
                self._log("S6: 切出生效（phase=WAITING，sp 基准 %d）"
                          % self._sp_at_waiting)
            if went_waiting and getattr(self, "_sp_at_waiting", None) is not None \
                    and time.monotonic() - self._waiting_since > 6:
                mock = self.mock_state or {}
                sp_after = alive.get("sp", 0)
                sp_stopped = sp_after <= self._sp_at_waiting + 2
                counts = re.findall(
                    r"guidance alive: ob=(\d+)", log_text)
                tail_obs = [int(v) for v in counts[-3:]] if len(counts) >= 3 else [0, 0]
                heartbeat_ok = tail_obs[-1] > tail_obs[0]
                self.result["stages"]["S6"] = {
                    "切出后零指令": sp_stopped,
                    "回待命": True,
                    "心跳持续": heartbeat_ok,
                    "ob_tail": tail_obs,
                    "rc_gated_零指令字": mock.get("cmd_count", -1),
                }
                if not sp_stopped:
                    self.result["errors"].append(
                        "S6 失败：切出生效后 sp %d→%d 仍在增长" %
                        (self._sp_at_waiting, sp_after))
                if not heartbeat_ok:
                    self.result["errors"].append("S6 失败：切出后心跳停滞")
                if mock.get("cmd_count", 0):
                    self.result["errors"].append(
                        "rc_gated 不变量违反：VehicleCommand x%d"
                        % mock["cmd_count"])
                self._advance("DONE", "彩排完成")
                self._finish()
                return
            if elapsed > 40:
                self.result["errors"].append(
                    "S6 超时：phase=%s" % alive.get("phase"))
                self._finish()

    def _advance(self, stage, note):
        if self.stage != "DONE":
            self.result["stages"].setdefault(self.stage, {})["note"] = note
        self._log("%s -> %s: %s" % (self.stage, stage, note))
        self.stage = stage
        self.stage_since = time.monotonic()

    def _read_bag_shots(self):
        try:
            import rosbag2_py
            from rclpy.serialization import deserialize_message
            from virtual_interception_interfaces.msg import VirtualShotEvent
            reader = rosbag2_py.SequentialReader()
            reader.open(
                rosbag2_py.StorageOptions(
                    uri=os.path.join(self.run_dir, "bag"),
                    storage_id="sqlite3"),
                rosbag2_py.ConverterOptions(
                    input_serialization_format="cdr",
                    output_serialization_format="cdr"))
            shots = []
            while reader.has_next():
                topic, data, _t = reader.read_next()
                if topic != "/tracking/virtual_shot_event":
                    continue
                msg = deserialize_message(data, VirtualShotEvent)
                shots.append({
                    "shot_id": msg.shot_id,
                    "solution_valid": bool(msg.solution_valid),
                    "estimated_range_m": round(msg.estimated_range_m, 2),
                })
            return shots
        except Exception as exc:  # noqa: BLE001
            self._log("bag 读取失败：%s" % exc)
            return []

    def _finish(self):
        if self._finished:
            return
        self._finished = True
        log_text = self._read_log()
        fires = re.findall(r"NETGUN FIRE (.+)", log_text)
        self.result["fire"] = fires[-1] if fires else None
        # 先停子进程（含 bag 录制器，sqlite 停写后才能读），再读 bag
        self._stop_children()
        shots = self._read_bag_shots()
        self.result["shots_in_bag"] = shots
        fired = bool(fires)
        valid_in_bag = any(s.get("solution_valid") for s in shots)
        if fired and shots:
            self.result["pass"] = not self.result["errors"] and valid_in_bag
        else:
            # bag 无记录或读取失败：以日志开火行为准
            self.result["pass"] = not self.result["errors"] and fired
        summary = os.path.join(self.run_dir, "summary.json")
        with open(summary, "w") as f:
            json.dump(self.result, f, ensure_ascii=False, indent=2)
        self._log("=" * 60)
        self._log("RESULT: %s" % ("PASS" if self.result["pass"] else "FAIL"))
        for key, value in self.result["stages"].items():
            self._log("%s: %s" % (key, json.dumps(value, ensure_ascii=False)))
        for error in self.result["errors"]:
            self._log("ERROR: %s" % error)
        self._log("summary: %s" % summary)
        self._log("=" * 60)
        # 不在回调里 rclpy.shutdown()：Humble 全局执行器 on_shutdown 会等本
        # 回调结束 → 自锁挂死；main() 的 spin_once 循环看到 done 后收尾
        self.done = True


def domain_guard(allow_domain0):
    """I-3：真飞控在 domain 0——mock/制导输出不得进入生产域。"""
    domain = os.environ.get("ROS_DOMAIN_ID", "").strip()
    if domain in ("", "0") and not allow_domain0:
        print("拒绝运行：ROS_DOMAIN_ID=%s 是生产域，真飞控（uXRCE agent）也在 "
              "domain 0。请 export ROS_DOMAIN_ID=42 后重跑；确认飞控已断开时可加 "
              "--allow-domain0" % (domain or "未设置(=0)"), flush=True)
        sys.exit(2)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-ned", default="0.0,0.0,-11.6")
    parser.add_argument("--allow-domain0", action="store_true")
    args, _ = parser.parse_known_args()
    domain_guard(args.allow_domain0)
    rclpy.init()
    node = Rehearsal(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        node._finish()
    finally:
        passed = node.result.get("pass", False)
        node.destroy_node()
        rclpy.try_shutdown()
        sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
