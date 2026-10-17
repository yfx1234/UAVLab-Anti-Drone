#!/usr/bin/env python3
"""无硬件全链路测试编排器（2026-10-15，日志驱动版）。

在 domain 0（生产域）里，用真实 netgun_real.launch.py 四节点（桥+估计器+
制导+火控）+ mock_px4 + 合成感知跑完整感知→拦截流程，以
/tracking/virtual_shot_event（开火软事件）为终点判据。

设计说明（为什么是日志/CLI 驱动而非订阅驱动）：本板 FastDDS 对"先建订阅、
后加入发布者"的若干端点组合存在匹配迟滞（mock/CLI 这类后加入参与者可靠，
编排器常驻订阅收不到栈节点 debug）。因此：
  - 阶段判定读 stack.log 的节点 stdout（估计器 diag 2Hz、制导 phase 转换、
    火控 NETGUN FIRE 行都在 stdout）；
  - engage 用一次性 ros2 topic pub 子进程（每次都是新参与者，路径已验证）；
  - 完整证据事后从 bag 读取（rosbag2_py 文件读取，不依赖 DDS）。

阶段与判据：
  P0 四节点进程启动且估计器开始打印 diag（≤45s）
  P1 估计器 init=1（≤20s）
  P2 engage 后制导 phase → TAKEOFF/TRACK（≤90s，含爬升与收敛）
  P3 火控 NETGUN FIRE（≤120s）
  不变量：mock 全程收到 VehicleCommand=0（rc_gated 承诺）

用法（板上，已 source 全部环境；生产拦截栈必须先停止）：
  export ROS_DOMAIN_ID=0 && unset ROS_LOCALHOST_ONLY
  python3 fullchain_test.py [--target-ned 0.0,0.0,-11.6]

产物：~/netgun_logs/fullchain_<stamp>/（各进程日志、bag、summary.json）
"""

import argparse
import json
import os
import re
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

HERE = os.path.dirname(os.path.abspath(__file__))
LOG_ROOT = os.path.expanduser("~/netgun_logs")


class Orchestrator(Node):

    def __init__(self, args):
        super().__init__("fullchain_orchestrator")
        self.args = args
        self.stamp = time.strftime("%Y%m%d_%H%M%S")
        self.run_dir = os.path.join(LOG_ROOT, "fullchain_%s" % self.stamp)
        os.makedirs(self.run_dir, exist_ok=True)
        self.stack_log_path = os.path.join(self.run_dir, "stack.log")

        self.weapon_auth = None
        self.mock_state = None
        self.engage_on = False
        self.last_engage_cli_s = -1e9
        self.phase = "P0"
        self.phase_since = time.monotonic()
        self.shot_line = None

        self.create_subscription(
            String, "/test/mock_px4/state", self._on_mock, 10)

        self.procs = {}
        self.result = {"pass": False, "phases": {}, "errors": []}
        self.create_timer(0.5, self._tick)
        self._start_children()

    # ---- 子进程管理 ----

    def _log(self, text):
        print("[fullchain %s] %s" % (time.strftime("%H:%M:%S"), text),
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
        if os.environ.get("ROS_DOMAIN_ID") in (None, "", "0"):
            self._log("NOTE: 在生产 domain 0 运行——确认拦截栈已停止")
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

    # ---- 外部交互 ----

    def _on_mock(self, msg):
        try:
            self.mock_state = json.loads(msg.data)
        except (json.JSONDecodeError, ValueError):
            pass

    def _engage_cli(self, value):
        """一次性 CLI 发布（新参与者路径在本板已验证可靠），10s 内不重复。"""
        now = time.monotonic()
        if now - self.last_engage_cli_s < 10.0 and self.engage_on == value:
            return
        self.last_engage_cli_s = now
        self.engage_on = value
        try:
            subprocess.run(
                ["ros2", "topic", "pub", "-t", "3", "-r", "1",
                 "/test/mock_px4/engage", "std_msgs/msg/Bool",
                 "{data: %s}" % ("true" if value else "false")],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                timeout=12)
        except subprocess.TimeoutExpired:
            pass
        self._log("engage -> %s (cli)" % value)

    def _read_log(self):
        try:
            with open(self.stack_log_path, "r", errors="replace") as f:
                return f.read()
        except OSError:
            return ""

    def _latest_estimator_diag(self, log_text):
        matches = re.findall(r"estimator diag: (.+)", log_text)
        return matches[-1] if matches else ""

    # ---- 状态机 ----

    def _tick(self):
        now = time.monotonic()
        elapsed = now - self.phase_since
        log_text = self._read_log()

        for name, (proc, _) in self.procs.items():
            if proc.poll() is not None and self.phase not in ("DONE",):
                self.result["errors"].append(
                    "子进程 %s 早退（exit=%d），见 %s/%s.log"
                    % (name, proc.returncode, self.run_dir, name))
                self._finish()
                return

        if self.phase == "P0":
            started = log_text.count("process started with pid")
            diag = self._latest_estimator_diag(log_text)
            if started >= 4 and diag:
                self._advance("P1", "四节点已启动，估计器 diag 流动")
            elif elapsed > 45:
                self.result["errors"].append(
                    "P0 超时：started=%d diag=%s" % (started, diag != ""))
                self._finish()
        elif self.phase == "P1":
            diag = self._latest_estimator_diag(log_text)
            if re.search(r"\binit=1\b", diag):
                self._advance("P2", "估计器 init=1（%s）" % diag)
            elif elapsed > 20:
                self.result["errors"].append("P1 超时：%s" % diag)
                self._finish()
        elif self.phase == "P2":
            self._engage_cli(True)
            if re.search(r"phase → (TAKEOFF|TRACK|AIM)", log_text):
                phase_line = re.findall(r"phase → (\w+)", log_text)
                self._advance("P3", "制导 %s" % phase_line[-1] if phase_line
                              else "制导已进入交战")
            elif elapsed > 90:
                self.result["errors"].append(
                    "P2 超时：制导未进入交战；est=%s" %
                    self._latest_estimator_diag(log_text))
                self._finish()
        elif self.phase == "P3":
            self._engage_cli(True)
            fire = re.findall(r"NETGUN FIRE (.+)", log_text)
            if fire:
                self.shot_line = fire[-1]
                self.result["phases"]["P3"] = {
                    "fire_log": self.shot_line,
                    "elapsed_s": round(elapsed, 1)}
                self._advance("P4", "开火：%s" % self.shot_line)
            elif elapsed > 120:
                self.result["errors"].append(
                    "P3 超时：未开火。est=%s" %
                    self._latest_estimator_diag(log_text))
                self._finish()
        elif self.phase == "P4":
            if elapsed > 6.0:  # 观察 SETTLE/SPIN
                self._engage_cli(False)
                self._advance("DONE", "测试完成")

    def _advance(self, phase, note):
        self.result["phases"][self.phase] = {
            "note": note,
            "elapsed_s": round(time.monotonic() - self.phase_since, 1)}
        self._log("%s -> %s: %s" % (self.phase, phase, note))
        self.phase = phase
        self.phase_since = time.monotonic()
        if phase == "DONE":
            self._finish()

    # ---- 收尾 ----

    def _read_bag_shots(self):
        """事后从 bag 读取 VirtualShotEvent 明细（文件读取，不依赖 DDS）。"""
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
                    "estimated_range_m": msg.estimated_range_m,
                    "actual_attack_axis_error_rad":
                        msg.actual_attack_axis_error_rad,
                    "predicted_time_of_flight_s":
                        msg.predicted_time_of_flight_s,
                })
            return shots
        except Exception as exc:  # noqa: BLE001
            self._log("bag 读取失败（不影响判定）：%s" % exc)
            return []

    def _finish(self):
        if getattr(self, "_finished", False):
            return
        self._finished = True
        log_text = self._read_log()
        mock = self.mock_state or {}
        diags = re.findall(r"estimator diag: (.+)", log_text)
        phases_seen = re.findall(r"phase → (\w+)", log_text)
        self.result["final"] = {
            "estimator_diag": diags[-1] if diags else "",
            "guidance_phases": phases_seen,
            "fire_log": self.shot_line,
        }
        self.result["mock"] = {
            "sp_count": mock.get("sp_count"),
            "ob_count": mock.get("ob_count"),
            "cmd_count": mock.get("cmd_count"),
            "final_pos_ned": mock.get("pos"),
        }
        self.result["bag"] = os.path.join(self.run_dir, "bag")
        shots = self._read_bag_shots()
        self.result["shots_in_bag"] = shots
        if shots:
            self.result["pass"] = any(s.get("solution_valid") for s in shots)
        else:
            self.result["pass"] = bool(self.shot_line)
        if mock.get("cmd_count"):
            self.result["errors"].append(
                "rc_gated 不变量被违反：mock 收到 VehicleCommand x%d"
                % mock["cmd_count"])
            self.result["pass"] = False
        self._stop_children()
        summary_path = os.path.join(self.run_dir, "summary.json")
        with open(summary_path, "w") as f:
            json.dump(self.result, f, ensure_ascii=False, indent=2)
        self._log("=" * 60)
        self._log("RESULT: %s" % ("PASS" if self.result["pass"] else "FAIL"))
        self._log("fire=%s" % self.shot_line)
        self._log("mock=%s phases=%s" % (
            self.result["mock"], phases_seen))
        self._log("est=%s" % (diags[-1] if diags else ""))
        self._log("summary: %s" % summary_path)
        self._log("=" * 60)
        rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-ned", default="0.0,0.0,-11.6")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = Orchestrator(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._finish()
    finally:
        passed = node.result.get("pass", False)
        node.destroy_node()
        sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
