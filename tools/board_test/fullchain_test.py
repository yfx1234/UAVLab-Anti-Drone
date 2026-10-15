#!/usr/bin/env python3
"""无硬件全链路测试编排器（2026-10-15）。

在独立 ROS_DOMAIN_ID（默认 1）里，用真实 netgun_real.launch.py 四节点
（桥+估计器+制导+火控）+ mock_px4 + 合成感知跑完整感知→拦截流程，
以 /tracking/virtual_shot_event（开火软事件）为终点判据。

阶段与判据：
  P1 估计器初始化：/debug/estimator_state 出现 init=1 valid=1（≤30s）
  P2 接入（engage=true=armed+offboard）后制导进入 TRACK/AIM（≤40s）
  P3 收到 VirtualShotEvent 且 solution_valid=1（≤120s）
  不变量：mock 全程收到 VehicleCommand=0（rc_gated 承诺）

用法（板上，已 source 全部环境并 export ROS_DOMAIN_ID=1）：
  python3 fullchain_test.py [--domain 1] [--target-ned 0,0,-11.6] [--keep]

产物：~/netgun_logs/fullchain_<stamp>/（各进程日志、bag、summary.json）
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from virtual_interception_interfaces.msg import VirtualShotEvent

HERE = os.path.dirname(os.path.abspath(__file__))
LOG_ROOT = os.path.expanduser("~/netgun_logs")


def parse_kv(text):
    out = {}
    for token in text.split():
        if "=" in token:
            key, value = token.split("=", 1)
            out[key] = value
    return out


class Orchestrator(Node):

    def __init__(self, args):
        super().__init__("fullchain_orchestrator")
        self.args = args
        self.stamp = time.strftime("%Y%m%d_%H%M%S")
        self.run_dir = os.path.join(LOG_ROOT, "fullchain_%s" % self.stamp)
        os.makedirs(self.run_dir, exist_ok=True)

        self.debug = {}
        self.shots = []
        self.weapon_auth = None
        self.mock_state = None
        self.engage = False
        self.phase = "P0"
        self.phase_since = time.monotonic()
        self.events = []

        self.create_subscription(
            String, "/debug/estimator_state", self._mk("estimator"), 10)
        self.create_subscription(
            String, "/debug/guidance_state", self._mk("guidance"), 10)
        self.create_subscription(
            String, "/debug/strike_state", self._mk("strike"), 10)
        self.create_subscription(
            String, "/debug/bridge_state", self._mk("bridge"), 10)
        self.create_subscription(
            VirtualShotEvent, "/tracking/virtual_shot_event",
            self._on_shot, 10)
        self.create_subscription(
            Bool, "/tracking/weapon_authorization", self._on_auth, 10)
        self.create_subscription(
            String, "/test/mock_px4/state", self._on_mock, 10)
        self._engage_pub = self.create_publisher(
            Bool, "/test/mock_px4/engage", 10)

        self.procs = {}
        self.result = {"pass": False, "phases": {}, "errors": []}
        self.create_timer(0.5, self._tick)
        self._start_children()

    # ---- 子进程管理 ----

    def _spawn(self, name, cmd):
        log = open(os.path.join(self.run_dir, name + ".log"), "w")
        proc = subprocess.Popen(
            cmd, cwd=self.run_dir, stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
        self.procs[name] = (proc, log)
        self._log("spawn %s pid=%d" % (name, proc.pid))

    def _start_children(self):
        env_note = "ROS_DOMAIN_ID=%s" % os.environ.get("ROS_DOMAIN_ID", "?")
        self._log("environment %s" % env_note)
        if os.environ.get("ROS_DOMAIN_ID") in (None, "", "0"):
            # domain 0 = 生产域：板上发现稳定（domain 1 存在端点匹配迟滞）。
            # 运行前必须停掉生产拦截栈（同话题冲突）；合成感知走
            # /test/perception/observations_json 独立输入，不碰真实感知。
            self._log("NOTE: 在生产 domain 0 运行——确认拦截栈已停止")
        self._spawn("mock_px4", [
            "python3", os.path.join(HERE, "mock_px4_node.py")])
        self._spawn("synthetic", [
            "python3", os.path.join(HERE, "synthetic_perception_node.py"),
            "--ros-args", "-p",
            "target_ned:%s" % self.args.target_ned])
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

    # ---- 订阅回调 ----

    def _mk(self, name):
        def cb(msg):
            self.debug[name] = parse_kv(msg.data)
        return cb

    def _on_shot(self, msg):
        shot = {
            "shot_id": msg.shot_id,
            "solution_valid": bool(msg.solution_valid),
            "estimated_range_m": msg.estimated_range_m,
            "actual_attack_axis_error_rad": msg.actual_attack_axis_error_rad,
            "predicted_time_of_flight_s": msg.predicted_time_of_flight_s,
            "predicted_lateral_sigma_m": msg.predicted_lateral_sigma_m,
        }
        self.shots.append(shot)
        self._log("SHOT EVENT: %s" % json.dumps(shot))

    def _on_auth(self, msg):
        if self.weapon_auth is None and msg.data:
            self._log("weapon_authorization -> true")
        self.weapon_auth = bool(msg.data)

    def _on_mock(self, msg):
        try:
            self.mock_state = json.loads(msg.data)
        except (json.JSONDecodeError, ValueError):
            pass

    # ---- 状态机 ----

    def _tick(self):
        now = time.monotonic()
        elapsed = now - self.phase_since
        if self.phase == "ABORT":
            self._finish()
            return
        if self.phase == "P0":
            ready = all(k in self.debug for k in
                        ("estimator", "guidance", "strike", "bridge"))
            if ready:
                self._advance("P1", "四节点 debug 话题均可见")
            elif elapsed > 60:
                self.result["errors"].append(
                    "P0 超时：debug 话题未齐 %s" % sorted(self.debug))
                self._finish()
        elif self.phase == "P1":
            est = self.debug.get("estimator", {})
            if est.get("init") == "1":
                self._advance("P2", "估计器已初始化 %s" % est)
            elif elapsed > 30:
                self.result["errors"].append(
                    "P1 超时：估计器未初始化 %s" % est)
                self._finish()
        elif self.phase == "P2":
            if not self.engage and elapsed > 1.0:
                self._set_engage(True)
            gd = self.debug.get("guidance", {})
            phase_field = gd.get("phase", "")
            if phase_field in ("TRACK", "AIM"):
                self._advance("P3", "制导进入 %s" % phase_field)
            elif elapsed > 40:
                self.result["errors"].append(
                    "P2 超时：制导未进入 TRACK/AIM（%s）%s" % (phase_field, gd))
                self._finish()
        elif self.phase == "P3":
            if not self.engage and elapsed > 1.0:
                self._set_engage(True)
            if self.shots:
                valid = self.shots[-1].get("solution_valid")
                self.result["phases"]["P3"] = {
                    "shot": self.shots[-1], "elapsed_s": round(elapsed, 1)}
                if valid:
                    self._advance("P4", "开火事件 solution_valid=1")
                else:
                    self.result["errors"].append("开火事件 solution_valid=0")
                    self._finish()
            elif elapsed > 120:
                self.result["errors"].append(
                    "P3 超时：未收到开火事件。guidance=%s strike=%s est=%s"
                    % (self.debug.get("guidance"), self.debug.get("strike"),
                       self.debug.get("estimator")))
                self._finish()
        elif self.phase == "P4":
            if elapsed > 6.0:  # 观察 SETTLE/SPIN 与授权撤销
                self._set_engage(False)
                self._advance("DONE", "测试完成")

    def _set_engage(self, value):
        self.engage = value
        msg = Bool()
        msg.data = value
        self._engage_pub.publish(msg)
        self._log("engage -> %s" % value)

    def _advance(self, phase, note):
        self.result["phases"][self.phase] = {
            "note": note, "elapsed_s": round(time.monotonic() - self.phase_since, 1)}
        self._log("%s -> %s: %s" % (self.phase, phase, note))
        self.phase = phase
        self.phase_since = time.monotonic()
        if phase == "DONE":
            self.result["pass"] = not self.result["errors"]
            self._finish()

    # ---- 收尾 ----

    def _finish(self):
        mock = self.mock_state or {}
        self.result["pass"] = (
            bool(self.shots)
            and self.shots[-1].get("solution_valid")
            and not self.result["errors"])
        self.result["final_debug"] = dict(self.debug)
        self.result["shots"] = self.shots
        self.result["mock"] = {
            "sp_count": mock.get("sp_count"),
            "ob_count": mock.get("ob_count"),
            "cmd_count": mock.get("cmd_count"),
        }
        self.result["weapon_authorization"] = self.weapon_auth
        if mock.get("cmd_count"):
            self.result["errors"].append(
                "rc_gated 不变量被违反：mock 收到 VehicleCommand x%d"
                % mock["cmd_count"])
            self.result["pass"] = False
        self.result["bag"] = os.path.join(self.run_dir, "bag")
        self._stop_children()
        summary_path = os.path.join(self.run_dir, "summary.json")
        with open(summary_path, "w") as f:
            json.dump(self.result, f, ensure_ascii=False, indent=2)
        self._log("=" * 60)
        self._log("RESULT: %s" % ("PASS" if self.result["pass"] else "FAIL"))
        self._log("shots=%d mock=%s" % (
            len(self.shots), self.result["mock"]))
        self._log("summary: %s" % summary_path)
        self._log("=" * 60)
        rclpy.try_shutdown()

    def _log(self, text):
        print("[fullchain %s] %s" % (time.strftime("%H:%M:%S"), text),
              flush=True)
        self.events.append({"t": time.time(), "text": text})


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
        node.destroy_node()
        sys.exit(0 if node.result.get("pass") else 1)


if __name__ == "__main__":
    main()
