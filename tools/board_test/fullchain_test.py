#!/usr/bin/env python3
"""无硬件全链路测试编排器（2026-10-15，日志驱动版；I-3 2026-10-21 增补开火模式）。

在隔离 domain（默认要求非 0，见下）里，用真实 netgun_real.launch.py 四节点
（桥+估计器+制导+火控）+ mock_px4 + 合成感知跑完整感知→拦截流程，以
/tracking/virtual_shot_event（开火软事件）为终点判据。

I-3 安全约束：真飞控接上后 uXRCE agent 就在 domain 0——在 domain 0 跑本测试
会让 mock_px4 与真飞控话题混流、制导心跳/setpoint 进入真飞控。因此
ROS_DOMAIN_ID 未设置或为 0 时拒绝运行（--allow-domain0 仅限确认飞控已断开）。

设计说明（为什么是日志/CLI 驱动而非订阅驱动）：本板 FastDDS 对"先建订阅、
后加入发布者"的若干端点组合存在匹配迟滞（mock/CLI 这类后加入参与者可靠，
编排器常驻订阅收不到栈节点 debug）。因此：
  - 阶段判定读 stack.log 的节点 stdout（估计器 diag 2Hz、制导 phase 转换、
    火控 NETGUN FIRE 行都在 stdout）；
  - engage 用一次性 ros2 topic pub 子进程（每次都是新参与者，路径已验证）；
  - 完整证据事后从 bag 读取（rosbag2_py 文件读取，不依赖 DDS）。

阶段与判据：
  P0 四节点进程启动且估计器开始打印 diag（≤45s）
  P1 估计器 init=1（≤20s）；期间按一次发射按钮（未解锁）→ 必须被拒
     （NOT_ARMED/FCU_STALE），且不得开火
  P2 engage 后制导 phase → TAKEOFF/TRACK（≤90s，含爬升与收敛）
  P3 火控 NETGUN FIRE（≤120s）：
     --fire auto（默认）  不按按钮，等火控门自动开火（PRIMARY_WINDOW/WINDOW_HOLD）
     --fire manual        launch 把自动门限设为负值（自动永不开火），制导进
                          TRACK 后用面板同款助手 deploy/netgun_fire_button.py
                          fire 按钮兜底（reason=MANUAL）
  P4 开火后再按一次（原始话题）→ 必须 "already fired"；助手 status 报 latched
  不变量：mock 全程收到 VehicleCommand=0（rc_gated 承诺）
  单发：stack.log 恰 1 条 NETGUN FIRE，bag 恰 1 个 VirtualShotEvent
  --trigger-hook：trigger_command 装订为干跑脚本 netgun_trigger_dryrun.sh
     （只写日志不碰硬件），验证恰执行 1 次且 "trigger command done rc=0"
  常驻发射通道（2026-10-23）：与生产同款 netgun_fire_button.py serve
     --parent-pid=<stack> 随栈起（套接字在本次产物目录，按域隔离）；助手按键
     须走常驻通道（path=server），fire_contract 记录按键到回复耗时 elapsed_ms

用法（板上，已 source 全部环境；生产拦截栈必须先停止）：
  export ROS_DOMAIN_ID=42 && unset ROS_LOCALHOST_ONLY
  python3 fullchain_test.py [--fire auto|manual] [--trigger-hook] \\
      [--target-ned 0.0,0.0,-11.6]

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
REPO_ROOT = os.path.dirname(os.path.dirname(HERE))
FIRE_BUTTON = os.path.join(REPO_ROOT, "deploy", "netgun_fire_button.py")
TRIGGER_DRYRUN = os.path.join(HERE, "netgun_trigger_dryrun.sh")
LOG_ROOT = os.path.expanduser("~/netgun_logs")


def domain_guard(allow_domain0):
    """I-3：真飞控在 domain 0——mock/制导输出不得进入生产域。"""
    domain = os.environ.get("ROS_DOMAIN_ID", "").strip()
    if domain in ("", "0") and not allow_domain0:
        print("拒绝运行：ROS_DOMAIN_ID=%s 是生产域，真飞控（uXRCE agent）也在 "
              "domain 0——mock_px4 会与真飞控话题混流、制导心跳/setpoint 会进入"
              "真飞控。请 export ROS_DOMAIN_ID=42 后重跑；确认飞控已断开时可加 "
              "--allow-domain0" % (domain or "未设置(=0)"), flush=True)
        sys.exit(2)


class Orchestrator(Node):

    def __init__(self, args):
        super().__init__("fullchain_orchestrator")
        self.args = args
        self.done = False
        self.stamp = time.strftime("%Y%m%d_%H%M%S")
        self.run_dir = os.path.join(LOG_ROOT, "fullchain_%s" % self.stamp)
        os.makedirs(self.run_dir, exist_ok=True)
        self.stack_log_path = os.path.join(self.run_dir, "stack.log")

        self.weapon_auth = None
        self.mock_state = None
        self.engage_on = False
        self.last_engage_cli_s = -1e9
        self.last_fire_cli_s = -1e9
        self.phase = "P0"
        self.phase_since = time.monotonic()
        self.shot_line = None
        self.pre_engage_pressed = False
        self.post_shot_checked = False
        self.button_results = []
        self.trigger_log_path = os.path.join(self.run_dir, "trigger_dryrun.log")

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
            self._log("WARNING: 在生产 domain 0 运行（--allow-domain0）——"
                      "确认拦截栈已停止且飞控已断开")
        self._log("fire=%s trigger_hook=%s" % (
            self.args.fire, self.args.trigger_hook))
        self._spawn("mock_px4", [
            "python3", os.path.join(HERE, "mock_px4_node.py")])
        self._spawn("synthetic", [
            "python3", os.path.join(HERE, "synthetic_perception_node.py"),
            "--ros-args", "-p",
            "target_ned:=%s" % self.args.target_ned])
        stack_cmd = [
            "ros2", "launch", "netgun_intercept", "netgun_real.launch.py",
            "bridge_input_topic:=/test/perception/observations_json"]
        if self.args.fire == "manual":
            # 自动门限设负值 = 自动永不开火，只验证手动兜底路径
            # （fire_mode 保持生产默认 auto_manual）
            stack_cmd += ["primary_axis_error_rad:=-1.0",
                          "window_axis_error_rad:=-1.0"]
        if self.args.trigger_hook:
            os.environ["NETGUN_TRIGGER_LOG"] = self.trigger_log_path
            stack_cmd.append("trigger_command:=bash %s" % TRIGGER_DRYRUN)
        self._spawn("stack", stack_cmd)
        # 生产同款常驻发射通道：助手与通道共用本目录的套接字（不碰生产链的 d0 套接字）
        os.environ["NETGUN_FIRE_SOCKET"] = os.path.join(
            self.run_dir, "netgun_fire.sock")
        self._spawn("fire_server", [
            "python3", "-u", FIRE_BUTTON, "serve", "--parent-pid",
            str(self.procs["stack"][0].pid)])
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

    def _fire_raw_cli(self, note):
        """原始话题按键（绕过助手的就绪预判）：直接考 strike 侧门控/闭锁。
        发 3 次：--once 默认只等 1 个匹配订阅者，bag 录制器（record --all）
        先匹配时单发会在火控匹配前发出而丢失（实测 2 轮各丢 1 次）。"""
        try:
            subprocess.run(
                ["ros2", "topic", "pub", "-t", "3", "-r", "2",
                 "--keep-alive", "1.0", "/netgun/fire_command",
                 "std_msgs/msg/Bool", "{data: true}"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                timeout=12)
        except subprocess.TimeoutExpired:
            pass
        self._log("fire_command raw -> pressed (%s)" % note)

    def _fire_button_helper(self, action):
        """面板同款助手（deploy/netgun_fire_button.py），返回其 JSON。"""
        try:
            out = subprocess.run(
                ["python3", FIRE_BUTTON, action], capture_output=True,
                text=True, timeout=40).stdout.strip().splitlines()
            payload = json.loads(out[-1]) if out else {}
        except (subprocess.TimeoutExpired, ValueError, IndexError) as exc:
            payload = {"ok": False, "reason": "助手调用失败：%s" % exc}
        self.button_results.append({"action": action, "result": payload})
        self._log("fire_button %s -> %s" % (
            action, json.dumps(payload, ensure_ascii=False)))
        return payload

    def _fire_button_press(self):
        """模拟操作手：≥5s 一次用助手按键（助手只在 fready=1 时才发布）。"""
        now = time.monotonic()
        if now - self.last_fire_cli_s < 5.0:
            return
        self.last_fire_cli_s = now
        self._fire_button_helper("fire")

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
            if not self.pre_engage_pressed:
                # 未解锁时按键：strike 必须拒绝（NOT_ARMED/FCU_STALE）
                self.pre_engage_pressed = True
                self._fire_raw_cli("pre-engage, expect reject")
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
            if (not fire and self.args.fire == "manual" and
                    re.search(r"phase → TRACK", log_text)):
                # I-3：自动门限已置负（自动永不开火）——制导进 TRACK 后
                # 模拟操作手用面板按钮兜底
                self._fire_button_press()
                log_text = self._read_log()
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
            if not self.post_shot_checked:
                # 开火后再按：strike 必须报 already fired；助手须报 latched
                self.post_shot_checked = True
                self._fire_raw_cli("post-shot, expect already fired")
                self._fire_button_helper("status")
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
        # 先停子进程（bag 录制器 SIGINT 后才写 metadata.yaml、关闭 sqlite），
        # 再读日志终稿与 bag——录制中读 bag 会报 "disk I/O error"
        self._stop_children()
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
        self._check_fire_contract(log_text, shots)
        if mock.get("cmd_count"):
            self.result["errors"].append(
                "rc_gated 不变量被违反：mock 收到 VehicleCommand x%d"
                % mock["cmd_count"])
            self.result["pass"] = False
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
        # 不在回调里 rclpy.shutdown()：Humble 全局执行器的 on_shutdown 会
        # executor.shutdown() 等本回调结束 → 自锁（进程挂死到 timeout）。
        # 由 main() 的 spin_once 循环看到 done 后在回调外收尾。
        self.done = True

    def _check_fire_contract(self, log_text, shots):
        """I-3：开火契约——单发、原因、按键门控/闭锁、击发钩子恰一次。"""
        errors = self.result["errors"]
        fires = re.findall(r"NETGUN FIRE (.+)", log_text)
        checks = {"fire_lines": len(fires), "bag_shots": len(shots)}
        if len(fires) != 1:
            errors.append("单发违反/未开火：NETGUN FIRE 行数=%d" % len(fires))
        if shots and len(shots) != 1:
            errors.append("单发违反：bag 中 VirtualShotEvent=%d" % len(shots))
        expected = ("MANUAL",) if self.args.fire == "manual" else (
            "PRIMARY_WINDOW", "WINDOW_HOLD")
        reason = re.search(r"reason=(\w+)", fires[0]).group(1) if fires else None
        checks["reason"] = reason
        if fires and reason not in expected:
            errors.append("开火原因 %s 不在期望 %s 内" % (reason, expected))
        rejects = re.findall(r"manual fire rejected: (\w+)", log_text)
        checks["manual_rejects"] = rejects
        if not any(r in ("NOT_ARMED", "FCU_STALE") for r in rejects):
            errors.append("未解锁按键未见拒绝日志（期望 NOT_ARMED/FCU_STALE）：%s"
                          % rejects)
        if self.post_shot_checked and "already" not in rejects:
            errors.append("开火后按键未见 already fired 拒绝：%s" % rejects)
        status = [b["result"] for b in self.button_results
                  if b["action"] == "status"]
        checks["helper_status_latched"] = bool(status) and bool(
            status[-1].get("latched"))
        if self.post_shot_checked and not checks["helper_status_latched"]:
            errors.append("开火后助手 status 未报 latched：%s"
                          % (status[-1] if status else None))
        presses = [b["result"] for b in self.button_results
                   if b["action"] == "fire"]
        checks["helper_fire"] = [
            {key: press.get(key) for key in (
                "path", "fired", "elapsed_ms", "publish_ms", "confirm_ms",
                "confirmed_by", "fallback_reason", "reason")}
            for press in presses]
        fired = [press for press in presses if press.get("fired")]
        if self.args.fire == "manual" and fired and fired[0].get("path") != "server":
            errors.append("手动按键未走常驻发射通道：path=%s（%s）"
                          % (fired[0].get("path"), fired[0].get("fallback_reason")))
        if self.args.trigger_hook:
            try:
                with open(self.trigger_log_path) as f:
                    hook_lines = [ln for ln in f.read().splitlines() if ln]
            except OSError:
                hook_lines = []
            checks["trigger_hook_lines"] = hook_lines
            done = re.findall(r"trigger command done rc=0", log_text)
            checks["trigger_done"] = len(done)
            if len(hook_lines) != 1 or len(done) != 1:
                errors.append("击发钩子应恰执行 1 次：脚本记录 %d 行、done %d 次"
                              % (len(hook_lines), len(done)))
            elif "shot_id=1" not in hook_lines[0] or (
                    reason and ("reason=%s" % reason) not in hook_lines[0]):
                errors.append("击发钩子环境变量不符：%s" % hook_lines[0])
        self.result["fire_contract"] = checks
        if errors:
            self.result["pass"] = False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-ned", default="0.0,0.0,-11.6")
    parser.add_argument("--fire", choices=("auto", "manual"), default="auto")
    parser.add_argument("--trigger-hook", action="store_true")
    parser.add_argument("--allow-domain0", action="store_true")
    args, _ = parser.parse_known_args()
    domain_guard(args.allow_domain0)

    rclpy.init()
    node = Orchestrator(args)
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
