#!/usr/bin/env python3
"""网枪发射按钮的板上助手（I-1 2026-10-19；I-3 2026-10-21 自动+手动兜底；
2026-10-23 常驻发射通道）。

用法（PC 面板经 SSH 调用；也可手动执行）：
  python3 ~/interceptor_ws/deploy/netgun_fire_button.py status
  python3 ~/interceptor_ws/deploy/netgun_fire_button.py fire
  python3 ~/interceptor_ws/deploy/netgun_fire_button.py serve [--parent-pid PID]
  （status/fire 加 --direct 强制走直连路径，测试对照用）

I-3 起实机默认 fire_mode=auto_manual：火控门满足即自动单发；按钮是人工
兜底（自动一直不发时用）。手动门由 strike 逐拍计算并随 /debug/strike_state
发布：fready=1 即此刻按键会发射，mblk=被拦下的首因（NONE/DISABLED/LATCHED/
FCU_STALE/NOT_ARMED/LOW_HEIGHT/TILT/NOT_AUTHORIZED）。

常驻发射通道（2026-10-23）：原实现每次按键现起 python+rclpy，再等 DDS 发现、
首条火控状态与订阅匹配，按键到发布要 2–3 s。现由 run_interceptor_stack.sh
start 随拦截链起一个常驻 serve 进程（ROS 节点 netgun_fire_server）：火控节点
进程运行 3 s 后才建 ROS 节点，火控端点出现后才建状态订阅/发射发布端（本板
FastDDS 对"先建订阅、后来发布者"的组合有匹配迟滞），提前完成匹配；经本机
Unix 套接字 ~/netgun_logs/netgun_fire_d<ROS_DOMAIN_ID>.sock（仅本用户可连）
受理 status/fire/ping。status/fire 先走常驻通道（客户端不需要 ROS 环境，
按键到发布约 0.1 s；开火确认靠 /tracking/virtual_shot_event，不必等 2 Hz
状态）。通道不存在/未就绪/1 s 内无应答时自动退回直连路径（本进程建节点，
约 2–3 s）；通道已受理 fire 后超时不再重发，结果以火控状态为准。

门控与输出语义不变：
status：单行 JSON {"ok","running","fire_mode","manual_enabled","auto_enabled",
  "ready","latched","block","auth","est","range_m","tilt_deg","shots","reason",
  "path"(server|direct),"elapsed_ms"}
fire：最新火控状态 fready=1 才向 /netgun/fire_command 发布一次 true，再等
  火控确认 shots 增加（"fired": true）；未就绪则不发布并给出中文原因。单发闭锁
  在火控侧（逐拍消费、不积压），重复按键天然安全。

直连路径需 rclpy：未 source ROS 时自动经 bash source /opt/ros/humble 重启自身；
serve 需完整拦截链环境（run_interceptor_stack.sh 已 source）。
"""

import json
import os
import re
import signal
import socket
import struct
import subprocess
import sys
import threading
import time

STRIKE_TOPIC = "/debug/strike_state"
FIRE_TOPIC = "/netgun/fire_command"
SHOT_TOPIC = "/tracking/virtual_shot_event"
STRIKE_NODE = "netgun_strike_node"
SERVER_NODE = "netgun_fire_server"
ROS_SETUP = "/opt/ros/humble/setup.bash"
# 直连路径：本板 FastDDS 晚加入订阅者匹配延迟 1-2s+（H-4/preflight 同款经验）：
# 首条火控状态最长等 8s；发布者与订阅同时创建，发现过程并行
SAMPLE_SECONDS = 8.0
MATCH_SECONDS = 3.0     # 等火控订阅匹配发射话题
CONFIRM_SECONDS = 3.0   # 发布后等火控确认（开火事件或 2Hz 状态 shots 增加）
# 常驻通道
STATE_FRESH_S = 1.5     # 火控状态 2Hz：超过 1.5s 未更新视为通道失联（退回直连）
HEAL_STALE_S = 5.0      # 订阅建立后 5s 收不到状态即重建订阅
MATCH_GRACE_S = 3.0     # 发布端建立 3s 后只要求火控已匹配（防其他订阅者 QoS 不兼容）
CONFIRM_HOLD_S = 2.0    # 本通道确认开火后 2s 内，shots 未增加的 2Hz 状态视为旧状态
STRIKE_SETTLE_S = 3.0   # 火控节点进程运行满 3s 再建 ROS 节点（做"晚加入者"）
CONNECT_TIMEOUT_S = 0.5
ACK_TIMEOUT_S = 1.0
REPLY_MARGIN_S = 2.0
MAX_LINE = 65536
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


def ros_domain():
    domain = os.environ.get("ROS_DOMAIN_ID", "").strip() or "0"
    return domain if re.fullmatch(r"\d{1,3}", domain) else "x"


def socket_path():
    """常驻通道套接字：按 ROS 域区分（隔离域测试栈与生产链互不串）。"""
    override = os.environ.get("NETGUN_FIRE_SOCKET", "").strip()
    if override:
        return override
    root = (os.environ.get("NETGUN_LOG_DIR", "").strip()
            or os.path.expanduser("~/netgun_logs"))
    return os.path.join(root, "netgun_fire_d%s.sock" % ros_domain())


def _process_domain(pid):
    with open("/proc/%d/environ" % pid, "rb") as f:
        env = f.read().split(b"\0")
    for item in env:
        if item.startswith(b"ROS_DOMAIN_ID="):
            return item.split(b"=", 1)[1].strip().decode(errors="replace") or "0"
    return "0"


def _pgrep(pattern, timeout_s=5.0):
    try:
        out = subprocess.run(
            ["pgrep", "-f", pattern],
            capture_output=True, text=True, timeout=timeout_s,
        ).stdout.split()
    except (OSError, ValueError, subprocess.SubprocessError):
        return []
    pids = []
    for token in out:
        try:
            pids.append(int(token))
        except ValueError:
            continue
    return pids


def production_launch_pids(timeout_s=5.0):
    """I-3（2026-10-21）：只认生产域（ROS_DOMAIN_ID 为空或 0）的
    netgun_real.launch 进程——隔离域测试栈（fullchain/rehearsal 跑在
    domain 42）不算"拦截链运行中"，免得面板误报运行、误点停止把测试栈
    一并 SIGINT（与 run_interceptor_stack.sh production_launch_pids 同判据）。"""
    pids = []
    for pid in _pgrep("netgun_real[.]launch", timeout_s):
        try:
            if _process_domain(pid) == "0":
                pids.append(pid)
        except (OSError, ValueError):
            continue
    return pids


def stack_running():
    """生产域拦截链 launch 是否存活（与 interceptor_status.py 同判据）。"""
    return bool(production_launch_pids())


def process_age_s(pid):
    with open("/proc/%d/stat" % pid) as f:
        fields = f.read().rsplit(")", 1)[1].split()
    with open("/proc/uptime") as f:
        uptime = float(f.read().split()[0])
    return uptime - int(fields[19]) / os.sysconf("SC_CLK_TCK")


def strike_process_age(domain):
    """本域火控节点进程已运行秒数（多个取最长）；没有则 None。"""
    ages = []
    for pid in _pgrep("netgun_strike_node"):
        try:
            if _process_domain(pid) == domain:
                ages.append(process_age_s(pid))
        except (OSError, ValueError, IndexError):
            continue
    return max(ages) if ages else None


def pid_alive(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


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


def fire_gate(payload):
    """按键前置门（与火控同源的最新状态）：返回拒绝原因，可发射时返回 None。"""
    if not payload["manual_enabled"]:
        return BLOCK_REASONS["DISABLED"]
    if payload["latched"]:
        return BLOCK_REASONS["LATCHED"]
    if not payload["ready"]:
        return "未发射：" + BLOCK_REASONS.get(payload["block"], payload["block"])
    return None


def emit(payload):
    print(json.dumps(payload, ensure_ascii=False), flush=True)


def log_line(text, level="INFO"):
    # 与 ROS 日志同格式（面板原始日志/状态脚本按同一时间戳解析）；
    # 勿含 "NETGUN FIRE"/"netgun diag:" 等状态脚本计数用的关键字
    print("[%s] [%.9f] [%s]: %s" % (level, time.time(), SERVER_NODE, text),
          flush=True)


# ---------------------------------------------------------------- 客户端

class ChannelError(Exception):
    """常驻通道不可用或未受理请求（未发出任何指令）——可安全退回直连。"""


def _read_json_line(reader):
    line = reader.readline(MAX_LINE)
    if not line:
        raise ValueError("通道已关闭")
    value = json.loads(line)
    if not isinstance(value, dict):
        raise ValueError("通道回复格式无效")
    return value


def _channel_reason(exc):
    if isinstance(exc, FileNotFoundError):
        return "常驻发射通道未运行"
    if isinstance(exc, ConnectionRefusedError):
        return "常驻发射通道未运行（残留套接字）"
    if isinstance(exc, socket.timeout):
        return "常驻发射通道无应答"
    return "常驻发射通道不可用（%s）" % exc


def channel_request(cmd, path=None, confirm_s=CONFIRM_SECONDS):
    """向常驻通道发一条请求并返回最终回复。

    ChannelError：通道不存在/拒绝/1s 内未受理/回复要求退回——此时未发出任何
    指令，调用方可退回直连。fire 受理后若最终回复超时，不抛 ChannelError
    （指令可能已发出），返回 ok=false 的说明。"""
    if not hasattr(socket, "AF_UNIX"):  # 非板上环境（如 Windows）不可能有常驻通道
        raise ChannelError(_channel_reason(FileNotFoundError()))
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        sock.settimeout(CONNECT_TIMEOUT_S)
        try:
            sock.connect(path or socket_path())
            sock.sendall((json.dumps({"cmd": cmd}) + "\n").encode())
            reader = sock.makefile("rb")
            sock.settimeout(ACK_TIMEOUT_S)
            ack = _read_json_line(reader)
        except (OSError, ValueError) as exc:
            raise ChannelError(_channel_reason(exc))
        if not ack.get("ack"):
            raise ChannelError("常驻发射通道拒绝请求：%s" % ack.get("reason", "未知"))
        sock.settimeout(confirm_s + REPLY_MARGIN_S)
        try:
            reply = _read_json_line(reader)
        except (OSError, ValueError) as exc:
            if cmd != "fire":
                raise ChannelError(_channel_reason(exc))
            return {"ok": False, "path": "server",
                    "reason": "常驻发射通道已受理发射但未回复（%s），结果以火控状态为准"
                              % _channel_reason(exc)}
        if reply.get("fallback"):
            raise ChannelError(reply.get("reason") or "常驻发射通道未就绪")
        return reply
    finally:
        sock.close()


# ---------------------------------------------------------------- 常驻通道

def peer_uid(conn):
    if not hasattr(socket, "SO_PEERCRED"):
        return None
    creds = conn.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED,
                            struct.calcsize("3i"))
    return struct.unpack("3i", creds)[1]


class FireChannel:
    """常驻发射通道核心（与 ROS 解耦，可无 ROS 单测）。

    link 需提供 status() -> {"matched","phase",...} 与 publish_fire()；
    ROS 回调经 on_state(text)/on_shot(shot_id) 喂入火控状态与开火事件。"""

    def __init__(self, link=None, clock=time.monotonic, log=log_line):
        self.link = link
        self.clock = clock
        self.log = log
        self.lock = threading.Lock()
        self.fire_lock = threading.Lock()
        self.stop = threading.Event()
        self.state = None
        self.state_rx = None
        self.state_count = 0
        self.pending = None
        self.confirmed = None   # {"at", "shots"}：本通道最近一次确认的开火
        self.started = clock()
        self.path = None
        self.sock_ino = None

    # ---- ROS 侧输入 ----
    def on_state(self, text):
        state = parse_state(text)
        with self.lock:
            self.state = state
            self.state_rx = self.clock()
            self.state_count += 1
            pending = self.pending
            if (pending is not None and not pending["event"].is_set()
                    and summarize(state)["shots"] > pending["shots_before"]):
                pending.update(by="strike_state", at=self.clock())
                pending["event"].set()

    def on_shot(self, shot_id):
        with self.lock:
            pending = self.pending
            if pending is not None and not pending["event"].is_set():
                pending.update(by="shot_event", shot_id=shot_id, at=self.clock())
                pending["event"].set()

    # ---- 状态 ----
    def snapshot(self):
        with self.lock:
            age = None if self.state_rx is None else self.clock() - self.state_rx
            return self.state, age

    def current(self):
        """最新火控状态摘要与其年龄（无状态时 (None, None)）。

        开火事件先于下一条 2Hz 状态到达：本通道刚确认开火的 CONFIRM_HOLD_S 内，
        shots 尚未增加的状态按已发射处理（并发/连按不会再发布一次）。"""
        with self.lock:
            state, rx, confirmed = self.state, self.state_rx, self.confirmed
            now = self.clock()
        if state is None:
            return None, None
        payload = summarize(state)
        if (confirmed is not None and payload["shots"] < confirmed["shots"]
                and now - confirmed["at"] < CONFIRM_HOLD_S):
            payload.update(shots=confirmed["shots"], latched=True, ready=False,
                           block="LATCHED")
        return payload, now - rx

    def link_status(self):
        if self.link is None:
            return {"matched": False, "phase": "等待火控节点"}
        return dict(self.link.status())

    def link_info(self, age):
        info = self.link_status()
        fresh = age is not None and age <= STATE_FRESH_S
        info.update(state_age_s=None if age is None else round(age, 3),
                    state_fresh=fresh, ready=bool(info.get("matched")) and fresh,
                    up_s=round(self.clock() - self.started, 1))
        return info

    @staticmethod
    def stale_reason(age):
        if age is None:
            return "常驻发射通道尚未收到火控状态"
        return "常驻发射通道的火控状态 %.1fs 未更新" % age

    # ---- 请求 ----
    def handle(self, cmd):
        if cmd == "fire":
            return self.handle_fire()
        if cmd == "status":
            return self.handle_status()
        return self.handle_ping()

    def handle_ping(self):
        payload, age = self.current()
        reply = {"ok": True, "link": self.link_info(age)}
        if payload is not None and age <= STATE_FRESH_S:
            payload["reason"] = reason_text(payload)
            reply["state"] = payload
        return reply

    def handle_status(self):
        payload, age = self.current()
        if payload is None or age > STATE_FRESH_S:
            return {"fallback": True, "reason": self.stale_reason(age)}
        payload.update(reason=reason_text(payload), path="server",
                       state_age_s=round(age, 3))
        return payload

    def handle_fire(self):
        started = self.clock()
        with self.fire_lock:
            payload, age = self.current()
            if payload is None or age > STATE_FRESH_S:
                return {"fallback": True, "reason": self.stale_reason(age)}
            payload.update(path="server", state_age_s=round(age, 3))
            reject = fire_gate(payload)
            if reject:
                payload.update(ok=False, reason=reject)
                self.log("手动发射请求被拦下：%s" % reject)
                return payload
            link = self.link_status()
            if not link.get("matched"):
                return {"fallback": True,
                        "reason": "常驻发射通道尚未与火控匹配（%s）" % link.get("phase", "")}
            event = threading.Event()
            pending = {"event": event, "shots_before": payload["shots"]}
            with self.lock:
                self.pending = pending
            try:
                published = self.clock()
                self.link.publish_fire()
            except Exception as exc:  # noqa: BLE001 —— 未发出即可退回直连
                with self.lock:
                    self.pending = None
                return {"fallback": True, "reason": "常驻发射通道发布失败：%s" % exc}
            self.log("手动发射指令已发布（火控状态 %.2fs 前 fready=1）" % age)
            confirmed = event.wait(CONFIRM_SECONDS)
            with self.lock:
                self.pending = None
                if confirmed:
                    self.confirmed = {"at": pending["at"],
                                      "shots": payload["shots"] + 1}
            payload.update(fired=confirmed,
                           publish_ms=round((published - started) * 1000.0, 1))
            if confirmed:
                shots = payload["shots"] + 1
                payload.update(latched=True, ready=False, shots=shots,
                               block="LATCHED", confirmed_by=pending.get("by"),
                               confirm_ms=round((pending["at"] - published) * 1000.0, 1),
                               reason="已发射（火控确认 shots=%d）" % shots)
                if pending.get("shot_id") is not None:
                    payload["shot_id"] = pending["shot_id"]
                self.log("火控确认开火：发布后 %.0f ms（%s）"
                         % (payload["confirm_ms"], payload["confirmed_by"]))
                return payload
            latest, _ = self.current()
            last = latest if latest is not None else payload
            payload.update(ok=False, reason="指令已发出但 %.0fs 内火控未确认（%s）"
                           % (CONFIRM_SECONDS, reason_text(last)))
            self.log(payload["reason"], level="WARN")
            return payload

    # ---- 套接字 ----
    def bind(self, path):
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        if os.path.lexists(path):
            probe = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            probe.settimeout(0.5)
            try:
                probe.connect(path)
            except OSError:
                os.unlink(path)  # 上次异常退出的残留套接字
            else:
                raise RuntimeError("已有常驻发射通道在运行（%s）" % path)
            finally:
                probe.close()
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        old = os.umask(0o177)
        try:
            sock.bind(path)
        finally:
            os.umask(old)
        os.chmod(path, 0o600)
        sock.listen(16)
        sock.settimeout(0.5)
        self.path = path
        self.sock_ino = os.stat(path).st_ino
        return sock

    def unlink(self):
        try:
            if self.path and os.stat(self.path).st_ino == self.sock_ino:
                os.unlink(self.path)
        except OSError:
            pass

    def serve_forever(self, sock):
        while not self.stop.is_set():
            try:
                conn, _ = sock.accept()
            except socket.timeout:
                continue
            except OSError:
                if self.stop.is_set():
                    break
                raise
            threading.Thread(target=self.serve_connection, args=(conn,),
                             daemon=True).start()

    def serve_connection(self, conn):
        with conn:
            try:
                conn.settimeout(2.0)
                uid = peer_uid(conn)
                if uid is not None and uid != os.getuid():
                    self._send(conn, {"ack": False, "reason": "非本用户进程"})
                    return
                line = conn.makefile("rb").readline(4096)
                request = json.loads(line) if line else None
                cmd = request.get("cmd") if isinstance(request, dict) else None
                if cmd not in ("status", "fire", "ping"):
                    self._send(conn, {"ack": False, "reason": "未知请求"})
                    return
                self._send(conn, {"ack": True})
                self._send(conn, self.handle(cmd))
            except (OSError, ValueError) as exc:
                self.log("请求处理失败：%s" % exc, level="WARN")

    @staticmethod
    def _send(conn, value):
        conn.sendall((json.dumps(value, ensure_ascii=False) + "\n").encode())


def load_ros():
    import rclpy  # noqa: PLC0415 —— 客户端快路径不需要 ROS
    from rclpy.node import Node  # noqa: PLC0415
    from rclpy.qos import qos_profile_sensor_data  # noqa: PLC0415
    from std_msgs.msg import Bool, String  # noqa: PLC0415
    return rclpy, Node, qos_profile_sensor_data, Bool, String


class RosLink:
    """常驻通道的 ROS 侧：火控端点出现后才建订阅/发布端，失联自愈。"""

    def __init__(self, channel):
        rclpy, Node, sensor_qos, Bool, String = load_ros()
        from rclpy.executors import SingleThreadedExecutor  # noqa: PLC0415
        try:
            from virtual_interception_interfaces.msg import (  # noqa: PLC0415
                VirtualShotEvent)
        except ImportError:
            VirtualShotEvent = None
        self.rclpy, self.Bool, self.String = rclpy, Bool, String
        self.sensor_qos, self.ShotEvent = sensor_qos, VirtualShotEvent
        self.channel = channel
        try:
            from rclpy.signals import SignalHandlerOptions  # noqa: PLC0415
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        except (ImportError, TypeError):
            rclpy.init()
        self.node = Node(SERVER_NODE)
        self.lock = threading.Lock()
        self.state_sub = None
        self.state_sub_since = 0.0
        self.last_heal = -1e9
        self.fire_pub = None
        self.fire_pub_since = 0.0
        self.shot_sub = None
        self.matched = False
        self.phase = "等待火控端点"
        self.ready_logged = None
        self.node.create_timer(0.5, self.maintain)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

    def status(self):
        with self.lock:
            return {"matched": self.matched, "phase": self.phase,
                    "shot_event": self.shot_sub is not None}

    def publish_fire(self):
        pub = self.fire_pub
        if pub is None:
            raise RuntimeError("发射话题发布端未建立")
        msg = self.Bool()
        msg.data = True
        pub.publish(msg)

    def _create_state_sub(self, now):
        self.state_sub = self.node.create_subscription(
            self.String, STRIKE_TOPIC,
            lambda msg: self.channel.on_state(msg.data), self.sensor_qos)
        self.state_sub_since = now

    def maintain(self):
        try:
            self._maintain()
        except Exception as exc:  # noqa: BLE001 —— 维护失败不拖垮通道
            log_line("通道维护失败：%s" % exc, level="WARN")

    def _maintain(self):
        now = time.monotonic()
        node = self.node
        strike_pub = any(i.node_name == STRIKE_NODE
                         for i in node.get_publishers_info_by_topic(STRIKE_TOPIC))
        fire_subs = node.get_subscriptions_info_by_topic(FIRE_TOPIC)
        shot_pub = any(i.node_name == STRIKE_NODE
                       for i in node.get_publishers_info_by_topic(SHOT_TOPIC))
        strike_sub = any(i.node_name == STRIKE_NODE for i in fire_subs)
        if self.state_sub is None:
            if strike_pub:
                self._create_state_sub(now)
        else:
            _, age = self.channel.snapshot()
            last = (self.state_sub_since if age is None
                    else max(self.state_sub_since, now - age))
            silent = now - last
            if (silent > HEAL_STALE_S and strike_pub
                    and now - self.last_heal >= 2 * HEAL_STALE_S):
                node.destroy_subscription(self.state_sub)
                self._create_state_sub(now)
                self.last_heal = now
                log_line("火控状态 %.0fs 未收到，已重建订阅" % silent, level="WARN")
        if self.fire_pub is None and strike_sub:
            self.fire_pub = node.create_publisher(self.Bool, FIRE_TOPIC, 1)
            self.fire_pub_since = now
        if self.shot_sub is None and self.ShotEvent is not None and shot_pub:
            self.shot_sub = node.create_subscription(
                self.ShotEvent, SHOT_TOPIC,
                lambda msg: self.channel.on_shot(int(msg.shot_id)), 10)
        matched = False
        if self.fire_pub is not None and strike_sub:
            count = self.fire_pub.get_subscription_count()
            matched = count >= len(fire_subs) or (
                count > 0 and now - self.fire_pub_since >= MATCH_GRACE_S)
        _, age = self.channel.snapshot()
        fresh = age is not None and age <= STATE_FRESH_S
        phase = ("就绪" if matched and fresh else
                 "等待火控端点" if not (strike_pub and strike_sub) else
                 "等待匹配" if not matched else "等待火控状态")
        with self.lock:
            self.matched = matched
            self.phase = phase
        ready = matched and fresh
        if ready != self.ready_logged and (ready or self.ready_logged):
            log_line("常驻发射通道就绪（按键直达火控）" if ready
                     else "常驻发射通道未就绪：%s（按键退回直连）" % phase,
                     level="INFO" if ready else "WARN")
        if ready or self.ready_logged is not None:
            self.ready_logged = ready

    def spin(self, stop, parent_pid=None):
        while not stop.is_set():
            self.executor.spin_once(timeout_sec=0.2)
            if parent_pid and not pid_alive(parent_pid):
                log_line("拦截链进程 %d 已退出，常驻发射通道随之停止" % parent_pid)
                stop.set()

    def close(self):
        try:
            self.executor.remove_node(self.node)
            self.node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        self.rclpy.try_shutdown()


def serve(args):
    parent_pid = None
    if "--parent-pid" in args:
        try:
            parent_pid = int(args[args.index("--parent-pid") + 1])
        except (IndexError, ValueError):
            emit({"ok": False, "reason": "--parent-pid 需要进程号"})
            return 2
    channel = FireChannel()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: channel.stop.set())
    path = socket_path()
    try:
        sock = channel.bind(path)
    except (OSError, RuntimeError) as exc:
        log_line("常驻发射通道未启动：%s" % exc, level="ERROR")
        return 1
    log_line("常驻发射通道启动：%s（等待火控节点）" % path)
    acceptor = threading.Thread(target=channel.serve_forever, args=(sock,),
                                daemon=True)
    acceptor.start()
    link = None
    try:
        domain = ros_domain()
        while not channel.stop.is_set():
            age = strike_process_age(domain)
            if age is not None and age >= STRIKE_SETTLE_S:
                break
            if parent_pid and not pid_alive(parent_pid):
                log_line("拦截链进程 %d 已退出，常驻发射通道随之停止" % parent_pid)
                return 0
            channel.stop.wait(0.5)
        if channel.stop.is_set():
            return 0
        link = RosLink(channel)
        channel.link = link
        link.spin(channel.stop, parent_pid)
        return 0
    finally:
        channel.stop.set()
        sock.close()
        channel.unlink()
        acceptor.join(timeout=2.0)
        if link is not None:
            link.close()
        log_line("常驻发射通道已停止")


# ---------------------------------------------------------------- 直连路径

def strike_matched(node, pub):
    """发布端须与火控订阅真正匹配：仅 get_subscription_count()>0 不够——
    bag 录制器等其他订阅者先匹配时会误判，单次发布丢失。
    图里要有火控订阅，且本发布端已匹配图中全部订阅者。"""
    infos = node.get_subscriptions_info_by_topic(FIRE_TOPIC)
    if not any(i.node_name == STRIKE_NODE for i in infos):
        return False
    return pub.get_subscription_count() >= len(infos)


def direct(action):
    """原直连路径：本进程建节点，等首条火控状态（最长 8s）再判定/发布。"""
    rclpy, Node, sensor_qos, Bool, String = load_ros()
    rclpy.init()
    node = Node("netgun_fire_button")
    states = []
    node.create_subscription(
        String, STRIKE_TOPIC, lambda msg: states.append(parse_state(msg.data)),
        sensor_qos)
    pub = node.create_publisher(Bool, FIRE_TOPIC, 1) if action == "fire" else None

    def spin_until(predicate, timeout_s):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if predicate():
                return True
            rclpy.spin_once(node, timeout_sec=0.05)
        return bool(predicate())

    try:
        if not spin_until(lambda: bool(states), SAMPLE_SECONDS):
            running = stack_running()
            return {"ok": False, "running": running,
                    "reason": ("拦截链在运行但收不到火控状态（/debug/strike_state）"
                               "——飞控姿态/状态未到或 DDS 未匹配，无法发射"
                               if running else "拦截链未运行")}
        payload = summarize(states[-1])
        if action == "status":
            payload["reason"] = reason_text(payload)
            return payload
        reject = fire_gate(payload)
        if reject:
            payload.update(ok=False, reason=reject)
            return payload
        if not spin_until(lambda: strike_matched(node, pub), MATCH_SECONDS):
            # 兜底：有订阅者 QoS 不兼容时全匹配永远不成立——只要图中有火控
            # 订阅且已有匹配就照发（是否打出由 shots 确认，见下）
            has_strike = any(
                i.node_name == STRIKE_NODE
                for i in node.get_subscriptions_info_by_topic(FIRE_TOPIC))
            if not (has_strike and pub.get_subscription_count() > 0):
                payload.update(ok=False, reason="火控未订阅 /netgun/fire_command，"
                               "指令未发出")
                return payload
        seen_before = len(states)
        shots_before = payload["shots"]
        msg = Bool()
        msg.data = True
        pub.publish(msg)
        confirmed = spin_until(
            lambda: any(summarize(s)["shots"] > shots_before
                        for s in states[seen_before:]),
            CONFIRM_SECONDS)
        payload["fired"] = confirmed
        if confirmed:
            payload.update(latched=True, ready=False, shots=shots_before + 1,
                           block="LATCHED",
                           reason="已发射（火控确认 shots=%d）" % (shots_before + 1))
            return payload
        last = summarize(states[-1])
        payload.update(ok=False, reason="指令已发出但 %.0fs 内火控未确认（%s）"
                       % (CONFIRM_SECONDS, reason_text(last)))
        return payload
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def reexec_with_ros(argv):
    """未 source ROS（面板 SSH 非登录 shell）时经 bash source humble 重启自身。"""
    if os.environ.get("NETGUN_FIRE_REEXEC") == "1" or not os.path.isfile(ROS_SETUP):
        return
    env = dict(os.environ, NETGUN_FIRE_REEXEC="1")
    script = 'source "%s" >/dev/null 2>&1; exec python3 "$0" "$@"' % ROS_SETUP
    os.execvpe("bash", ["bash", "-c", script, os.path.abspath(__file__)]
               + list(argv[1:]), env)


def finish(payload, started, note=None):
    payload.setdefault("path", "direct")
    if note and payload.get("path") != "server":
        payload["fallback_reason"] = note
    payload["elapsed_ms"] = round((time.time() - started) * 1000.0, 1)
    emit(payload)
    return 0 if payload.get("ok") else 1


def main(argv):
    action = argv[1] if len(argv) > 1 else "status"
    if action == "serve":
        return serve(argv[2:])
    if action not in ("status", "fire"):
        emit({"ok": False, "reason": "用法: netgun_fire_button.py status|fire|serve"})
        return 2
    try:
        started = float(os.environ.get("NETGUN_FIRE_T0", ""))
    except ValueError:
        started = time.time()
    note = os.environ.get("NETGUN_FIRE_FALLBACK") or None
    if "--direct" not in argv[2:] and os.environ.get("NETGUN_FIRE_REEXEC") != "1":
        try:
            return finish(channel_request(action), started)
        except ChannelError as exc:
            note = str(exc)
        if ros_domain() == "0" and not stack_running():
            return finish({"ok": False, "running": False, "path": "none",
                           "reason": "拦截链未运行"}, started, note)
    try:
        load_ros()
    except ImportError:
        os.environ.update(NETGUN_FIRE_T0=repr(started),
                          NETGUN_FIRE_FALLBACK=note or "")
        reexec_with_ros(argv if "--direct" in argv else list(argv) + ["--direct"])
        return finish({"ok": False, "reason": "板上缺少 ROS 2 环境（rclpy），无法直连"},
                      started, note)
    return finish(direct(action), started, note)


if __name__ == "__main__":
    sys.exit(main(sys.argv))
