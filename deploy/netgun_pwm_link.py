#!/usr/bin/env python3
"""网枪实物击发链路（2026-10-23）：拦截链开火 → MAVLink → 飞控 FMU PWM OUT 5。

链路：netgun_strike_node 开火 → trigger_command（/bin/sh -c，环境变量
NETGUN_SHOT_ID / NETGUN_SHOT_REASON）→ 本脚本 fire 客户端 → 本机 Unix 套接字
→ 常驻服务 serve（用户级 systemd：netgun-pwm-link.service，开机自启）→ UDP
127.0.0.1:14555 → MAVProxy（qgc-mavlink-forward.service，USB 连飞控）→ PX4。
不碰 ttyTHS1（uXRCE-DDS 专用）；拦截链 /fmu/in/vehicle_command 仍无发布者
（rc_gated 红线不变）。本服务不发心跳、不改参数、不发解锁类指令。

用法：
  python3 netgun_pwm_link.py serve [--fire-value -1] [--safe-value 1|none] [--hold-s 1.0]
  python3 netgun_pwm_link.py fire [--bench] [--timeout 1.5]   （装订时用 python3 -I -S，启动更快）
  python3 netgun_pwm_link.py status [--brief]

飞控（PX4 v1.15.4）配置：PWM_AUX_FUNC5=301（Peripheral via Actuator Set 1）、
DIS5=0（未解锁无脉冲）、FAIL5=0、MIN5=1000、MAX5=2000、TIM1=50（AUX5–6 50 Hz）；
其他 PWM_MAIN/AUX_FUNC1–8 不得再用 301。服务启动后只读核对（PARAM_REQUEST_READ，
每 60 s 复核，不符时每 10 s），不符或未读到时拒绝击发。

两种击发（击发值/安全值为归一化 −1..+1，对应 MIN5..MAX5）：
  live（trigger_command 默认装订）：仅飞控已解锁时发 MAV_CMD_DO_SET_ACTUATOR(187)
    p1=击发值（默认 −1 → 1000 µs），等 ACK；保持 hold_s（默认 1 s）后发安全值
    （默认 +1 → 2000 µs；--safe-value none 不复位）。未解锁时直接拒绝、什么都
    不发——PX4 会锁存设定值，未解锁时发出的值会在下次解锁瞬间输出。
  bench（--bench，台架）：仅未解锁时发 MAV_CMD_ACTUATOR_TEST(310) p5=1301，
    击发值保持 hold_s → 安全值 0.5 s → 释放（回到 DIS5 无脉冲）。不锁存；
    已解锁或 COM_MOT_TEST_EN≠1 时拒绝。
ACK 每次等 0.25 s，最多发 3 次（confirmation 0/1/2）。指令发出后无论 ACK 结果
如何都执行复位/释放。同一时刻只允许一个击发序列（保持/复位期间再请求返回 BUSY）。

fire 输出一行 "NETGUN_PWM_RESULT {json}"（经火控节点 stdout 进运行日志），退出码
0=飞控已接受，1=拒绝/飞控未接受/回复超时，2=链路不可用（服务未运行或飞控无心跳，
未发出任何指令）。status 输出 JSON（--brief 一行中文），退出码 0=就绪，1=未就绪，
2=服务不可达。

记录：每次请求（含拒绝）以 JSON 行追加到 $NETGUN_LOG_DIR/netgun_pwm_events.jsonl
（默认 ~/netgun_logs）；请求带 NETGUN_REVIEW_DIR（run_interceptor_stack.sh 传给
拦截链）时同时写入该复盘目录 pwm_events.jsonl（含复位结果与舵机输出回读）。

控制套接字：$NETGUN_PWM_SOCKET，缺省 $NETGUN_LOG_DIR/netgun_pwm.sock（仅本用户可连）。
客户端只用标准库（fire 路径约 0.1 s）；serve 需要 pymavlink。
"""

import json
import math
import os
import signal
import socket
import struct
import sys
import threading
import time

LISTEN = ("127.0.0.1", 14555)
SRC_SYSTEM = 245
SRC_COMPONENT = 191
CMD_DO_SET_ACTUATOR = 187
CMD_ACTUATOR_TEST = 310
ACTUATOR_SET1 = 301
TEST_FUNCTION = 1000 + ACTUATOR_SET1  # ACTUATOR_TEST p5≥1000：直接按输出功能号
EXPECTED = (
    ("PWM_AUX_FUNC5", 301),
    ("PWM_AUX_DIS5", 0),
    ("PWM_AUX_FAIL5", 0),
    ("PWM_AUX_MIN5", 1000),
    ("PWM_AUX_MAX5", 2000),
    ("PWM_AUX_TIM1", 50),
)
OTHER_FUNCS = tuple("PWM_MAIN_FUNC%d" % i for i in range(1, 9)) + tuple(
    "PWM_AUX_FUNC%d" % i for i in range(1, 9) if i != 5)
NOTES = ("COM_MOT_TEST_EN", "COM_PREARM_MODE")
PWM_MIN_US, PWM_MAX_US = 1000, 2000

HEARTBEAT_STALE_S = 3.0
ACK_TIMEOUT_S = 0.25
ACK_ATTEMPTS = 3
PARAM_PERIOD_S = 60.0
PARAM_RETRY_S = 10.0
PARAM_GAP_S = 0.02
BENCH_SAFE_S = 0.5
FIRE_TIMEOUT_S = 1.5
STATUS_TIMEOUT_S = 0.5
CONNECT_TIMEOUT_S = 0.3
MAX_LINE = 65536
WANTED = (0, 22, 36, 77)  # HEARTBEAT, PARAM_VALUE, SERVO_OUTPUT_RAW, COMMAND_ACK
RESULT_NAMES = {0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED",
                3: "UNSUPPORTED", 4: "FAILED", 5: "IN_PROGRESS", 6: "CANCELLED"}
REFUSALS = {
    "BUSY": "上一次击发序列未结束（保持/复位中）",
    "NO_LINK": "飞控无心跳（MAVProxy 转发 127.0.0.1:14555 未通或飞控未上电）",
    "CONFIG": "飞控 AUX5 参数不符或未读到",
    "DISARMED": "飞控未解锁：不发击发指令（PX4 会锁存设定值，解锁瞬间才输出）；台架测试用 --bench",
    "ARMED": "飞控已解锁：台架模式只在未解锁时可用",
    "MOT_TEST_DISABLED": "COM_MOT_TEST_EN≠1：飞控不接受执行器测试（台架模式不可用）",
}


def log_dir():
    return (os.environ.get("NETGUN_LOG_DIR", "").strip()
            or os.path.expanduser("~/netgun_logs"))


def socket_path():
    return os.environ.get("NETGUN_PWM_SOCKET", "").strip() or os.path.join(
        log_dir(), "netgun_pwm.sock")


def pwm_us(value):
    if value is None:
        return None
    return int(round(PWM_MIN_US + (value + 1.0) * 0.5 * (PWM_MAX_US - PWM_MIN_US)))


def log_line(text, level="INFO"):
    # 与 ROS 日志同格式；勿含 "NETGUN FIRE"/"netgun diag:" 等状态脚本计数关键字
    print("[%s] [%.9f] [netgun_pwm_link]: %s" % (level, time.time(), text), flush=True)


def emit(payload, prefix=None):
    text = json.dumps(payload, ensure_ascii=False, sort_keys=True)
    try:
        print(("%s %s" % (prefix, text)) if prefix else text, flush=True)
    except UnicodeEncodeError:  # 非 UTF-8 终端/日志：退回 ASCII 转义，结果行不能丢
        text = json.dumps(payload, sort_keys=True)
        print(("%s %s" % (prefix, text)) if prefix else text, flush=True)


# ---------------------------------------------------------------- 客户端

class LinkUnavailable(Exception):
    """服务不可达（未发出任何指令）。"""


class ReplyTimeout(Exception):
    """请求已送达但未在时限内回复（fire 时指令可能已发出）。"""


def request(payload, timeout_s=FIRE_TIMEOUT_S, path=None):
    if not hasattr(socket, "AF_UNIX"):
        raise LinkUnavailable("击发服务未运行（非板上环境）")
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        sock.settimeout(min(CONNECT_TIMEOUT_S, timeout_s))
        try:
            sock.connect(path or socket_path())
            sock.sendall((json.dumps(payload) + "\n").encode())
        except FileNotFoundError:
            raise LinkUnavailable("击发服务未运行")
        except ConnectionRefusedError:
            raise LinkUnavailable("击发服务未运行（残留套接字）")
        except OSError as exc:
            raise LinkUnavailable("击发服务不可用（%s）" % exc)
        sock.settimeout(timeout_s)
        try:
            line = sock.makefile("rb").readline(MAX_LINE)
        except OSError as exc:
            raise ReplyTimeout("击发服务未回复（%s）" % exc)
        if not line:
            raise ReplyTimeout("击发服务关闭了连接")
        try:
            reply = json.loads(line)
        except ValueError:
            raise ReplyTimeout("击发服务回复格式无效")
        if not isinstance(reply, dict):
            raise ReplyTimeout("击发服务回复格式无效")
        return reply
    finally:
        sock.close()


def status(timeout_s=STATUS_TIMEOUT_S, path=None):
    """服务状态；不可达/超时时 {"ok": False, "error": ...}（供状态脚本/预检调用）。"""
    try:
        return request({"cmd": "status"}, timeout_s, path)
    except (LinkUnavailable, ReplyTimeout) as exc:
        return {"ok": False, "error": str(exc)}


def brief(info):
    """一行中文摘要（run_interceptor_stack.sh start/status、日志）。"""
    if not info.get("ok"):
        return "不可用：%s" % (info.get("error") or "击发服务未运行")
    link = info.get("link") or {}
    config = info.get("config") or {}
    if not link.get("alive"):
        return "飞控无心跳：%s" % REFUSALS["NO_LINK"]
    if config.get("ok") is None:
        return "参数核对中（未读到 %s）" % "、".join(config.get("missing") or ["参数"])
    if not config.get("ok"):
        return "参数不符：%s" % "；".join(config.get("problems") or ["未知"])
    options = info.get("options") or {}
    safe = options.get("safe_value")
    text = "就绪（飞控 %s · %s · AUX5 %+g→%s µs 保持 %.1f s%s）" % (
        link.get("fc") or "?", "已解锁" if link.get("armed") else "未解锁",
        options.get("fire_value", -1.0), options.get("fire_us"),
        options.get("hold_s", 1.0),
        " → %+g→%s µs" % (safe, options.get("safe_us")) if safe is not None else " · 不复位")
    if config.get("warnings"):
        text += "；注意：" + "；".join(config["warnings"])
    return text


def client_fire(args):
    started = time.monotonic()
    timeout_s = FIRE_TIMEOUT_S
    if "--timeout" in args:
        try:
            timeout_s = max(0.5, float(args[args.index("--timeout") + 1]))
        except (IndexError, ValueError):
            emit({"ok": False, "reason_text": "--timeout 需要秒数"}, "NETGUN_PWM_RESULT")
            return 2
    shot = os.environ.get("NETGUN_SHOT_ID", "").strip()
    payload = {
        "cmd": "fire", "mode": "bench" if "--bench" in args else "live",
        "shot_id": int(shot) if shot.isdigit() else None,
        "reason": os.environ.get("NETGUN_SHOT_REASON", "").strip() or None,
        "source": "trigger_command" if shot else "cli",
        "review_dir": os.environ.get("NETGUN_REVIEW_DIR", "").strip() or None,
    }
    code = 1
    try:
        reply = request(payload, timeout_s)
        code = 0 if reply.get("ok") else (2 if reply.get("refused") == "NO_LINK" else 1)
    except LinkUnavailable as exc:
        reply, code = {"ok": False, "refused": "UNAVAILABLE",
                       "reason_text": str(exc) + "，未发出击发指令"}, 2
    except ReplyTimeout as exc:
        reply = {"ok": False, "refused": "NO_REPLY",
                 "reason_text": str(exc) + "，指令可能已发出，以飞控与复盘记录为准"}
    reply.setdefault("mode", payload["mode"])
    reply.setdefault("shot_id", payload["shot_id"])
    reply["client_ms"] = round((time.monotonic() - started) * 1000.0, 1)
    emit(reply, "NETGUN_PWM_RESULT")
    return code


def client_status(args):
    info = status(1.0)
    if "--brief" in args:
        print("实物击发链路：" + brief(info), flush=True)
    else:
        emit(info)
    if not info.get("ok"):
        return 2
    return 0 if info.get("ready") else 1


# ---------------------------------------------------------------- 常驻服务

def _text(value):
    if isinstance(value, (bytes, bytearray)):
        value = bytes(value).decode("ascii", "replace")
    return str(value).split("\x00", 1)[0]


def param_number(msg):
    """PX4 整型参数按字节装在 float32 里：取原始载荷前 4 字节，避免 NaN 位型失真。"""
    if int(getattr(msg, "param_type", 9)) == 9:  # MAV_PARAM_TYPE_REAL32
        return float(msg.param_value)
    raw = None
    try:
        buf = bytes(msg.get_msgbuf())
        offset = 10 if buf[:1] == b"\xfd" else 6
        if len(buf) >= offset + 4:
            raw = buf[offset:offset + 4]
    except Exception:  # noqa: BLE001 —— 回退到 float 位型转换
        raw = None
    if raw is None:
        raw = struct.pack("<f", msg.param_value)
    return struct.unpack("<i", raw)[0]


class _UdpWriter:
    def __init__(self, link):
        self.link = link

    def write(self, buf):
        self.link.sock.sendto(buf, self.link.dest)


class FcLink:
    """经 MAVProxy 本机 UDP 口与飞控通信：只解码心跳/参数/舵机输出/ACK。"""

    def __init__(self, mavlink, listen=LISTEN, clock=time.monotonic, log=log_line):
        self.clock = clock
        self.log = log
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(listen)
        self.sock.settimeout(0.5)
        self.listen = self.sock.getsockname()
        self.rx = mavlink.MAVLink(None, srcSystem=SRC_SYSTEM, srcComponent=SRC_COMPONENT)
        self.rx.robust_parsing = True
        self.tx = mavlink.MAVLink(_UdpWriter(self), srcSystem=SRC_SYSTEM,
                                  srcComponent=SRC_COMPONENT)
        self.tx_lock = threading.Lock()
        self.cond = threading.Condition()
        self.dest = None
        self.peer = None
        self.peer_t = None
        self.fc = None
        self.hb_t = None
        self.armed = None
        self.base_mode = None
        self.system_status = None
        self.datagrams = 0
        self.bad = 0
        self.params = {}
        self.acks = []
        self.ack_seq = 0
        self.servo = {}
        self.watchers = []

    # ---- 接收 ----

    def run(self, stop):
        while not stop.is_set():
            try:
                data, addr = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                if stop.is_set():
                    break
                time.sleep(0.1)
                continue
            self.on_datagram(data, addr)

    def on_datagram(self, data, addr):
        now = self.clock()
        with self.cond:
            if self.peer != addr:
                self.log("MAVProxy 转发端 %s:%d" % addr)
            self.peer, self.peer_t = addr, now
            self.datagrams += 1
        for msg in self._decode(data):
            self._handle(msg, now)

    def _decode(self, data):
        size = len(data)
        msgid = None
        if size >= 12 and data[0] == 0xFD:
            if size == 12 + data[1] + (13 if data[2] & 0x01 else 0):
                msgid = data[7] | (data[8] << 8) | (data[9] << 16)
        elif size >= 8 and data[0] == 0xFE:
            if size == 8 + data[1]:
                msgid = data[5]
        if msgid is not None:
            # MAVProxy 每条消息单独一个 UDP 报文：按消息号先筛（飞控 USB 流约 340 条/s）
            if msgid not in WANTED:
                return ()
            try:
                return (self.rx.decode(bytearray(data)),)
            except Exception:  # noqa: BLE001 —— 坏帧丢弃
                self.bad += 1
                return ()
        try:  # 多帧/残帧：流式解析兜底
            msgs = self.rx.parse_buffer(data) or []
        except Exception:  # noqa: BLE001
            self.bad += 1
            return ()
        return [m for m in msgs if m.get_msgId() in WANTED]

    def _handle(self, msg, now):
        mid = msg.get_msgId()
        src = (msg.get_srcSystem(), msg.get_srcComponent())
        if mid == 0:
            # 只认 PX4 飞控本体（QGC/其他部件的心跳不能用来判断解锁）
            if msg.autopilot != 12 or msg.type == 6 or src[1] != 1:
                return
            armed = bool(msg.base_mode & 128)
            with self.cond:
                if self.fc != src:
                    self.log("飞控 %d/%d 心跳" % src)
                elif self.armed is not None and armed != self.armed:
                    self.log("飞控%s" % ("已解锁" if armed else "已上锁"))
                self.fc, self.hb_t, self.armed = src, now, armed
                self.base_mode, self.system_status = msg.base_mode, msg.system_status
            return
        with self.cond:
            if self.fc is None or src != self.fc:
                return
            if mid == 22:
                name = _text(msg.param_id)
                self.params[name] = (param_number(msg), now)
            elif mid == 77:
                self.ack_seq += 1
                self.acks.append((self.ack_seq, now, int(msg.command), int(msg.result),
                                  int(getattr(msg, "target_system", 0) or 0),
                                  int(getattr(msg, "target_component", 0) or 0)))
                del self.acks[:-32]
            elif mid == 36:
                values = [int(getattr(msg, "servo%d_raw" % i, 0) or 0) for i in range(1, 17)]
                port = int(getattr(msg, "port", 0) or 0)
                self.servo[port] = (now, values)
                for watch in self.watchers:
                    low, high = watch.setdefault(port, ([65535] * 16, [0] * 16))
                    for i, value in enumerate(values):
                        low[i] = min(low[i], value)
                        high[i] = max(high[i], value)
            self.cond.notify_all()

    # ---- 状态 ----

    def snapshot(self):
        now = self.clock()
        with self.cond:
            hb_age = None if self.hb_t is None else now - self.hb_t
            return {
                "alive": hb_age is not None and hb_age <= HEARTBEAT_STALE_S and self.peer is not None,
                "fc": "%d/%d" % self.fc if self.fc else None,
                "heartbeat_age_s": None if hb_age is None else round(hb_age, 2),
                "armed": self.armed,
                "peer": "%s:%d" % self.peer if self.peer else None,
                "rx_age_s": None if self.peer_t is None else round(now - self.peer_t, 2),
                "datagrams": self.datagrams,
                "bad": self.bad,
                "listen": "%s:%d" % self.listen,
            }

    def param(self, name):
        with self.cond:
            return self.params.get(name)

    def servo_now(self):
        with self.cond:
            return {str(port): values for port, (_, values) in sorted(self.servo.items())}

    def watch_start(self):
        watch = {}
        with self.cond:
            self.watchers.append(watch)
        return watch

    def watch_stop(self, watch):
        with self.cond:
            if watch in self.watchers:
                self.watchers.remove(watch)
            ports = {}
            for port, (low, high) in sorted(watch.items()):
                changed = {str(i + 1): [low[i], high[i]] for i in range(16)
                           if high[i] != low[i] or (i == 4 and high[i] > 0)}
                if changed:
                    ports[str(port)] = changed
            return ports

    # ---- 发送 ----

    def send(self, name, *fields):
        with self.cond:
            peer, fc = self.peer, self.fc
        if peer is None or fc is None:
            raise OSError("飞控链路未建立")
        with self.tx_lock:
            self.dest = peer
            msg = getattr(self.tx, name + "_encode")(fc[0], fc[1], *fields)
            self.tx.send(msg)
            return self.clock()

    def command(self, command, params, attempts=ACK_ATTEMPTS, timeout_s=ACK_TIMEOUT_S):
        """COMMAND_LONG + 等 COMMAND_ACK（每次 timeout_s，最多 attempts 次）。"""
        first = None
        for attempt in range(attempts):
            with self.cond:
                since = self.ack_seq
            try:
                sent = self.send("command_long", command, attempt, *params)
            except OSError as exc:
                return {"result": None, "result_name": "SEND_ERROR", "error": str(exc),
                        "attempts": attempt}
            first = sent if first is None else first
            ack = self._wait_ack(command, since, sent + timeout_s)
            if ack is not None:
                return {"result": ack[3], "result_name": RESULT_NAMES.get(ack[3], str(ack[3])),
                        "ack_ms": round((ack[1] - first) * 1000.0, 1), "attempts": attempt + 1}
        return {"result": None, "result_name": "NO_ACK", "attempts": attempts}

    def _wait_ack(self, command, since, deadline):
        with self.cond:
            while True:
                for ack in self.acks:
                    if (ack[0] > since and ack[2] == command and ack[4] in (0, SRC_SYSTEM)
                            and ack[5] in (0, SRC_COMPONENT)):
                        return ack
                remaining = deadline - self.clock()
                if remaining <= 0:
                    return None
                self.cond.wait(remaining)

    def close(self):
        self.sock.close()


class PwmService:
    def __init__(self, link, fire_value=-1.0, safe_value=1.0, hold_s=1.0,
                 events_path=None, clock=time.monotonic, log=log_line):
        self.link = link
        self.fire_value = fire_value
        self.safe_value = safe_value
        self.hold_s = hold_s
        self.events_path = events_path or os.path.join(log_dir(), "netgun_pwm_events.jsonl")
        self.clock = clock
        self.log = log
        self.stop = threading.Event()
        self.busy = threading.Lock()
        self.last_fire = None
        self.started = time.time()
        self.param_sent = None
        self.was_alive = False
        self.last_state = None
        self.path = None
        self.sock_ino = None

    # ---- 参数核对 ----

    def config(self):
        values, problems, missing, warnings = {}, [], [], []
        stamps = []
        for name, expected in EXPECTED:
            item = self.link.param(name)
            if item is None:
                missing.append(name)
                continue
            values[name] = item[0]
            stamps.append(item[1])
            if item[0] != expected:
                problems.append("%s=%s（应为 %s）" % (name, item[0], expected))
        for name in OTHER_FUNCS:
            item = self.link.param(name)
            if item is not None and item[0] == ACTUATOR_SET1:
                values[name] = item[0]
                problems.append("%s=301（Actuator Set 1 同时驱动另一路输出）" % name)
        for name in NOTES:
            item = self.link.param(name)
            if item is not None:
                values[name] = item[0]
        if values.get("COM_MOT_TEST_EN", 1) != 1:
            warnings.append("COM_MOT_TEST_EN≠1，台架模式不可用")
        if values.get("COM_PREARM_MODE", 0) != 0:
            warnings.append("COM_PREARM_MODE≠0，未解锁时也会输出锁存的击发/安全值")
        ok = False if problems else (None if missing else True)
        age = None if not stamps else round(self.clock() - min(stamps), 1)
        return {"ok": ok, "values": values, "problems": problems, "missing": missing,
                "warnings": warnings, "age_s": age}

    def maintain(self):
        """周期只读核对参数；记录链路/参数状态变化。"""
        link = self.link.snapshot()
        config = self.config()
        now = self.clock()
        if link["alive"] and not self.was_alive:
            self.param_sent = None  # 链路刚恢复（飞控可能重启过）：立即重读
        self.was_alive = link["alive"]
        if link["alive"]:
            period = PARAM_PERIOD_S if config["ok"] is True else PARAM_RETRY_S
            if self.param_sent is None or now - self.param_sent >= period:
                self.param_sent = now
                for name in [n for n, _ in EXPECTED] + list(OTHER_FUNCS) + list(NOTES):
                    if self.stop.is_set():
                        break
                    try:
                        self.link.send("param_request_read", name.encode(), -1)
                    except OSError:
                        break
                    self.stop.wait(PARAM_GAP_S)
        state = (link["alive"], config["ok"], tuple(config["problems"]))
        if state != self.last_state:
            self.last_state = state
            self.log("链路 %s；参数 %s" % (
                "正常（飞控 %s）" % link["fc"] if link["alive"] else "无飞控心跳",
                "一致" if config["ok"] else ("不符：" + "；".join(config["problems"])
                                            if config["ok"] is False else "核对中")),
                level="INFO" if link["alive"] and config["ok"] else "WARN")

    def status(self):
        link = self.link.snapshot()
        config = self.config()
        return {
            "ok": True,
            "ready": bool(link["alive"] and config["ok"] is True),
            "link": link,
            "config": config,
            "options": {"fire_value": self.fire_value, "fire_us": pwm_us(self.fire_value),
                        "safe_value": self.safe_value, "safe_us": pwm_us(self.safe_value),
                        "hold_s": self.hold_s},
            "busy": self.busy.locked(),
            "last_fire": self.last_fire,
            "servo": self.link.servo_now(),
            "started": self.started,
        }

    # ---- 击发 ----

    def fire(self, request):
        started = self.clock()
        mode = "bench" if request.get("mode") == "bench" else "live"
        record = {
            "event": "fire", "time": time.time(), "mode": mode,
            "shot_id": request.get("shot_id"), "reason": request.get("reason"),
            "source": request.get("source"), "fire_value": self.fire_value,
            "pwm_us": pwm_us(self.fire_value), "hold_s": self.hold_s,
            "safe_value": self.safe_value, "ok": False,
        }
        review_dir = request.get("review_dir")
        if not self.busy.acquire(blocking=False):
            return self._refuse(record, "BUSY", review_dir, started)
        handed_off = False
        try:
            link = self.link.snapshot()
            record["armed"] = link["armed"]
            if not link["alive"]:
                return self._refuse(record, "NO_LINK", review_dir, started)
            config = self.config()
            if config["ok"] is not True:
                detail = "；".join(config["problems"]) or "未读到 " + "、".join(config["missing"])
                return self._refuse(record, "CONFIG", review_dir, started, detail)
            if mode == "live" and not link["armed"]:
                return self._refuse(record, "DISARMED", review_dir, started)
            if mode == "bench" and link["armed"]:
                return self._refuse(record, "ARMED", review_dir, started)
            if mode == "bench" and config["values"].get("COM_MOT_TEST_EN", 1) != 1:
                return self._refuse(record, "MOT_TEST_DISABLED", review_dir, started)
            watch = self.link.watch_start()
            record["servo_before"] = self.link.servo_now()
            if mode == "live":
                result = self.link.command(CMD_DO_SET_ACTUATOR, self._set_params(self.fire_value))
            else:
                result = self.link.command(CMD_ACTUATOR_TEST, self._test_params(
                    self.fire_value, self.hold_s + BENCH_SAFE_S))
            record.update(result)
            record["ok"] = result["result"] == 0
            if not record["ok"]:
                record["reason_text"] = "飞控未接受击发指令（%s）" % result["result_name"]
            record["elapsed_ms"] = round((self.clock() - started) * 1000.0, 1)
            reply = {k: v for k, v in record.items() if k != "servo_before"}
            self.last_fire = dict(reply, pending=True)
            if result["result_name"] != "SEND_ERROR" or result.get("attempts"):
                worker = threading.Thread(target=self._complete, args=(record, watch, review_dir),
                                          daemon=True)
                worker.start()
                handed_off = True
            else:
                self.link.watch_stop(watch)
                self._record(record, review_dir)
            self.log("击发 %s shot=%s %s ack=%s ms（%s 次）" % (
                mode, record["shot_id"], result["result_name"], result.get("ack_ms", "—"),
                result.get("attempts")), level="INFO" if record["ok"] else "ERROR")
            return reply
        finally:
            if not handed_off:
                self.busy.release()

    def _set_params(self, value):
        nan = float("nan")
        return (value, nan, nan, nan, nan, nan, 0.0)

    @staticmethod
    def _test_params(value, timeout_s):
        return (value, min(max(timeout_s, 0.0), 3.0), 0.0, 0.0, float(TEST_FUNCTION), 0.0, 0.0)

    def _complete(self, record, watch, review_dir):
        """保持 hold_s 后复位（live：安全值；bench：安全值 0.5 s 后释放）。"""
        try:
            self.stop.wait(self.hold_s)
            reset = {}
            if record["mode"] == "live":
                if self.safe_value is None:
                    reset["skipped"] = "safe_value=none：保持击发值（锁存到重启飞控或下一条指令）"
                else:
                    result = self.link.command(CMD_DO_SET_ACTUATOR, self._set_params(self.safe_value))
                    reset.update(value=self.safe_value, pwm_us=pwm_us(self.safe_value), **result)
            else:
                if self.safe_value is not None:
                    result = self.link.command(CMD_ACTUATOR_TEST, self._test_params(
                        self.safe_value, BENCH_SAFE_S + 0.5))
                    reset.update(value=self.safe_value, pwm_us=pwm_us(self.safe_value), **result)
                    self.stop.wait(BENCH_SAFE_S)
                release = self.link.command(CMD_ACTUATOR_TEST, self._test_params(self.safe_value or 0.0, 0.0))
                reset["release"] = release["result_name"]
            record["reset"] = reset
            record["servo"] = self.link.watch_stop(watch)
            ok_reset = reset.get("result_name", "ACCEPTED") == "ACCEPTED" and reset.get(
                "release", "ACCEPTED") == "ACCEPTED"
            self.log("复位 %s：%s；舵机输出回读 %s" % (
                record["mode"], json.dumps(reset, ensure_ascii=False, sort_keys=True),
                json.dumps(record["servo"], sort_keys=True) if record["servo"] else "无变化/未收到"),
                level="INFO" if ok_reset else "ERROR")
        except Exception as exc:  # noqa: BLE001 —— 复位失败也要记录并释放 busy
            record["reset_error"] = str(exc)
            self.log("复位异常：%s" % exc, level="ERROR")
        finally:
            record["done_ms"] = round((time.time() - record["time"]) * 1000.0, 1)
            self._record(record, review_dir)
            self.busy.release()

    def _refuse(self, record, code, review_dir, started, detail=None):
        record["refused"] = code
        record["reason_text"] = REFUSALS[code] + ("：" + detail if detail else "")
        record["elapsed_ms"] = round((self.clock() - started) * 1000.0, 1)
        self.log("拒绝击发 %s shot=%s：%s" % (record["mode"], record["shot_id"],
                                          record["reason_text"]), level="WARN")
        self._record(record, review_dir)
        return {k: v for k, v in record.items() if k != "servo_before"}

    def _record(self, record, review_dir):
        self.last_fire = dict(record)
        line = json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n"
        targets = [self.events_path]
        if review_dir and os.path.isabs(review_dir) and os.path.isdir(review_dir):
            try:
                if os.stat(review_dir).st_uid == os.getuid():
                    targets.append(os.path.join(review_dir, "pwm_events.jsonl"))
            except OSError:
                pass
        for path in targets:
            try:
                directory = os.path.dirname(path)
                if directory:
                    os.makedirs(directory, exist_ok=True)
                with open(path, "a", encoding="utf-8") as f:
                    f.write(line)
            except OSError as exc:
                self.log("记录写入失败 %s：%s" % (path, exc), level="WARN")

    # ---- 控制套接字 ----

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
                raise RuntimeError("已有击发服务在运行（%s）" % path)
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
            threading.Thread(target=self.serve_connection, args=(conn,), daemon=True).start()

    def serve_connection(self, conn):
        with conn:
            try:
                conn.settimeout(2.0)
                if hasattr(socket, "SO_PEERCRED"):
                    creds = conn.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED,
                                            struct.calcsize("3i"))
                    if struct.unpack("3i", creds)[1] != os.getuid():
                        self._send(conn, {"ok": False, "reason": "非本用户进程"})
                        return
                line = conn.makefile("rb").readline(4096)
                request = json.loads(line) if line else None
                cmd = request.get("cmd") if isinstance(request, dict) else None
                if cmd == "fire":
                    self._send(conn, self.fire(request))
                elif cmd == "status":
                    self._send(conn, self.status())
                elif cmd == "ping":
                    self._send(conn, {"ok": True, "pong": True})
                else:
                    self._send(conn, {"ok": False, "reason": "未知请求"})
            except (OSError, ValueError) as exc:
                self.log("请求处理失败：%s" % exc, level="WARN")

    @staticmethod
    def _send(conn, value):
        conn.sendall((json.dumps(value, ensure_ascii=False) + "\n").encode())


def _option(args, name, default, parse):
    if name not in args:
        return default
    try:
        return parse(args[args.index(name) + 1])
    except (IndexError, ValueError):
        raise ValueError("%s 参数无效" % name)


def _unit_value(text):
    value = float(text)
    if not math.isfinite(value) or not -1.0 <= value <= 1.0:
        raise ValueError(text)
    return value


def _hold(text):
    value = float(text)
    if not math.isfinite(value) or not 0.1 <= value <= 2.5:
        raise ValueError(text)
    return value


def _listen(text):
    host, _, port = text.rpartition(":")
    return (host or "127.0.0.1", int(port))


def serve(args):
    try:
        fire_value = _option(args, "--fire-value", -1.0, _unit_value)
        safe_value = _option(args, "--safe-value", 1.0,
                             lambda t: None if t.lower() == "none" else _unit_value(t))
        hold_s = _option(args, "--hold-s", 1.0, _hold)
        listen = _option(args, "--listen", LISTEN, _listen)
    except ValueError as exc:
        log_line("%s（--fire-value/--safe-value 取 −1..1 或 none，--hold-s 取 0.1–2.5）" % exc,
                 level="ERROR")
        return 2
    try:
        from pymavlink.dialects.v20 import common as mavlink  # noqa: PLC0415 —— 仅服务端需要
    except ImportError as exc:
        log_line("缺少 pymavlink（%s）：pip3 install --user pymavlink" % exc, level="ERROR")
        return 1
    try:
        link = FcLink(mavlink, listen)
    except OSError as exc:
        log_line("UDP %s:%d 绑定失败：%s" % (listen[0], listen[1], exc), level="ERROR")
        return 1
    service = PwmService(link, fire_value, safe_value, hold_s)
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: service.stop.set())
    path = socket_path()
    try:
        sock = service.bind(path)
    except (OSError, RuntimeError) as exc:
        log_line("控制套接字未建立：%s" % exc, level="ERROR")
        link.close()
        return 1
    log_line("击发服务启动：UDP %s:%d ↔ MAVProxy，控制 %s；击发值 %+g（%d µs）保持 %.1f s，安全值 %s" % (
        link.listen[0], link.listen[1], path, fire_value, pwm_us(fire_value), hold_s,
        "%+g（%d µs）" % (safe_value, pwm_us(safe_value)) if safe_value is not None else "无（不复位）"))
    threads = [threading.Thread(target=link.run, args=(service.stop,), daemon=True),
               threading.Thread(target=service.serve_forever, args=(sock,), daemon=True)]
    for thread in threads:
        thread.start()
    try:
        while not service.stop.is_set():
            service.maintain()
            service.stop.wait(1.0)
    finally:
        service.stop.set()
        sock.close()
        service.unlink()
        # 正在保持/复位的击发序列先收尾（stop 置位后保持立即结束、马上复位）
        if service.busy.acquire(timeout=5.0):
            service.busy.release()
        for thread in threads:
            thread.join(timeout=2.0)
        link.close()
        log_line("击发服务已停止")
    return 0


def main(argv):
    action = argv[1] if len(argv) > 1 else "status"
    if action == "serve":
        return serve(argv[2:])
    if action == "fire":
        return client_fire(argv[2:])
    if action == "status":
        return client_status(argv[2:])
    print("用法: netgun_pwm_link.py serve|fire [--bench]|status [--brief]", file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
