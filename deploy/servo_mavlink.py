#!/usr/bin/env python3
"""舵机 MAVLink 直控服务（通用工具，独立于拦截链，换板子拷这一个文件即可）。

链路：本服务（板上常驻）→ UDP 127.0.0.1:14560 ↔ MAVProxy → PX4 → 舵机。
前提：MAVProxy 转发服务含 --out=udp:127.0.0.1:14560（install_servo_mavlink.sh 自动加）。

用法：
  servo_mavlink.py serve                     # 常驻服务（systemd 用户服务）
  servo_mavlink.py move --slot 2 --norm 0.3  # 立即转到归一化位置（-1..1），等 ACK
  servo_mavlink.py seq --points '<json>'     # 序列执行，点=[{"norm":..,"hold_ms":..,"interp_ms":..}]
  servo_mavlink.py stop                      # 停止序列
  servo_mavlink.py state                     # 服务状态（链路/最近值/序列进度）
  servo_mavlink.py ping                      # 服务存活检查

面板使用 ACTUATOR_TEST(310)，仅未解锁且遥测新鲜时允许，单条命令最多保持 1 秒。
序列保持期间续发短时测试，完成或停止后释放；不修改飞控参数，不发送解锁命令。
槽位 1-6 对应输出功能 301-306（Peripheral via Actuator Set 1-6）；
每个槽位由飞控输出功能参数映射到物理接口；本工具仅用于独立舵机台架。

QGC 一次性配置（例：舵机接 I/O PWM 座 6 号通道 = MAIN6）：
  PWM_MAIN_FUNC6=302
  PWM_MAIN_MIN6 / PWM_MAIN_MAX6 = 舵机行程脉宽（µs，如 1000/2000）
  PWM_MAIN_DIS6 = 1500   # 可选：未解锁时输出中位（地面调舵机方便）；0=未解锁无脉冲
服务只发命令、只读核对，不改任何飞控参数。
"""
import json
import math
import os
import socket
import struct
import sys
import threading
import time

LISTEN = ("127.0.0.1", 14560)
CONTROL_PATH = os.path.expanduser("~/servo_panel.sock")
CMD_DO_SET_ACTUATOR = 187
CMD_ACTUATOR_TEST = 310
SLOT_MIN, SLOT_MAX = 1, 6          # command_long 只有 7 个参数位，param7 恒 0 → 槽 1-6
ACK_TIMEOUT_S = 0.25
ACK_ATTEMPTS = 3
HEARTBEAT_STALE_S = 3.0
STEP_S = 0.02                       # 序列插值步进 20 ms
PARAMS_LIMIT_BYTES = 262144

RESULT_NAMES = {0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED", 3: "UNSUPPORTED",
                4: "FAILED", 5: "NO_ACK"}


def log_line(text, level="INFO"):
    print("%s.%03d %s %s" % (time.strftime("%H:%M:%S"), int(time.time() * 1000) % 1000,
                             level, text), flush=True)


class LinkUnavailable(Exception):
    pass


class ServoLink:
    """学习 MAVProxy peer（收遥测记源地址），发 command_long，等 COMMAND_ACK。"""

    def __init__(self, listen=LISTEN, clock=time.monotonic):
        self.clock = clock
        self.cond = threading.Condition()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(listen)
        self.sock.settimeout(0.5)
        self.listen = self.sock.getsockname()
        self.tx_lock = threading.Lock()
        self.command_lock = threading.Lock()
        self.dest = None
        self.peer_t = None
        self.fc = None          # 飞控 (sysid, comp)，从任意消息源地址学习
        self.hb_t = None
        self.last_rx_t = None   # 最后收到流的时刻（链路存活判据）
        self.ack_seq = 0        # 收到的 ACK 计数（等待窗口用）
        self.acks = []          # (seq, t, command, result, sysid, comp)
        self.params = {}        # PX4 parameters received via read-only requests.
        self.armed = None
        self.outputs = {}
        self._tx = None         # 惰性创建（需要 pymavlink，仅 serve 模式）
        self.rx_pkts = 0        # 诊断：收到 UDP 包数
        self.rx_bad = 0         # 诊断：解码失败数
        self.rx_msgs = 0        # 诊断：解码成功数
        self.rxbuf = bytearray()  # 跨包帧缓冲（MAVProxy 一包可能含多帧，
                                  # 且板上 pymavlink 2.4.49 decode 是无状态单帧解码，
                                  # 必须自行按帧头+长度精确切片后喂入）

    def tx(self):
        if self._tx is None:
            import pymavlink.dialects.v20.common as mavlink
            self._tx = mavlink.MAVLink(_UdpWriter(self), srcSystem=255, srcComponent=191)
        return self._tx

    # ---- 接收线程 ----
    def run(self, stop):
        import pymavlink.dialects.v20.common as mavlink  # 延迟导入，CLI 不需要
        rx = mavlink.MAVLink(None, srcSystem=2, srcComponent=1)
        rx.robust_parsing = True
        while not stop.is_set():
            try:
                data, addr = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            self.dest = addr
            self.peer_t = self.clock()
            self.last_rx_t = self.clock()
            self.rx_pkts += 1
            self.rxbuf += data
            # 帧提取循环：跳过垃圾字节 → 按帧头/长度切出完整帧 → 单帧喂解码
            while self.rxbuf:
                m0 = self.rxbuf[0]
                if m0 == 0xFD:
                    if len(self.rxbuf) < 10:
                        break
                    total = 12 + self.rxbuf[1] + (13 if self.rxbuf[2] & 0x01 else 0)
                elif m0 == 0xFE:
                    if len(self.rxbuf) < 8:
                        break
                    total = 8 + self.rxbuf[1]
                else:
                    del self.rxbuf[0]   # 垃圾字节（非帧头）逐个跳过
                    continue
                if len(self.rxbuf) < total:
                    break                # 帧未收完整，等下一包
                frame = bytearray(self.rxbuf[:total])
                del self.rxbuf[:total]
                try:
                    msg = rx.decode(frame)
                except Exception:
                    self.rx_bad += 1
                    continue
                if msg is None:
                    continue
                self.rx_msgs += 1
                # 只从飞控组件学习目标，排除地面站及其他组件。
                src_sys, src_comp = msg.get_srcSystem(), msg.get_srcComponent()
                if 1 <= src_sys <= 250 and src_comp == 1:
                    self.fc = (src_sys, src_comp)
                mtype = msg.get_type()
                if (src_sys, src_comp) != self.fc:
                    continue
                if mtype == "HEARTBEAT":
                    self.hb_t = self.clock()
                    self.armed = bool(msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                elif mtype == "PARAM_VALUE":
                    name = msg.param_id
                    if isinstance(name, bytes):
                        name = name.decode("ascii", "replace")
                    value = msg.param_value
                    if msg.param_type == mavlink.MAV_PARAM_TYPE_INT32:
                        # PX4 uses the bytewise parameter protocol, not a float cast.
                        value = struct.unpack("<i", struct.pack("<f", value))[0]
                    with self.cond:
                        self.params[name.rstrip("\x00")] = value
                        self.cond.notify_all()
                elif mtype == "SERVO_OUTPUT_RAW":
                    self.outputs[str(msg.port)] = {
                        str(n): getattr(msg, "servo%d_raw" % n) for n in range(1, 9)}
                elif mtype == "COMMAND_ACK":
                    with self.cond:
                        self.ack_seq += 1
                        self.acks.append((self.ack_seq, self.clock(), msg.command,
                                          msg.result, msg.get_srcSystem(), msg.get_srcComponent()))
                        if len(self.acks) > 64:
                            del self.acks[:-32]
                        self.cond.notify_all()

    def read_parameters(self, names, timeout_s=3.0):
        """Read PX4 parameters; never write parameters or command an actuator."""
        if not self.alive():
            raise LinkUnavailable("未收到飞控遥测，暂不能读取输出配置")
        with self.cond:
            for name in names:
                self.params.pop(name, None)
        tx = self.tx()
        with self.tx_lock:
            for name in names:
                tx.send(tx.param_request_read_encode(self.fc[0], self.fc[1],
                                                     name.encode("ascii"), -1))
        deadline = self.clock() + timeout_s
        with self.cond:
            while any(name not in self.params for name in names):
                remain = deadline - self.clock()
                if remain <= 0:
                    break
                self.cond.wait(remain)
            return {name: self.params[name] for name in names if name in self.params}

    # ---- 发送 ----
    def send_command(self, slot, value, attempts=ACK_ATTEMPTS):
        with self.cond:
            dest, fc = self.dest, self.fc
        if dest is None or fc is None:
            return {"result": None, "result_name": "NO_PEER",
                    "error": "未收到 MAVProxy 流（检查 --out=udp:127.0.0.1:14560）"}
        nan = float("nan")
        params = [nan] * 7
        params[slot - 1] = value
        params[6] = 0.0
        first = None
        tx = self.tx()
        for attempt in range(attempts):
            with self.cond:
                since = self.ack_seq
            msg = tx.command_long_encode(
                fc[0], fc[1], CMD_DO_SET_ACTUATOR, attempt,
                params[0], params[1], params[2], params[3],
                params[4], params[5], params[6])
            first = first if first is not None else self.clock()
            with self.tx_lock:
                tx.send(msg)
            first = first if first is not None else self.clock()
            ack = self._wait_ack(CMD_DO_SET_ACTUATOR, since, self.clock() + ACK_TIMEOUT_S)
            if ack is not None:
                return {"result": ack[3], "result_name": RESULT_NAMES.get(ack[3], str(ack[3])),
                        "ack_ms": round((ack[1] - first) * 1000.0, 1), "attempts": attempt + 1}
        return {"result": None, "result_name": "NO_ACK", "attempts": attempts}

    def _wait_ack(self, command, since, deadline):
        with self.cond:
            while True:
                for ack in self.acks:
                    if (ack[0] > since and ack[2] == command
                            and ack[4] in (0, self.fc[0] if self.fc else 0)
                            and ack[5] in (0, self.fc[1] if self.fc else 1)):
                        return ack
                remain = deadline - self.clock()
                if remain <= 0:
                    return None
                self.cond.wait(min(remain, 0.05))

    def send_test(self, slot, value, timeout_s=1.0):
        """Bench-test a peripheral for at most one second, without arming."""
        if (not SLOT_MIN <= slot <= SLOT_MAX or not math.isfinite(value)
                or not -1 <= value <= 1 or not 0 <= timeout_s <= 1):
            return {"result": None, "error": "台架测试位置、槽位或时限无效"}
        if (self.armed is not False or not self.alive() or self.hb_t is None
                or self.clock() - self.hb_t > HEARTBEAT_STALE_S):
            return {"result": None, "error": "台架测试需要新鲜遥测且飞控处于未解锁状态"}
        tx = self.tx()
        with self.command_lock:
            first = self.clock()
            for attempt in range(ACK_ATTEMPTS):
                with self.cond:
                    since = self.ack_seq
                # PX4-specific output function: 1000 + Peripheral Set (300 + slot).
                message = tx.command_long_encode(
                    self.fc[0], self.fc[1], CMD_ACTUATOR_TEST, attempt,
                    value, timeout_s, 0, 0, 1300 + slot, 0, 0)
                with self.tx_lock:
                    tx.send(message)
                ack = self._wait_ack(CMD_ACTUATOR_TEST, since, self.clock() + ACK_TIMEOUT_S)
                if ack is not None:
                    return {"result": ack[3], "result_name": RESULT_NAMES.get(ack[3], str(ack[3])),
                            "command": CMD_ACTUATOR_TEST, "timeout_s": timeout_s,
                            "ack_ms": round((ack[1] - first) * 1000, 1)}
        return {"result": None, "result_name": "NO_ACK", "error": "台架测试未收到飞控确认"}

    def alive(self):
        with self.cond:
            return bool(self.fc and self.last_rx_t is not None
                        and self.clock() - self.last_rx_t <= HEARTBEAT_STALE_S)


class _UdpWriter:
    def __init__(self, link):
        self.link = link

    def write(self, buf):
        # send_command holds tx_lock around MAVLink.send; do not acquire it twice.
        return self.link.sock.sendto(buf, self.link.dest)


# ---- 服务 ----

class ServoService:
    def __init__(self):
        self.link = ServoLink()
        self.stop = threading.Event()
        self.busy = threading.Lock()      # 一次一个序列
        self.seq_stop = threading.Event()
        self.seq_thread = None
        self.last = {}                    # slot -> 最近下发的归一化值
        self.last_t = 0.0

    # ---- 服务端命令 ----
    def handle(self, request):
        action = request.get("action")
        try:
            if action == "move":
                return self.move(int(request["slot"]), float(request["norm"]),
                                 bool(request.get("ack", True)))
            if action == "seq":
                return self.seq_start(request.get("points") or [])
            if action == "stop":
                self.seq_stop.set()
                if self.seq_thread and self.seq_thread.is_alive():
                    self.seq_thread.join(1)
                for slot in list(self.last):
                    self.link.send_test(slot, 0, timeout_s=0)
                return {"ok": True, "stopped": True}
            if action == "state":
                return self.state()
            if action == "inspect":
                names = ["PWM_%s_%s5" % (bank, field)
                         for bank in ("MAIN", "AUX")
                         for field in ("FUNC", "MIN", "MAX", "DIS", "FAIL")]
                return {**self.state(), "params": self.link.read_parameters(names)}
            if action == "ping":
                return {"ok": True, "pong": True}
            return {"ok": False, "error": "未知 action：%r" % action}
        except (KeyError, TypeError, ValueError, LinkUnavailable) as exc:
            return {"ok": False, "error": "参数无效：%s" % exc}

    def move(self, slot, norm, wait_ack):
        if not SLOT_MIN <= slot <= SLOT_MAX:
            return {"ok": False, "error": "槽位需在 %d-%d" % (SLOT_MIN, SLOT_MAX)}
        if not math.isfinite(norm) or not -1.0 <= norm <= 1.0:
            return {"ok": False, "error": "位置需为 -1..1 范围内的有限数值"}
        if self.seq_thread and self.seq_thread.is_alive():
            return {"ok": False, "error": "序列正在运行，请先停止"}
        result = self.link.send_test(slot, norm)
        ok = result.get("result") == 0
        if ok:
            self.last[slot] = norm
            self.last_t = time.time()
        return {"ok": ok, "slot": slot, "norm": round(norm, 4), **result}

    def seq_start(self, points):
        if not isinstance(points, list) or not points:
            return {"ok": False, "error": "points 需为非空数组"}
        if len(points) > 200:
            return {"ok": False, "error": "点数过多（≤200）"}
        parsed = []
        for i, p in enumerate(points):
            if not isinstance(p, dict):
                return {"ok": False, "error": "第 %d 点需为对象" % (i + 1)}
            try:
                slot = int(p.get("slot", 2))
                norm = float(p["norm"])
                hold = max(0, int(p.get("hold_ms", 0)))
                interp = max(0, int(p.get("interp_ms", 0)))
            except (KeyError, TypeError, ValueError) as exc:
                return {"ok": False, "error": "第 %d 点字段无效：%s" % (i + 1, exc)}
            if not SLOT_MIN <= slot <= SLOT_MAX:
                return {"ok": False, "error": "第 %d 点槽位越界" % (i + 1)}
            if not math.isfinite(norm) or not -1.0 <= norm <= 1.0:
                return {"ok": False, "error": "第 %d 点位置需为 -1..1 范围内的有限数值" % (i + 1)}
            parsed.append({"slot": slot, "norm": norm,
                           "hold_ms": hold, "interp_ms": interp})
        if self.seq_thread and self.seq_thread.is_alive():
            return {"ok": False, "error": "已有序列在执行（先 stop）"}
        self.seq_stop.clear()
        self.seq_thread = threading.Thread(
            target=self._seq_run, args=(parsed,), daemon=True)
        self.seq_thread.start()
        return {"ok": True, "seq_started": True, "points": len(parsed)}

    def _seq_run(self, points):
        started = self.clock_ms()
        for idx, p in enumerate(points):
            if self.seq_stop.is_set():
                break
            slot, target = p["slot"], p["norm"]
            current = self.last.get(slot, 0.0)
            steps = max(1, int(p["interp_ms"] / (STEP_S * 1000)))
            for s in range(1, steps + 1):
                if self.seq_stop.is_set():
                    break
                v = current + (target - current) * s / steps
                result = self.link.send_test(slot, v)
                if result.get("result") == 0:
                    self.last[slot] = v
                else:
                    self.seq_stop.set()
                    break
                self.stop.wait(STEP_S)
            if self.seq_stop.is_set():
                break
            remain = p["hold_ms"] / 1000.0
            while remain > 0 and not self.seq_stop.is_set():
                duration = min(remain, 0.4)
                self.seq_stop.wait(duration)
                remain -= duration
                if remain > 0 and not self.seq_stop.is_set():
                    if self.link.send_test(slot, target).get("result") != 0:
                        self.seq_stop.set()
        for slot in {p["slot"] for p in points}:
            self.link.send_test(slot, 0, timeout_s=0)
        log_line("序列结束（%d 点，用时 %.1f s%s）" % (
            len(points), (self.clock_ms() - started) / 1000.0,
            "，中途停止" if self.seq_stop.is_set() else ""))

    def state(self):
        lnk = self.link
        with lnk.cond:
            acks = len(lnk.acks)
        return {"ok": True, "alive": lnk.alive(), "fc": lnk.fc,
                "armed": lnk.armed, "outputs": lnk.outputs,
                "last": self.last, "last_age_s": round(time.time() - self.last_t, 1) if self.last_t else None,
                "seq_running": bool(self.seq_thread and self.seq_thread.is_alive()),
                "rx_pkts": lnk.rx_pkts, "rx_bad": lnk.rx_bad, "rx_msgs": lnk.rx_msgs,
                "acks": acks, "dest": str(lnk.dest), "hb_age": round(lnk.clock() - lnk.hb_t, 1) if lnk.hb_t else None}

    @staticmethod
    def clock_ms():
        return time.monotonic() * 1000.0

    # ---- 控制套接字 ----
    def bind(self, path):
        if os.path.lexists(path):
            try:
                probe = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                probe.settimeout(0.5)
                probe.connect(path)
                probe.close()
                raise RuntimeError("已有舵机服务在运行（%s）" % path)
            except (ConnectionRefusedError, socket.timeout):
                os.unlink(path)  # 残留套接字
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        old = os.umask(0o177)
        try:
            sock.bind(path)
        finally:
            os.umask(old)
        os.chmod(path, 0o600)
        sock.listen(8)
        sock.settimeout(0.5)
        self.path = path
        return sock

    def unlink_sock(self):
        try:
            if getattr(self, "path", None) and os.path.exists(self.path):
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
                break
            threading.Thread(target=self._serve_conn, args=(conn,), daemon=True).start()

    def _serve_conn(self, conn):
        with conn:
            try:
                conn.settimeout(2.0)
                line = conn.makefile("rb").readline(PARAMS_LIMIT_BYTES)
                request = json.loads(line) if line else None
                reply = self.handle(request) if isinstance(request, dict) else \
                    {"ok": False, "error": "请求需为 JSON 对象"}
                conn.sendall((json.dumps(reply, ensure_ascii=False) + "\n").encode())
            except (OSError, ValueError) as exc:
                log_line("请求处理失败：%s" % exc, level="WARN")


# ---- CLI ----

def _control_request(request, timeout_s=8.0):
    if not hasattr(socket, "AF_UNIX"):
        raise LinkUnavailable("本机不是板子（无 Unix 套接字）")
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        sock.settimeout(3.0)
        sock.connect(CONTROL_PATH)
        sock.sendall((json.dumps(request) + "\n").encode())
        sock.settimeout(timeout_s)
        data = b""
        while True:
            chunk = sock.recv(65536)
            if not chunk:
                break
            data += chunk
            if b"\n" in data:
                break
        return json.loads(data.decode())
    except (OSError, ValueError) as exc:
        raise LinkUnavailable("舵机服务未运行或不可达（%s）。先启动：systemctl --user start servo-mavlink" % exc)
    finally:
        sock.close()


def _emit(doc):
    print("SERVO_RESULT " + json.dumps(doc, ensure_ascii=False))


def main(argv):
    if not argv or argv[0] == "serve":
        service = ServoService()
        try:
            sock = service.bind(CONTROL_PATH)
        except RuntimeError as exc:
            log_line(str(exc), level="ERROR")
            return 1
        import pymavlink  # noqa: F401 —— 确认板上已装（MAVProxy 依赖自带）
        stop = threading.Event()
        rx = threading.Thread(target=service.link.run, args=(stop,), daemon=True)
        rx.start()
        log_line("舵机服务启动：UDP %s:%d ↔ MAVProxy，控制 %s" % (
            service.link.listen[0], service.link.listen[1], CONTROL_PATH))
        try:
            service.serve_forever(sock)
        except KeyboardInterrupt:
            pass
        stop.set()
        service.unlink_sock()
        return 0

    cmd = argv[0]

    def _opt(name, cast, default=None):
        if name not in argv:
            return default
        try:
            return cast(argv[argv.index(name) + 1])
        except (IndexError, ValueError):
            _emit({"ok": False, "error": "%s 参数无效" % name})
            sys.exit(2)

    try:
        if cmd == "move":
            slot = _opt("--slot", int, 2)
            norm = _opt("--norm", float, None)
            if norm is None:
                us = _opt("--us", float, None)
                mid = _opt("--mid", float, 1500.0)
                half = _opt("--half", float, 500.0)
                if us is None:
                    _emit({"ok": False, "error": "move 需要 --norm 或 --us"})
                    return 2
                norm = (us - mid) / half
            _emit(_control_request({"action": "move", "slot": slot, "norm": norm,
                                    "ack": _opt("--ack", int, 1) == 1}))
            return 0
        if cmd == "seq":
            raw = _opt("--points", str, "[]")
            try:
                points = json.loads(raw)
            except ValueError as exc:
                _emit({"ok": False, "error": "points JSON 无效：%s" % exc})
                return 2
            _emit(_control_request({"action": "seq", "points": points}, timeout_s=15.0))
            return 0
        if cmd in ("stop", "state", "ping", "inspect"):
            _emit(_control_request({"action": cmd}, timeout_s=5.0))
            return 0
    except LinkUnavailable as exc:
        _emit({"ok": False, "error": str(exc)})
        return 3
    _emit({"ok": False, "error": "用法：serve | move | seq | stop | state | ping"})
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
