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

DO_SET_ACTUATOR(187)：param[slot]=归一化值，param7 恒 0，其余 NaN（PX4 忽略=保持原值）。
槽位 1-6 对应输出功能 301-306（Peripheral via Actuator Set 1-6）；
槽 1 一般留给网枪（FUNC301），舵机用槽 2 起（FUNC302 起，即 param2 起）。

QGC 一次性配置（例：舵机接 I/O PWM 座 6 号通道 = MAIN6）：
  PWM_MAIN_FUNC6=302
  PWM_MAIN_MIN6 / PWM_MAIN_MAX6 = 舵机行程脉宽（µs，如 1000/2000）
  PWM_MAIN_DIS6 = 1500   # 可选：未解锁时输出中位（地面调舵机方便）；0=未解锁无脉冲
服务只发命令、只读核对，不改任何飞控参数。
"""
import json
import os
import socket
import struct
import sys
import threading
import time

LISTEN = ("127.0.0.1", 14560)
CONTROL_PATH = os.path.expanduser("~/servo_panel.sock")
CMD_DO_SET_ACTUATOR = 187
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
        self.dest = None
        self.peer_t = None
        self.fc = None          # 飞控 (sysid, comp)，从心跳学习
        self.hb_t = None
        self.ack_seq = 0        # 收到的 ACK 计数（等待窗口用）
        self.acks = []          # (seq, t, command, result, sysid, comp)
        self._tx = None         # 惰性创建（需要 pymavlink，仅 serve 模式）

    def tx(self):
        if self._tx is None:
            import pymavlink.dialects.v20.common as mavlink
            self._tx = mavlink.MAVLink(_UdpWriter(self), srcSystem=2, srcComponent=1)
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
            try:
                msg = rx.decode(data)
            except Exception:
                continue
            mtype = msg.get_type()
            if mtype == "HEARTBEAT" and msg.type != mavlink.MAV_TYPE_GCS:
                self.fc = (msg.get_srcSystem(), msg.get_srcComponent())
                self.hb_t = self.clock()
            elif mtype == "COMMAND_ACK":
                with self.cond:
                    self.ack_seq += 1
                    self.acks.append((self.ack_seq, self.clock(), msg.command,
                                      msg.result, msg.get_srcSystem(), msg.get_srcComponent()))
                    if len(self.acks) > 64:
                        del self.acks[:-32]

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

    def alive(self):
        with self.cond:
            return bool(self.fc and self.hb_t is not None
                        and self.clock() - self.hb_t <= HEARTBEAT_STALE_S)


class _UdpWriter:
    class _MavFile:
        def __init__(self, link):
            self.link = link

        def write(self, buf):
            with self.link.tx_lock:
                self.link.sock.sendto(buf, self.link.dest)

    def __init__(self, link):
        self.mav = type("M", (), {})()
        self.mav.file = self._MavFile(link)


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
                return {"ok": True, "stopped": True}
            if action == "state":
                return self.state()
            if action == "ping":
                return {"ok": True, "pong": True}
            return {"ok": False, "error": "未知 action：%r" % action}
        except (KeyError, TypeError, ValueError) as exc:
            return {"ok": False, "error": "参数无效：%s" % exc}

    def move(self, slot, norm, wait_ack):
        if not SLOT_MIN <= slot <= SLOT_MAX:
            return {"ok": False, "error": "槽位需在 %d-%d（槽 1 一般留给网枪）" % (SLOT_MIN, SLOT_MAX)}
        norm = max(-1.0, min(1.0, norm))
        result = self.link.send_command(slot, norm)
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
            parsed.append({"slot": slot, "norm": max(-1.0, min(1.0, norm)),
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
                result = self.link.send_command(slot, v)
                if result.get("result") == 0:
                    self.last[slot] = v
                self.stop.wait(STEP_S)
            if self.seq_stop.is_set():
                break
            remain = p["hold_ms"] / 1000.0
            while remain > 0 and not self.seq_stop.is_set():
                self.stop.wait(min(remain, 0.1))
                remain -= 0.1
        log_line("序列结束（%d 点，用时 %.1f s%s）" % (
            len(points), (self.clock_ms() - started) / 1000.0,
            "，中途停止" if self.seq_stop.is_set() else ""))

    def state(self):
        return {"ok": True, "alive": self.link.alive(), "fc": self.link.fc,
                "last": self.last, "last_age_s": round(time.time() - self.last_t, 1) if self.last_t else None,
                "seq_running": bool(self.seq_thread and self.seq_thread.is_alive())}

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
        if cmd in ("stop", "state", "ping"):
            _emit(_control_request({"action": cmd}, timeout_s=5.0))
            return 0
    except LinkUnavailable as exc:
        _emit({"ok": False, "error": str(exc)})
        return 3
    _emit({"ok": False, "error": "用法：serve | move | seq | stop | state | ping"})
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
