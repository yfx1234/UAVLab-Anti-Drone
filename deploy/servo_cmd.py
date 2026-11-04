#!/usr/bin/env python3
"""舵机控制 CLI（经击发服务 netgun_pwm_link 的 servo 通道，纯标准库）。

用法（参数 norm = 归一化位置 -1..1，对应飞控 MIN..MAX 脉宽）：
  servo_cmd.py move --slot 2 --norm 0.3
  servo_cmd.py seq --points '[{"slot":2,"norm":0.5,"hold_ms":500,"interp_ms":800},...]'
  servo_cmd.py stop
  servo_cmd.py state
  servo_cmd.py ping
"""
import json
import os
import socket
import sys

SOCK_PATH = os.environ.get("NETGUN_PWM_SOCKET") or os.path.expanduser(
    "~/netgun_logs/netgun_pwm.sock")


def _request(request, timeout_s=10.0):
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        sock.settimeout(3.0)
        sock.connect(SOCK_PATH)
        sock.sendall((json.dumps(request) + "\n").encode())
        sock.settimeout(timeout_s)
        data = b""
        while b"\n" not in data:
            chunk = sock.recv(65536)
            if not chunk:
                break
            data += chunk
        return json.loads(data.decode())
    except (OSError, ValueError) as exc:
        return {"ok": False, "error": "击发服务未运行或不可达（%s）。先启动："
                "systemctl --user start netgun-pwm-link" % exc}
    finally:
        sock.close()


def _emit(doc):
    print("SERVO_RESULT " + json.dumps(doc, ensure_ascii=False))


def main(argv):
    if not argv:
        _emit({"ok": False, "error": "用法：move | seq | stop | state | ping"})
        return 2
    cmd = argv[0]

    def _opt(name, cast, default=None):
        if name not in argv:
            return default
        try:
            return cast(argv[argv.index(name) + 1])
        except (IndexError, ValueError):
            _emit({"ok": False, "error": "%s 参数无效" % name})
            sys.exit(2)

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
        _emit(_request({"cmd": "servo", "action": "move", "slot": slot, "norm": norm}))
        return 0
    if cmd == "seq":
        raw = _opt("--points", str, "[]")
        try:
            points = json.loads(raw)
        except ValueError as exc:
            _emit({"ok": False, "error": "points JSON 无效：%s" % exc})
            return 2
        _emit(_request({"cmd": "servo", "action": "seq", "points": points}, timeout_s=15.0))
        return 0
    if cmd in ("stop", "state", "ping"):
        _emit(_request({"cmd": "servo", "action": cmd}, timeout_s=5.0))
        return 0
    _emit({"ok": False, "error": "用法：move | seq | stop | state | ping"})
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
