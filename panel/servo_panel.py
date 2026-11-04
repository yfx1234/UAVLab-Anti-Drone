#!/usr/bin/env python3
"""独立舵机台架面板（默认端口 8769，可设 SERVO_PANEL_PORT）。

本机 ──SSH──> 板上 deploy/servo_mavlink.py ──UDP/MAVProxy──> 飞控 PWM ──> 舵机。
板上零配置依赖：只要 servo_mavlink.py 在板 + MAVProxy 含
--out=udp:127.0.0.1:14560 输出口（install_servo_mavlink.sh 自动处理）。

用法：python servo_panel.py  →  浏览器 http://127.0.0.1:8769/
"""
import json
import math
import os
import shlex
import threading
import time
import http.server
import urllib.parse
from pathlib import Path

try:
    import paramiko
except ImportError:
    paramiko = None

HOST, PORT = "127.0.0.1", int(os.environ.get("SERVO_PANEL_PORT", "8769"))
BOARD_CLI = "python3 $HOME/interceptor_ws/deploy/servo_mavlink.py"

_LOCK = threading.Lock()
_SSH = None          # paramiko.SSHClient
_CFG = {"host": "192.168.50.100", "user": "uavlab", "password": ""}


def _ssh_exec(cmd, timeout_s=15.0):
    """在板子上执行命令，返回 (exit_code, stdout)。抛错=SSH 未连接。"""
    global _SSH
    with _LOCK:
        if _SSH is None:
            raise RuntimeError("未连接板子")
        try:
            _, out, err = _SSH.exec_command(cmd, timeout=timeout_s)
            code = out.channel.recv_exit_status()
            text = out.read().decode("utf-8", "replace")
            if code != 0:
                text += err.read().decode("utf-8", "replace")
            return code, text
        except Exception as exc:
            # 连接可能已断：丢弃会话，要求重连
            try:
                _SSH.close()
            except Exception:
                pass
            _SSH = None
            raise RuntimeError("SSH 连接断开：%s（请重新连接）" % exc)


def _ssh_connect(host, user, password):
    global _SSH, _CFG
    if paramiko is None:
        raise RuntimeError("本机未安装 paramiko：python -m pip install paramiko")
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(host, username=user, password=password,
                   timeout=10, banner_timeout=10, auth_timeout=10,
                   allow_agent=True, look_for_keys=True)
    with _LOCK:
        if _SSH is not None:
            try:
                _SSH.close()
            except Exception:
                pass
        _SSH = client
        _CFG = {"host": host, "user": user, "password": password}


def _ssh_connect_capped(host, user, password, cap_s=15.0):
    """连接硬超时：图传链路闪断时 paramiko 可能挂死，超过 cap_s 强制报错返回。"""
    result = {"done": False, "error": None}

    def _work():
        try:
            _ssh_connect(host, user, password)
        except Exception as exc:
            result["error"] = str(exc) or type(exc).__name__
        result["done"] = True

    worker = threading.Thread(target=_work, daemon=True)
    worker.start()
    worker.join(cap_s)
    if not result["done"]:
        raise RuntimeError("连接超时（%ds）：图传链路闪断，稍等几秒重试" % int(cap_s))
    if result["error"]:
        raise RuntimeError(result["error"])


def _servo(payload):
    """通过 SSH 调板上 servo CLI，解析 SERVO_RESULT JSON。"""
    op = payload["op"]
    if op == "check":
        op = "state"
    if op == "move":
        mode = str(payload.get("mode", "pulse"))
        hold_s = float(payload.get("hold_s", 1))
        norm = float(payload["norm"])
        if mode not in ("pulse", "hold") or not math.isfinite(hold_s) or not 0.2 <= hold_s <= 30:
            raise ValueError("动作模式或保持时长无效（0.2–30 秒）")
        if not math.isfinite(norm) or not -1 <= norm <= 1:
            raise ValueError("目标位置超出范围")
        cmd = "%s move --slot %d --norm %.4f --ack 1 --mode %s --hold-s %.3f" % (
            BOARD_CLI, int(payload["slot"]), norm, mode, hold_s)
        timeout = 12.0
    elif op == "seq":
        pts = json.dumps(payload["points"], ensure_ascii=False)
        after = str(payload.get("after", "center"))
        if after not in ("center", "hold"):
            raise ValueError("序列结束方式无效")
        cmd = "%s seq --points %s --after %s" % (BOARD_CLI, shlex.quote(pts), after)
        timeout = 20.0
    elif op == "stop":
        cmd = "%s stop" % BOARD_CLI
        timeout = 8.0
    elif op == "state":
        cmd = "%s state" % BOARD_CLI
        if payload.get("lease_id"):
            cmd += " --lease-id %s" % shlex.quote(str(payload["lease_id"]))
        timeout = 8.0
    elif op == "inspect":
        bank, channel = str(payload.get("bank", "MAIN")), int(payload.get("channel", 5))
        if bank not in ("MAIN", "AUX") or not 1 <= channel <= 8:
            raise ValueError("输出接口或通道无效")
        cmd = "%s inspect --bank %s --channel %d" % (BOARD_CLI, bank, channel)
        timeout = 8.0
    else:
        return {"ok": False, "error": "未知 op"}
    code, text = _ssh_exec(cmd, timeout)
    for line in text.splitlines():
        if line.startswith("SERVO_RESULT "):
            doc = json.loads(line[len("SERVO_RESULT "):])
            doc.setdefault("ok", doc.get("result") == 0 or doc.get("ok", False))
            return doc
    return {"ok": False, "error": "板上无响应（exit %d）：%s" % (code, text[-200:])}


_PANEL_DIR = Path(__file__).resolve().parent


class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def _json(self, doc, code=200):
        body = json.dumps(doc, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _page(self):
        body = (_PANEL_DIR / "servo_panel.html").read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urllib.parse.urlparse(self.path).path
        if path in ("/", "/index.html"):
            self._page()
        elif path in ("/servo_panel.css", "/servo_panel.js"):
            body = (_PANEL_DIR / path[1:]).read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/css; charset=utf-8" if path.endswith(".css") else "text/javascript; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_error(404)

    def do_POST(self):
        global _SSH
        if urllib.parse.urlparse(self.path).path != "/api":
            self.send_error(404)
            return
        origin = self.headers.get("Origin")
        if origin and origin not in ("http://127.0.0.1:%d" % PORT, "http://localhost:%d" % PORT):
            self._json({"ok": False, "error": "请从本机面板操作"}, 403)
            return
        try:
            length = int(self.headers.get("Content-Length") or 0)
            if not 0 <= length <= 131072:
                raise ValueError("请求过大")
            payload = json.loads(self.rfile.read(length) or b"{}")
            if not isinstance(payload, dict):
                raise ValueError("请求应为对象")
        except ValueError:
            self._json({"ok": False, "error": "请求 JSON 无效"})
            return
        op = payload.get("op")
        try:
            if op == "connection":
                transport = _SSH.get_transport() if _SSH is not None else None
                self._json({"ok": True, "connected": bool(transport and transport.is_active()),
                            "host": _CFG["host"], "user": _CFG["user"]})
            elif op == "connect":
                _ssh_connect_capped(str(payload["host"]).strip(),
                                    str(payload["user"]).strip(),
                                    str(payload.get("pass", "")))
                self._json({"ok": True})
            elif op == "disconnect":
                if _SSH is not None:
                    _servo({"op": "stop"})
                    with _LOCK:
                        _SSH.close()
                        _SSH = None
                self._json({"ok": True})
            elif _SSH is None:
                self._json({"ok": False, "error": "未连接板子（先点连接板子）"})
            else:
                self._json(_servo(payload))
        except Exception as exc:
            self._json({"ok": False, "error": str(exc)})


def main():
    try:
        server = http.server.ThreadingHTTPServer((HOST, PORT), Handler)
    except OSError as exc:
        print("启动失败：端口 %d 被占用（%s）。" % (PORT, exc))
        print("可能已有一个面板实例在运行：先关掉旧的命令行窗口，或重启电脑后重试。")
        input("按回车退出...")
        return 1
    print("舵机面板 http://%s:%d/  （Ctrl+C 停止）" % (HOST, PORT), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
