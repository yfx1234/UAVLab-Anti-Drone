#!/usr/bin/env python3
"""网枪实物击发链路：拦截链开火 → MAVLink → 飞控 FMU PWM OUT 5（AUX5）。
2026-10-23 建链；2026-10-24 按网枪触发模块说明改为"低位待命 → 跳高位击发"，击发参数可调。

链路：netgun_strike_node 开火 → trigger_command（/bin/sh -c，环境变量
NETGUN_SHOT_ID / NETGUN_SHOT_REASON）→ 本脚本 fire 客户端 → 本机 Unix 套接字
→ 常驻服务 serve（用户级 systemd：netgun-pwm-link.service，开机自启）→ UDP
127.0.0.1:14555 → MAVProxy（qgc-mavlink-forward.service，USB 连飞控）→ PX4。
不碰 ttyTHS1（uXRCE-DDS 专用）；拦截链 /fmu/in/vehicle_command 仍无发布者
（rc_gated 红线不变）。本服务不发心跳、不改参数、不发解锁类指令。

网枪触发模块（说明书）：PWM 50 Hz、900–2100 µs。上电 LED 5 Hz 闪 3 s 初始化、安全延时 5 s，
之后 1 Hz 闪。待命态先要 ≤1300 µs（低位）持续 100 ms 才进入就绪——上电时就是高位不会触发；
就绪后 ≥1700 µs（高位）持续 100 ms → 输出打开（击发），一直保持到 <1700 µs 持续 100 ms →
关闭、回待命。所以击发 = 先低位 ≥0.1 s 再跳高位 ≥0.1 s，常态必须是低位。

AUX5 脉冲时序：
  未解锁：无脉冲（PWM_AUX_DIS5=0）。
  已解锁常态：待命低位 low_us（默认 1000 µs）。服务启动、飞控重连、改参数、每次击发后都发
    MAV_CMD_DO_SET_ACTUATOR(187) p1=低位，PX4 锁存（未解锁时也可发，解锁后才输出），飞控 ACK
    后才算就绪。PX4 v1.15.4 的 Actuator Set 只认有限值、锁存到下一条指令或飞控重启（NaN 被
    忽略，无法"释放"），所以常态锁存的只能是低位。
  live（trigger_command 默认装订，仅已解锁）：确认低位已锁存（否则先补发）且已输出满 low_s
    （默认 0.3 s；解锁/进执行器测试时 PX4 输出有 0.5 s 斜坡，从解锁起另加 0.5 s）→ 发高位
    high_us（默认 2000 µs），ACK 后回复火控 → 保持 hold_s（默认 1.0 s）→ 回低位（锁存）。
    未解锁时拒绝、不发高位：PX4 会锁存，解锁瞬间就输出高位（等于解锁即击发）。
  bench（--bench，台架，仅未解锁）：MAV_CMD_ACTUATOR_TEST(310) p5=1301：低位 0.5 s+low_s →
    高位 hold_s → 低位 low_s → 释放（回到 DIS5 无脉冲）。不锁存；COM_MOT_TEST_EN≠1 时拒绝。
ACK 每次等 0.25 s，最多发 3 次（confirmation 0/1/2）。高位发出后无论 ACK 结果如何都回低位；
回低位未被确认时每秒补发。同一时刻只允许一个击发序列（序列中再请求返回 BUSY）。

击发参数：~/netgun_config/netgun_pwm.json（NETGUN_CONFIG_DIR 可改目录；文件不存在 = 默认值；
文件无效时拒绝击发、不静默退回默认）。范围：low_us 1000–1250、high_us 1750–2000（模块门限
1300/1700 各留 50 µs，且在 MIN5/MAX5 内）、hold_s 0.3–2.5 s、low_s 0.2–2.0 s（模块确认
0.1 s）。只能在未解锁时改，立即生效：
  python3 netgun_pwm_link.py params                                 查看
  python3 netgun_pwm_link.py params --high-us 1900 --hold-s 0.8     修改（未给的项不变）
  python3 netgun_pwm_link.py params --reset                         恢复默认（删文件）
  面板「拦截任务 → 击发参数」（params save，stdin：{"config": {...}}）。
  服务未运行时直接写文件，下次启动生效。

用法：
  python3 netgun_pwm_link.py serve [--listen 127.0.0.1:14555]
  python3 netgun_pwm_link.py fire [--bench] [--timeout 5]   （装订时用 python3 -I -S，启动更快）
  python3 netgun_pwm_link.py status [--brief]
  python3 netgun_pwm_link.py params [--low-us N] [--high-us N] [--hold-s S] [--low-s S] | --reset | save

飞控（PX4 v1.15.4）配置：PWM_AUX_FUNC5=301（Peripheral via Actuator Set 1）、
DIS5=0（未解锁无脉冲；800–1250 也可）、FAIL5=0、MIN5=1000、MAX5=2000、TIM1=50（AUX5–6
50 Hz）、PWM_AUX_REV 不含 AUX5；其他 PWM_MAIN/AUX_FUNC1–8 不得再用 301。服务启动后只读核对
（PARAM_REQUEST_READ，每 60 s 复核，不符时每 10 s），不符或未读到时拒绝击发。

fire 输出一行 "NETGUN_PWM_RESULT {json}"（经火控节点 stdout 进运行日志），退出码
0=飞控已接受高位，1=拒绝/飞控未接受/回复超时，2=链路不可用（服务未运行或飞控无心跳，
未发出任何指令）。status 输出 JSON（--brief 一行中文），退出码 0=就绪，1=未就绪，
2=服务不可达。params 输出 JSON，退出码 0=成功，1=失败/被拒，2=用法错误。

记录：每次请求（含拒绝）以 JSON 行追加到 $NETGUN_LOG_DIR/netgun_pwm_events.jsonl
（默认 ~/netgun_logs）；请求带 NETGUN_REVIEW_DIR（run_interceptor_stack.sh 传给
拦截链）时同时写入该复盘目录 pwm_events.jsonl（含回低位结果与舵机输出回读）。

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
PWM_MIN_US, PWM_MAX_US = 1000, 2000  # PWM_AUX_MIN5/MAX5：归一化 −1..+1 ↔ 1000..2000 µs
MODULE_LOW_MAX_US = 1300  # 网枪触发模块：≤1300 µs 持续 100 ms = 低位（待命）
MODULE_HIGH_MIN_US = 1700  # ≥1700 µs 持续 100 ms = 高位（击发）；<1700 µs 持续 100 ms = 关闭
MARGIN_US = 50
ARM_RAMP_S = 0.5  # PX4 解锁/进执行器测试时输出从 DIS5 线性斜坡 0.5 s 到设定值
BANKS = {  # 输出域 → 参数名前缀/REV 参数/定时器核对；aux=FMU PWM OUT（AUX5），main=IO PWM OUT（MAIN5）
    "aux":  {"prefix": "PWM_AUX_",  "rev": "PWM_AUX_REV",  "timer": ("PWM_AUX_TIM1", 50)},
    "main": {"prefix": "PWM_MAIN_", "rev": "PWM_MAIN_REV", "timer": (None, None)},
}
DEFAULT_BANK = "aux"
CHANNEL_BIT5 = 1 << 4


def bank_checks(bank):
    """核对清单：FUNC5/DIS5/FAIL5/MIN5/MAX5（两域同名规则）+ 域定时器。

    main 域不核定时器（MAIN5–8 默认 50 Hz；MAIN_TIM1 属 MAIN1–4 不得动）。"""
    prefix = BANKS[bank]["prefix"]
    checks = [
        (prefix + "FUNC5", lambda v: v == ACTUATOR_SET1, "301"),
        (prefix + "DIS5", lambda v: v == 0 or 800 <= v <= MODULE_LOW_MAX_US - MARGIN_US, "0 或 800–1250"),
        (prefix + "FAIL5", lambda v: v in (0, -1) or 800 <= v <= MODULE_LOW_MAX_US - MARGIN_US,
         "0、-1 或 800–1250"),
        (prefix + "MIN5", lambda v: v == PWM_MIN_US, str(PWM_MIN_US)),
        (prefix + "MAX5", lambda v: v == PWM_MAX_US, str(PWM_MAX_US)),
    ]
    timer = BANKS[bank]["timer"]
    if timer[0]:
        checks.append((timer[0], lambda v, t=timer[1]: v == t, str(timer[1])))
    return tuple(checks)


def bank_conflicts(bank):
    """301 冲突扫描范围：两域 FUNC1–8 除本通道外全部（301 不得重复指派）。"""
    names = []
    for key in BANKS:
        prefix = BANKS[key]["prefix"]
        names += [prefix + "FUNC%d" % i for i in range(1, 9) if not (key == bank and i == 5)]
    return tuple(names)


def bank_note_names(bank):
    return ("COM_MOT_TEST_EN", "COM_PREARM_MODE", BANKS[bank]["rev"])

DEFAULT_PARAMS = {"low_us": 1000, "high_us": 2000, "hold_s": 1.0, "low_s": 0.3}
PARAM_SPECS = (  # 键、名称、单位、下限、上限、整数、范围依据
    ("low_us", "待命低位", "µs", PWM_MIN_US, MODULE_LOW_MAX_US - MARGIN_US, True,
     "模块 ≤1300 µs 判低位，留 50 µs；不低于 MIN5"),
    ("high_us", "击发高位", "µs", MODULE_HIGH_MIN_US + MARGIN_US, PWM_MAX_US, True,
     "模块 ≥1700 µs 判高位，留 50 µs；不高于 MAX5"),
    ("hold_s", "高位保持", "s", 0.3, 2.5, False, "模块要高位持续 0.1 s 才击发"),
    ("low_s", "低位确认", "s", 0.2, 2.0, False, "模块要低位持续 0.1 s 才待命"),
)
PARAM_KEYS = tuple(spec[0] for spec in PARAM_SPECS)
PARAM_OPTIONS = {"--low-us": "low_us", "--high-us": "high_us", "--hold-s": "hold_s", "--low-s": "low_s"}
PARAMS_FILE = "netgun_pwm.json"
PARAMS_SCHEMA = 1
PARAMS_META_KEYS = ("schema_version", "saved_at")
PARAMS_LIMIT_BYTES = 65536

HEARTBEAT_STALE_S = 3.0
ACK_TIMEOUT_S = 0.25
ACK_ATTEMPTS = 3
PARAM_PERIOD_S = 60.0
PARAM_RETRY_S = 10.0
PARAM_GAP_S = 0.02
LATCH_RETRY_S = 1.0
FIRE_TIMEOUT_S = 5.0
STATUS_TIMEOUT_S = 0.5
PARAMS_TIMEOUT_S = 3.0
CONNECT_TIMEOUT_S = 0.3
MAX_LINE = 65536
WANTED = (0, 22, 36, 77)  # HEARTBEAT, PARAM_VALUE, SERVO_OUTPUT_RAW, COMMAND_ACK
RESULT_NAMES = {0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED",
                3: "UNSUPPORTED", 4: "FAILED", 5: "IN_PROGRESS", 6: "CANCELLED"}
REFUSALS = {
    "BUSY": "上一次击发序列未结束（低位确认/高位保持/回低位中）",
    "NO_LINK": "飞控无心跳（MAVProxy 转发 127.0.0.1:14555 未通或飞控未上电）",
    "CONFIG": "飞控 PWM5 参数不符或未读到（核对当前输出域）",
    "PARAMS": "击发参数无效",
    "DISARMED": "飞控未解锁：不发击发高位（PX4 会锁存设定值，解锁瞬间就会输出）；台架测试用 --bench",
    "ARMED": "飞控已解锁：台架模式只在未解锁时可用",
    "MOT_TEST_DISABLED": "COM_MOT_TEST_EN≠1：飞控不接受执行器测试（台架模式不可用）",
    "LOW_FAILED": "飞控未确认待命低位（网枪模块要先见低位才会被高位触发），未发高位",
    "STOPPING": "击发服务正在停止，未发高位",
}


def log_dir():
    return (os.environ.get("NETGUN_LOG_DIR", "").strip()
            or os.path.expanduser("~/netgun_logs"))


def socket_path():
    return os.environ.get("NETGUN_PWM_SOCKET", "").strip() or os.path.join(
        log_dir(), "netgun_pwm.sock")


def config_dir():
    return (os.environ.get("NETGUN_CONFIG_DIR", "").strip()
            or os.path.expanduser("~/netgun_config"))


def params_path():
    return os.path.join(config_dir(), PARAMS_FILE)


def pwm_us(value):
    """归一化值 −1..+1 → 脉宽（MIN5..MAX5）。"""
    if value is None:
        return None
    return int(round(PWM_MIN_US + (value + 1.0) * 0.5 * (PWM_MAX_US - PWM_MIN_US)))


def unit_value(us):
    """脉宽 → DO_SET_ACTUATOR/ACTUATOR_TEST 的归一化值（PX4 按 MIN5..MAX5 线性换算、四舍五入）。"""
    return (float(us) - PWM_MIN_US) * 2.0 / (PWM_MAX_US - PWM_MIN_US) - 1.0


def seconds_text(value):
    text = "%.2f" % value
    return text[:-1] if text.endswith("0") else text


def plan_text(params):
    return "待命 %d µs → 击发 %d µs 保持 %s s" % (
        params["low_us"], params["high_us"], seconds_text(params["hold_s"]))


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


# ---------------------------------------------------------------- 击发参数

class ParamsError(ValueError):
    """击发参数无效。"""


def validate_params(raw):
    """完整校验四项击发参数，返回规整后的副本（µs 取整数，秒保留两位小数）。"""
    if not isinstance(raw, dict):
        raise ParamsError("击发参数需为 JSON 对象")
    unknown = sorted(str(key) for key in raw if key not in PARAM_KEYS)
    if unknown:
        raise ParamsError("未知参数：%s（可用 %s）" % ("、".join(unknown), "、".join(PARAM_KEYS)))
    params = {}
    for key, label, unit, low, high, integer, why in PARAM_SPECS:
        if key not in raw:
            raise ParamsError("缺少%s（%s）" % (label, key))
        value = raw[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ParamsError("%s需为数字" % label)
        if integer:
            if value != int(value):
                raise ParamsError("%s需为整数（µs）" % label)
            value = int(value)
        else:
            value = round(float(value), 2)
        if not low <= value <= high:
            raise ParamsError("%s %s %s 超出范围 %s–%s %s（%s）" % (
                label, value, unit, low, high, unit, why))
        params[key] = value
    return params


def _meta(source="default", path=None, saved_at=None, error=None, bank=None):
    return {"source": source, "path": path, "saved_at": saved_at, "error": error,
            "bank": bank or DEFAULT_BANK}


def load_params(path=None):
    """(params, meta)：文件不存在 → 默认值；文件无效 → (None, meta.error)，不静默退回默认。

    meta.bank = 输出域（"aux"=FMU PWM OUT/AUX5，"main"=IO PWM OUT/MAIN5）；无效 bank 视为文件无效。"""
    path = path or params_path()
    try:
        with open(path, encoding="utf-8") as f:
            text = f.read(PARAMS_LIMIT_BYTES + 1)
    except FileNotFoundError:
        return dict(DEFAULT_PARAMS), _meta(path=path)
    except (OSError, ValueError) as exc:
        return None, _meta("file", path, error="击发参数文件读取失败（%s）：%s" % (path, exc))
    try:
        if len(text) > PARAMS_LIMIT_BYTES:
            raise ParamsError("文件过大")
        document = json.loads(text)
        if not isinstance(document, dict):
            raise ParamsError("内容需为 JSON 对象")
        bank = document.get("bank", DEFAULT_BANK)
        if bank not in BANKS:
            raise ParamsError("bank 无效：%r（可用 aux/main）" % bank)
        params = validate_params({k: v for k, v in document.items()
                                  if k not in PARAMS_META_KEYS and k != "bank"})
    except ValueError as exc:
        return None, _meta("file", path, error="击发参数文件无效（%s）：%s" % (path, exc))
    saved_at = document.get("saved_at")
    return params, _meta("file", path, saved_at if isinstance(saved_at, str) else None, bank=bank)


def save_params(params, path=None, bank=None):
    """校验后原子写入（同目录临时文件 + rename），返回 meta。"""
    params = validate_params(params)
    if bank is not None and bank not in BANKS:
        raise ParamsError("bank 无效：%r（可用 aux/main）" % bank)
    path = path or params_path()
    stamp = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    if len(stamp) > 5 and stamp[-5] in "+-":
        stamp = stamp[:-2] + ":" + stamp[-2:]
    document = {"schema_version": PARAMS_SCHEMA, "saved_at": stamp}
    if bank:
        document["bank"] = bank
    document.update(params)
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    temporary = "%s.tmp%d" % (path, os.getpid())
    try:
        with open(temporary, "w", encoding="utf-8") as f:
            f.write(json.dumps(document, ensure_ascii=False, indent=2) + "\n")
            f.flush()
            os.fsync(f.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise
    return _meta("file", path, stamp, bank=bank or DEFAULT_BANK)


def remove_params(path=None):
    """恢复默认 = 删除参数文件。"""
    path = path or params_path()
    try:
        os.unlink(path)
    except FileNotFoundError:
        pass
    return _meta(path=path)


def options_info(params, meta):
    """状态/面板用：四项参数（无效时为 null）+ 来源/路径/保存时间/错误 + 默认值与范围 + 输出域。"""
    info = {key: (params or {}).get(key) for key in PARAM_KEYS}
    info.update(meta)
    info.setdefault("bank", DEFAULT_BANK)
    info["bank"] = info.get("bank") or DEFAULT_BANK
    info["defaults"] = dict(DEFAULT_PARAMS)
    info["limits"] = {spec[0]: [spec[3], spec[4]] for spec in PARAM_SPECS}
    return info


def params_info(path=None):
    """不经服务、直接读文件（服务未运行时面板/CLI 用）。"""
    return options_info(*load_params(path))


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
    """一行中文摘要（run_interceptor_stack.sh start/status、预检、日志）。"""
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
    if "high_us" not in options:
        return "击发服务是旧版本：板上重跑 install_netgun_pwm_link.sh"
    if not isinstance(options.get("high_us"), int):
        return "击发参数无效：%s" % (options.get("error") or "未知")
    idle = info.get("idle") or {}
    if not idle.get("latched"):
        return "待命低位未设好（%s）" % (
            idle.get("error") or "等待飞控确认 %s µs" % options.get("low_us"))
    text = "就绪（飞控 %s · %s · %s5 %s%s）" % (
        link.get("fc") or "?", "已解锁" if link.get("armed") else "未解锁",
        ((info.get("options") or {}).get("bank") or (options or {}).get("bank") or "aux").upper(),
        plan_text(options),
        " · 默认参数" if options.get("source") == "default" else "")
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


def _params_request(args):
    """命令行 → 服务请求：None=查看；{"reset": True}；{"set": {...}}。"""
    if args[:1] == ["save"]:
        text = sys.stdin.read(PARAMS_LIMIT_BYTES + 1)
        if len(text) > PARAMS_LIMIT_BYTES:
            raise ParamsError("请求内容过大")
        document = json.loads(text)
        if not isinstance(document, dict) or not set(document) <= {"config", "bank"}:
            raise ParamsError("请求格式无效（应为 {\"config\": {...}}，可选 \"bank\"）")
        if not isinstance(document.get("config", {}), dict):
            raise ParamsError("config 需为 JSON 对象")
        request = {"set": document.get("config", {})}
        if "bank" in document:
            request["bank"] = document["bank"]
        return request
    if args == ["--reset"]:
        return {"reset": True}
    changes, rest = {}, list(args)
    while rest:
        name = rest.pop(0)
        if name not in PARAM_OPTIONS or not rest:
            raise ParamsError("用法：params [--low-us N] [--high-us N] [--hold-s S] [--low-s S] | --reset | save")
        try:
            changes[PARAM_OPTIONS[name]] = float(rest.pop(0))
        except ValueError:
            raise ParamsError("%s 需要数字" % name)
    return {"set": changes} if changes else None


def _params_offline(req, reason):
    """服务未运行：直接读/写文件（下次启动生效）。"""
    note = "击发服务未运行（%s）" % reason
    if req is None:
        return {"ok": True, "service": False, "note": note + "：显示文件内容", "options": params_info()}
    if req.get("reset"):
        remove_params()
    else:
        current, current_meta = load_params()
        save_params(validate_params(dict(current or DEFAULT_PARAMS, **req["set"])),
                    bank=req.get("bank") or current_meta.get("bank") or DEFAULT_BANK)
    return {"ok": True, "service": False, "applied": False,
            "note": note + "：已写入文件，服务启动后生效", "options": params_info()}


def client_params(args):
    try:
        req = _params_request(args)
    except ValueError as exc:
        emit({"ok": False, "error": str(exc)})
        return 2
    try:
        reply = request(dict({"cmd": "params"}, **(req or {})), PARAMS_TIMEOUT_S)
        reply.setdefault("service", True)
    except LinkUnavailable as exc:
        try:
            reply = _params_offline(req, exc)
        except (ValueError, OSError) as error:
            reply = {"ok": False, "service": False, "error": str(error)}
    except ReplyTimeout as exc:
        reply = {"ok": False, "error": "%s（参数可能已保存，用 params 查看）" % exc}
    emit(reply)
    return 0 if reply.get("ok") else 1


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
        self.armed_t = None
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
                fresh = self.hb_t is None or now - self.hb_t > HEARTBEAT_STALE_S or self.fc != src
                if self.fc != src:
                    self.log("飞控 %d/%d 心跳" % src)
                elif self.armed is not None and armed != self.armed:
                    self.log("飞控%s" % ("已解锁" if armed else "已上锁"))
                # 解锁时刻按首次看到已解锁心跳记（晚于真实解锁，等待只会偏长）
                if not armed:
                    self.armed_t = None
                elif fresh or not self.armed or self.armed_t is None:
                    self.armed_t = now
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
                "armed_for_s": (round(now - self.armed_t, 1)
                                if self.armed and self.armed_t is not None else None),
                "peer": "%s:%d" % self.peer if self.peer else None,
                "rx_age_s": None if self.peer_t is None else round(now - self.peer_t, 2),
                "datagrams": self.datagrams,
                "bad": self.bad,
                "listen": "%s:%d" % self.listen,
            }

    def armed_since(self):
        """本服务看到飞控解锁的时刻（clock）；未解锁为 None。"""
        with self.cond:
            return self.armed_t if self.armed else None

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


def _brief_result(result):
    return {key: result.get(key) for key in ("result_name", "ack_ms", "attempts") if key in result}


class PwmService:
    def __init__(self, link, params=None, events_path=None, params_file=None,
                 clock=time.monotonic, log=log_line):
        self.link = link
        self.params_file = params_file or params_path()
        if params is None:
            self.params, self.params_meta = load_params(self.params_file)
        else:
            self.params, self.params_meta = validate_params(params), _meta("given")
        self.bank = self.params_meta.get("bank") or DEFAULT_BANK
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
        self.latch = None  # 飞控已 ACK 的待命低位 {"low_us", "t", "why"}
        self.latch_error = None
        self.latch_try = None
        self.path = None
        self.sock_ino = None

    # ---- 参数核对 ----

    def config(self):
        values, problems, missing, warnings = {}, [], [], []
        stamps = []
        for name, good, expected in bank_checks(self.bank):
            item = self.link.param(name)
            if item is None:
                missing.append(name)
                continue
            values[name] = item[0]
            stamps.append(item[1])
            if not good(item[0]):
                problems.append("%s=%s（应为 %s）" % (name, item[0], expected))
        for name in bank_conflicts(self.bank):
            item = self.link.param(name)
            if item is not None and item[0] == ACTUATOR_SET1:
                values[name] = item[0]
                problems.append("%s=301（Actuator Set 1 同时驱动另一路输出）" % name)
        for name in bank_note_names(self.bank):
            item = self.link.param(name)
            if item is not None:
                values[name] = item[0]
        rev_param = BANKS[self.bank]["rev"]
        if int(values.get(rev_param, 0)) & CHANNEL_BIT5:
            problems.append("%s=%s 含 5 号通道（输出反向：待命低位会变成高位）"
                            % (rev_param, values[rev_param]))
        if values.get("COM_MOT_TEST_EN", 1) != 1:
            warnings.append("COM_MOT_TEST_EN≠1，台架模式不可用")
        if values.get("COM_PREARM_MODE", 0) != 0:
            warnings.append("COM_PREARM_MODE≠0，未解锁时也会输出锁存的待命低位")
        ok = False if problems else (None if missing else True)
        age = None if not stamps else round(self.clock() - min(stamps), 1)
        return {"ok": ok, "values": values, "problems": problems, "missing": missing,
                "warnings": warnings, "age_s": age}

    def options(self):
        return options_info(self.params, self.params_meta)

    def idle(self):
        latch, params = self.latch, self.params
        latched = bool(latch and params and latch["low_us"] == params["low_us"])
        return {"latched": latched,
                "low_us": latch["low_us"] if latch else None,
                "age_s": None if not latch else round(self.clock() - latch["t"], 1),
                "why": latch["why"] if latch else None,
                "error": None if latched else self.latch_error}

    def _latch_low(self, why, low_us=None):
        """DO_SET_ACTUATOR 低位并等 ACK，ACCEPTED 才算已锁存。调用方持有 busy。"""
        low = self.params["low_us"] if low_us is None else low_us
        self.latch_try = self.clock()
        result = self.link.command(CMD_DO_SET_ACTUATOR, self._set_params(unit_value(low)))
        if result["result"] == 0:
            recovered = self.latch_error is not None
            self.latch = {"low_us": low, "t": self.clock(), "why": why}
            self.latch_error = None
            if why != "reset" or recovered:
                self.log("待命低位 %d µs 已锁存（%s；ACK %s ms）" % (low, why, result.get("ack_ms")))
        else:
            self.latch = None
            error = "飞控未确认低位 %d µs（%s）" % (low, result["result_name"])
            if error != self.latch_error:
                self.log("%s，每 %.0f s 重试" % (error, LATCH_RETRY_S), level="WARN")
            self.latch_error = error
        return result

    def maintain(self):
        """周期只读核对参数；锁存待命低位（链路恢复后重来）；记录链路/参数状态变化。"""
        link = self.link.snapshot()
        config = self.config()
        now = self.clock()
        if link["alive"] != self.was_alive:
            self.latch = None  # 链路中断/恢复：飞控可能重启过（锁存值已丢），恢复后重新锁存
            if link["alive"]:
                self.param_sent = None  # 立即重读参数
        self.was_alive = link["alive"]
        if link["alive"]:
            period = PARAM_PERIOD_S if config["ok"] is True else PARAM_RETRY_S
            if self.param_sent is None or now - self.param_sent >= period:
                self.param_sent = now
                for name in [n for n, _, _ in bank_checks(self.bank)] \
                        + list(bank_conflicts(self.bank)) + list(bank_note_names(self.bank)):
                    if self.stop.is_set():
                        break
                    try:
                        self.link.send("param_request_read", name.encode(), -1)
                    except OSError:
                        break
                    self.stop.wait(PARAM_GAP_S)
        if (link["alive"] and config["ok"] is True and self.params is not None
                and not self.idle()["latched"] and not self.stop.is_set()
                and (self.latch_try is None or now - self.latch_try >= LATCH_RETRY_S)
                and self.busy.acquire(blocking=False)):
            try:
                self._latch_low("链路就绪" if self.latch_error is None else "重试")
            finally:
                self.busy.release()
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
        idle = self.idle()
        return {
            "ok": True,
            "ready": bool(link["alive"] and config["ok"] is True and self.params is not None
                          and idle["latched"]),
            "link": link,
            "config": config,
            "options": self.options(),
            "idle": idle,
            "busy": self.busy.locked(),
            "last_fire": self.last_fire,
            "servo": self.link.servo_now(),
            "started": self.started,
        }

    # ---- 击发参数 ----

    def handle_params(self, request):
        """cmd=params：无 set/reset = 查看；set = 部分或全部修改；reset = 恢复默认（删文件）。"""
        reset = request.get("reset") is True
        changes = request.get("set")
        if not reset and changes is None:
            return {"ok": True, "options": self.options(), "idle": self.idle()}
        if not reset and not isinstance(changes, dict):
            return {"ok": False, "error": "set 需为 JSON 对象"}
        changes = dict(changes) if changes else {}
        bank = request.get("bank", changes.pop("bank", None))
        if bank is not None and bank not in BANKS:
            return {"ok": False, "error": "bank 无效：%r（可用 aux/main）" % bank}
        try:
            params = (dict(DEFAULT_PARAMS) if reset
                      else validate_params(dict(self.params or DEFAULT_PARAMS, **changes)))
        except ParamsError as exc:
            return {"ok": False, "error": str(exc)}
        new_bank = self.bank if (reset or bank is None) else bank
        link = self.link.snapshot()
        if link["alive"] and link["armed"]:
            return {"ok": False, "error": "飞控已解锁：上锁后再改击发参数"}
        if not self.busy.acquire(blocking=False):
            return {"ok": False, "error": "击发序列进行中，结束后再改"}
        try:
            try:
                meta = (remove_params(self.params_file) if reset
                        else save_params(params, self.params_file, bank=new_bank))
            except OSError as exc:
                return {"ok": False, "error": "击发参数写入失败：%s" % exc}
            old, self.params, self.params_meta = self.params, params, meta
            old_bank, self.bank = self.bank, new_bank
            self.log("击发参数%s：%s，低位确认 %s s" % (
                "恢复默认" if reset else "已保存", plan_text(params), seconds_text(params["low_s"])))
            if old is None or old["low_us"] != params["low_us"]:
                self.latch = None
                if link["alive"] and self.config()["ok"] is True:
                    self._latch_low("参数修改")
            if old_bank != self.bank:
                self.latch = None
                self.log("输出域切换：%s5（%s PWM OUT 座）" % (
                    self.bank.upper(),
                    "FMU" if self.bank == "aux" else "I/O"))
            return {"ok": True, "applied": True, "options": self.options(), "idle": self.idle()}
        finally:
            self.busy.release()

    # ---- 击发 ----

    def fire(self, request):
        started = self.clock()
        mode = "bench" if request.get("mode") == "bench" else "live"
        params = self.params
        record = {
            "event": "fire", "time": time.time(), "mode": mode,
            "shot_id": request.get("shot_id"), "reason": request.get("reason"),
            "source": request.get("source"), "ok": False,
        }
        if params:
            record.update(params, pwm_us=params["high_us"])
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
            if params is None:
                return self._refuse(record, "PARAMS", review_dir, started, self.params_meta.get("error"))
            if mode == "live" and not link["armed"]:
                return self._refuse(record, "DISARMED", review_dir, started)
            if mode == "bench" and link["armed"]:
                return self._refuse(record, "ARMED", review_dir, started)
            if mode == "bench" and config["values"].get("COM_MOT_TEST_EN", 1) != 1:
                return self._refuse(record, "MOT_TEST_DISABLED", review_dir, started)
            watch = self.link.watch_start()
            record["servo_before"] = self.link.servo_now()
            try:
                prepare = self._prepare_live if mode == "live" else self._prepare_bench
                refusal = prepare(record, params)
            except BaseException:
                self.link.watch_stop(watch)
                raise
            if refusal:
                record["servo"] = self.link.watch_stop(watch)
                return self._refuse(record, refusal[0], review_dir, started, refusal[1])
            high = unit_value(params["high_us"])
            if mode == "live":
                result = self.link.command(CMD_DO_SET_ACTUATOR, self._set_params(high))
            else:
                result = self.link.command(CMD_ACTUATOR_TEST, self._test_params(
                    high, params["hold_s"] + 0.5))
            record.update(result)
            record["ok"] = result["result"] == 0
            if not record["ok"]:
                record["reason_text"] = "飞控未接受击发高位（%s）" % result["result_name"]
            record["elapsed_ms"] = round((self.clock() - started) * 1000.0, 1)
            reply = {k: v for k, v in record.items() if k != "servo_before"}
            self.last_fire = dict(reply, pending=True)
            worker = threading.Thread(target=self._complete, args=(record, watch, review_dir, params),
                                      daemon=True)
            worker.start()
            handed_off = True
            self.log("击发 %s shot=%s 高位 %d µs %s ack=%s ms（%s 次）%s" % (
                mode, record["shot_id"], params["high_us"], result["result_name"],
                result.get("ack_ms", "—"), result.get("attempts"),
                "，先等低位 %s ms" % record["low_wait_ms"] if record.get("low_wait_ms") else ""),
                level="INFO" if record["ok"] else "ERROR")
            return reply
        finally:
            if not handed_off:
                self.busy.release()

    def _prepare_live(self, record, params):
        """高位前：低位已锁存（否则补发）且已输出满 low_s（解锁斜坡另计 0.5 s）。"""
        latch = self.latch
        if not (latch and latch["low_us"] == params["low_us"]):
            result = self._latch_low("击发前补发")
            record["pre_low"] = _brief_result(result)
            if result["result"] != 0:
                return "LOW_FAILED", "%d µs %s" % (params["low_us"], result["result_name"])
            latch = {"t": self.clock()}  # 不再读 self.latch：链路状态线程可能同时清掉它
        starts = [latch["t"]]
        armed_t = self.link.armed_since()
        if armed_t is not None:
            starts.append(armed_t + ARM_RAMP_S)
        wait = max(starts) + params["low_s"] - self.clock()
        if wait > 0:
            record["low_wait_ms"] = round(wait * 1000.0, 1)
            if self.stop.wait(wait):
                return "STOPPING", None
            link = self.link.snapshot()
            record["armed"] = link["armed"]
            if not link["alive"]:
                return "NO_LINK", None
            if not link["armed"]:
                return "DISARMED", "等待低位期间飞控已上锁"
        return None

    def _prepare_bench(self, record, params):
        """台架：执行器测试先给低位（PX4 进测试模式有 0.5 s 斜坡）再等 low_s。"""
        pre = ARM_RAMP_S + params["low_s"]
        result = self.link.command(CMD_ACTUATOR_TEST, self._test_params(
            unit_value(params["low_us"]), pre + 0.5))
        record["pre_low"] = _brief_result(result)
        if result["result"] != 0:
            self._release()
            return "LOW_FAILED", "%d µs %s" % (params["low_us"], result["result_name"])
        record["low_wait_ms"] = round(pre * 1000.0, 1)
        if self.stop.wait(pre):
            self._release()
            return "STOPPING", None
        return None

    def _release(self):
        return self.link.command(CMD_ACTUATOR_TEST, self._test_params(-1.0, 0.0))

    @staticmethod
    def _set_params(value):
        nan = float("nan")
        return (value, nan, nan, nan, nan, nan, 0.0)

    @staticmethod
    def _test_params(value, timeout_s):
        return (value, min(max(timeout_s, 0.0), 3.0), 0.0, 0.0, float(TEST_FUNCTION), 0.0, 0.0)

    def _complete(self, record, watch, review_dir, params):
        """保持 hold_s 后回低位（live：锁存低位；bench：低位 low_s 后释放）。"""
        try:
            self.stop.wait(params["hold_s"])
            reset = {"pwm_us": params["low_us"]}
            if record["mode"] == "live":
                reset.update(self._latch_low("reset", params["low_us"]))
            else:
                reset.update(self.link.command(CMD_ACTUATOR_TEST, self._test_params(
                    unit_value(params["low_us"]), params["low_s"] + 0.5)))
                self.stop.wait(params["low_s"])
                reset["release"] = self._release()["result_name"]
            record["reset"] = reset
            record["servo"] = self.link.watch_stop(watch)
            ok_reset = reset.get("result_name") == "ACCEPTED" and reset.get(
                "release", "ACCEPTED") == "ACCEPTED"
            self.log("回低位 %s：%s；舵机输出回读 %s" % (
                record["mode"], json.dumps(reset, ensure_ascii=False, sort_keys=True),
                json.dumps(record["servo"], sort_keys=True) if record["servo"] else "无变化/未收到"),
                level="INFO" if ok_reset else "ERROR")
        except Exception as exc:  # noqa: BLE001 —— 回低位失败也要记录并释放 busy
            record["reset_error"] = str(exc)
            self.log("回低位异常：%s" % exc, level="ERROR")
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
                elif cmd == "params":
                    self._send(conn, self.handle_params(request))
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


def _listen(text):
    host, _, port = text.rpartition(":")
    return (host or "127.0.0.1", int(port))


def serve(args):
    try:
        listen = _option(args, "--listen", LISTEN, _listen)
    except ValueError as exc:
        log_line("%s（--listen 取 主机:端口）" % exc, level="ERROR")
        return 2
    legacy = [name for name in ("--fire-value", "--safe-value", "--hold-s") if name in args]
    if legacy:
        log_line("已忽略旧参数 %s：击发参数改在 %s（netgun_pwm_link.py params 或面板「击发参数」）" % (
            "、".join(legacy), params_path()), level="WARN")
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
    service = PwmService(link)
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: service.stop.set())
    path = socket_path()
    try:
        sock = service.bind(path)
    except (OSError, RuntimeError) as exc:
        log_line("控制套接字未建立：%s" % exc, level="ERROR")
        link.close()
        return 1
    params = service.params
    log_line("击发服务启动：UDP %s:%d ↔ MAVProxy，控制 %s；%s" % (
        link.listen[0], link.listen[1], path,
        "%s5 %s → 回待命，低位确认 %s s（%s）" % (
            (service.bank if params else "aux").upper(),
            plan_text(params), seconds_text(params["low_s"]),
            "默认参数" if service.params_meta["source"] == "default" else service.params_file)
        if params else "击发参数无效：%s；拒绝击发，修正后用 params --reset 或面板重新保存"
        % service.params_meta["error"]), level="INFO" if params else "ERROR")
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
        # 正在进行的击发序列先收尾（stop 置位后等待/保持立即结束、马上回低位）
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
    if action == "params":
        return client_params(argv[2:])
    print("用法: netgun_pwm_link.py serve | fire [--bench] | status [--brief] | "
          "params [--low-us N] [--high-us N] [--hold-s S] [--low-s S] | params --reset | params save",
          file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
