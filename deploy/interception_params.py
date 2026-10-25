#!/usr/bin/env python3
"""拦截参数（击发后安全后撤 / 开火延迟）的文件配置与 launch 透传。

面板「拦截参数」区与本脚本共用 ~/netgun_config/interception_params.json；
run_interceptor_stack.sh start 时经 `launch-args` 子命令转成
`name:=value` 参数行传给 netgun_real.launch.py（命令行显式参数仍可覆盖）。
栈运行中保存只写文件，下次启动生效。

用法：
  interception_params.py params            # 读取（JSON，含默认值与范围）
  interception_params.py save              # 从 stdin {"config": {...}} 保存
  interception_params.py launch-args       # 输出 name:=value 行（无文件=空）
"""
import json
import os
import sys
from datetime import datetime, timezone, timedelta

PARAM_KEYS = ("post_shot_retreat_m", "retreat_timeout_s", "attack_enable_delay_s")
DEFAULT_PARAMS = {
    "post_shot_retreat_m": 3.0,   # 击发后沿机体后向水平后撤距离（米），0=旧行为
    "retreat_timeout_s": 4.5,     # 后撤到位判据的超时兜底（秒）
    "attack_enable_delay_s": 2.0, # 就位后延迟开火（秒）；比赛跟踪计分需 ~30
}
LIMITS = {
    "post_shot_retreat_m": [0.0, 8.0],
    "retreat_timeout_s": [2.0, 10.0],
    "attack_enable_delay_s": [0.0, 120.0],
}
CONFIG_PATH = os.path.join(
    os.environ.get("NETGUN_CONFIG_DIR",
                   os.path.expanduser("~/netgun_config")),
    "interception_params.json")


class ParamsError(ValueError):
    pass


def load_params():
    """返回 (params|None, saved_at|None)。文件不存在=（None, None）；
    文件损坏抛 ParamsError（拒绝静默回退默认值）。"""
    if not os.path.exists(CONFIG_PATH):
        return None, None
    try:
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            doc = json.load(f)
    except (OSError, ValueError) as exc:
        raise ParamsError("配置文件损坏：%s（%s）" % (CONFIG_PATH, exc))
    saved_at = doc.get("saved_at") if isinstance(doc, dict) else None
    params = doc.get("params") if isinstance(doc, dict) else doc
    if not isinstance(params, dict):
        raise ParamsError("配置文件格式无效：%s" % CONFIG_PATH)
    return validate_params(params), saved_at


def validate_params(params):
    """白名单键 + 数字 + 范围校验，返回规范化后的 dict。"""
    if not isinstance(params, dict) or not params:
        raise ParamsError("config 需为非空 JSON 对象")
    unknown = set(params) - set(PARAM_KEYS)
    if unknown:
        raise ParamsError("未知参数：%s" % ", ".join(sorted(unknown)))
    out = {}
    for key, value in params.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) \
                or not (value == value) or value in (float("inf"), float("-inf")):
            raise ParamsError("%s 需为有限数字" % key)
        value = float(value)
        lo, hi = LIMITS[key]
        if not lo <= value <= hi:
            raise ParamsError(
                "%s=%.3g 超出范围 [%.3g, %.3g]" % (key, value, lo, hi))
        out[key] = value
    return out


def params_info():
    params, saved_at = load_params()
    if params is None:
        return {"ok": True, "options": {
            "path": CONFIG_PATH, "source": "default", "saved_at": None,
            "values": dict(DEFAULT_PARAMS), "defaults": dict(DEFAULT_PARAMS),
            "limits": {k: list(v) for k, v in LIMITS.items()},
            "applied": "未保存过（栈按 launch 默认值运行）",
        }}
    merged = dict(DEFAULT_PARAMS)
    merged.update(params)
    return {"ok": True, "options": {
        "path": CONFIG_PATH, "source": "file", "saved_at": saved_at,
        "values": merged, "defaults": dict(DEFAULT_PARAMS),
        "limits": {k: list(v) for k, v in LIMITS.items()},
        "applied": "已保存；栈下次启动时生效（当前栈沿用启动时的值）",
    }}


def save_params(params):
    params = validate_params(params)
    current, _ = load_params() if os.path.exists(CONFIG_PATH) else (None, None)
    if current is None:
        current = dict(DEFAULT_PARAMS)
    current.update(params)
    os.makedirs(os.path.dirname(CONFIG_PATH), exist_ok=True)
    doc = {"params": current,
           "saved_at": datetime.now(timezone(timedelta(hours=8))).isoformat(
               timespec="seconds")}
    tmp = CONFIG_PATH + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(doc, f, ensure_ascii=False, indent=2, sort_keys=True)
        f.write("\n")
    os.replace(tmp, CONFIG_PATH)
    return params_info()


def launch_args():
    params, _ = load_params()
    if params is None:
        return []
    merged = dict(DEFAULT_PARAMS)
    merged.update(params)
    # 始终带小数点（30 → 30.0）：launch 对无小数点的字符串按 int 推断，
    # 节点 double 参数会因类型不符崩进程（launch 侧另有 ParameterValue 兜底）
    return ["%s:=%.6g" % (key, merged[key]) if merged[key] % 1 else
            "%s:=%.1f" % (key, merged[key]) for key in PARAM_KEYS if key in merged]


def main(argv):
    try:
        if argv == ["params"]:
            emit(params_info())
            return 0
        if argv == ["launch-args"]:
            for line in launch_args():
                print(line)
            return 0
        if argv == ["save"]:
            text = sys.stdin.read(65536)
            document = json.loads(text)
            if not isinstance(document, dict) or set(document) != {"config"}:
                raise ParamsError('请求格式无效（应为 {"config": {...}}）')
            emit(save_params(document["config"]))
            return 0
    except ParamsError as exc:
        emit({"ok": False, "error": str(exc)})
        return 2
    except ValueError as exc:
        emit({"ok": False, "error": "请求 JSON 无效：%s" % exc})
        return 2
    emit({"ok": False, "error": "用法：params | save | launch-args"})
    return 2


def emit(doc):
    print(json.dumps(doc, ensure_ascii=False))


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
