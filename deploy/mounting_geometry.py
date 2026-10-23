#!/usr/bin/env python3
"""网枪拦截器安装几何（2026-10-23 修订）：以网枪顶面中心为基准的相机、RTK 天线与飞控位置。

坐标为机体 FRD（前、右、下为正，米），原点 = 网枪顶面中心（枪口中心）。网枪装在
拦截器中轴线上，枪轴 = 拦截器轴线 = 机体 -Z；相机朝天，光轴与枪轴平行、无夹角
（2026-10-23 用户确认，不做枪轴核对，火控 boresight 保持 launch 默认 0）。
面板「拦截任务 → 安装参数」经本脚本读写；启动拦截链时 run_interceptor_stack.sh 用
launch-args 换算成 netgun_real.launch.py 参数。运行中修改不影响已启动的链，重启拦截链
后生效。

存储：~/netgun_config/mounting_geometry.json（运行配置，不入 Git；目录可用环境
变量 NETGUN_CONFIG_DIR 覆盖）。文件不存在时用 DEFAULT_CONFIG；文件存在但无效
时 launch-args 失败、拦截链拒绝启动（不静默退回默认值）。旧文件留档 history/。

派生量（拦截链内部原点仍是飞控，与 PX4 姿态、本地位置同一原点）：
  左目光心（ZED 坐标原点）= 相机顶面中心 + 光心深度（机体 +Z）− 半基线 × 右目方向
  估计器 camera_translation_body = 左目光心 − 飞控；camera_q_body_to_optical 按右目方向查表
  火控 muzzle_offset_body = 网枪顶面中心 − 飞控
  飞控 RTK 天线偏置建议值 EKF2_GPS_POS_X/Y/Z = RTK 天线 − 飞控（按飞控≈重心）。
  只给建议值，由飞手在 QGC 设置——面板与本脚本从不写飞控参数。

子命令：show | save（stdin：{"config": {...}}）| launch-args | summary | brief
"""
import datetime
import json
import math
import os
import sys
import tempfile

SCHEMA_VERSION = 2  # v1（2026-10-22）以网枪中心为基准、含枪轴偏角，已停用
REFERENCE = "netgun_top_center"
BASELINE_M = 0.05  # ZED X Mini 双目基线 50 mm；左目光心是 ZED 坐标原点
# ZED X Mini 光心在镜头面（相机顶面）以下约 2 cm：Stereolabs zed-ros2-description 的
# zedxm 模型前表面在相机中心前 15.5 mm、光心在中心后 5 mm（20.5 mm，取 2 cm）。
# 机身厚 36.7 mm（不含背面 GMSL2 接口），默认 RTK 深度按 3.7 cm 计。
OPTICAL_CENTER_BELOW_TOP_M = 0.02
POSITION_LIMIT_M = 0.5
HISTORY_KEEP = 20
REQUEST_LIMIT_BYTES = 65536
EDITABLE_KEYS = (
    "camera_top_m",
    "camera_right_eye",
    "rtk_antenna_m",
    "flight_controller_m",
)
METADATA_KEYS = ("schema_version", "reference", "frame", "saved_at")
FIELD_NAMES = {
    "camera_top_m": "相机顶面",
    "camera_right_eye": "右目朝向",
    "rtk_antenna_m": "RTK 天线",
    "flight_controller_m": "飞控",
}
_HALF = math.sqrt(0.5)
# 相机朝天（光轴 = 机体 -Z）。右目方向 = 光学系 +X（图像向右）在机体系的方向；
# q 为 body→optical 旋转（w, x, y, z），与估计器 camera_q_body_to_optical 同义。
RIGHT_EYE = {
    "forward": {"label": "朝机头", "axis": (1.0, 0.0, 0.0), "q_wxyz": (0.0, 1.0, 0.0, 0.0)},
    "backward": {"label": "朝机尾", "axis": (-1.0, 0.0, 0.0), "q_wxyz": (0.0, 0.0, 1.0, 0.0)},
    "right": {"label": "朝右", "axis": (0.0, 1.0, 0.0), "q_wxyz": (0.0, _HALF, _HALF, 0.0)},
    "left": {"label": "朝左", "axis": (0.0, -1.0, 0.0), "q_wxyz": (0.0, _HALF, -_HALF, 0.0)},
}
# 2026-10-23 用户口径（未实测）：网枪顶面与相机顶面（镜头面）平行且同高；相机中心在
# 网枪轴线右侧 6 cm、右目朝机头；RTK 天线在相机正下方、距相机底面 8.5 cm（顶面以下
# 3.7 + 8.5 = 12.2 cm）；飞控在网枪正下方约 18 cm（按距网枪顶面计）。
DEFAULT_CONFIG = {
    "camera_top_m": [0.0, 0.06, 0.0],
    "camera_right_eye": "forward",
    "rtk_antenna_m": [0.0, 0.06, 0.122],
    "flight_controller_m": [0.0, 0.0, 0.18],
}


class GeometryError(ValueError):
    """安装参数不合法或文件损坏（消息直接给操作员看）。"""


def config_dir():
    return os.environ.get("NETGUN_CONFIG_DIR") or os.path.expanduser("~/netgun_config")


def config_path():
    return os.path.join(config_dir(), "mounting_geometry.json")


def _number(value, label, limit, digits):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise GeometryError("%s需为数字" % label)
    value = float(value)
    if not math.isfinite(value):
        raise GeometryError("%s需为有限数字" % label)
    if abs(value) > limit + 1e-12:
        raise GeometryError("%s超出范围（±%g）" % (label, limit))
    value = round(value, digits)
    return 0.0 if value == 0 else value


def _vector(value, key, size, limit, digits, axes):
    name = FIELD_NAMES[key]
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise GeometryError("%s需为 %d 个数" % (name, size))
    return [
        _number(item, "%s%s" % (name, axis), limit, digits)
        for item, axis in zip(value, axes)
    ]


def validate_editable(raw):
    """校验可编辑字段（面板请求与文件内容共用），返回规范化副本。"""
    if not isinstance(raw, dict):
        raise GeometryError("安装参数需为对象")
    unknown = sorted(set(raw) - set(EDITABLE_KEYS))
    missing = [key for key in EDITABLE_KEYS if key not in raw]
    if unknown:
        raise GeometryError("未知安装参数字段：%s" % "、".join(unknown))
    if missing:
        raise GeometryError("缺少安装参数：%s" % "、".join(FIELD_NAMES[k] for k in missing))
    axes = ("前", "右", "下")
    eye = raw["camera_right_eye"]
    if not isinstance(eye, str) or eye not in RIGHT_EYE:
        raise GeometryError("右目朝向需为 %s 之一" % "、".join(sorted(RIGHT_EYE)))
    return {
        "camera_top_m": _vector(raw["camera_top_m"], "camera_top_m", 3, POSITION_LIMIT_M, 4, axes),
        "camera_right_eye": eye,
        "rtk_antenna_m": _vector(raw["rtk_antenna_m"], "rtk_antenna_m", 3, POSITION_LIMIT_M, 4, axes),
        "flight_controller_m": _vector(raw["flight_controller_m"], "flight_controller_m", 3, POSITION_LIMIT_M, 4, axes),
    }


def _validate_file(raw):
    if not isinstance(raw, dict):
        raise GeometryError("内容不是对象")
    if raw.get("schema_version") == 1:
        raise GeometryError("旧版格式（以网枪中心为基准、含枪轴偏角）已停用，请在面板核对后重新保存")
    if raw.get("schema_version") != SCHEMA_VERSION:
        raise GeometryError("schema_version 需为 %d" % SCHEMA_VERSION)
    if raw.get("reference") != REFERENCE or raw.get("frame") != "body_frd":
        raise GeometryError("基准需为网枪顶面中心、机体 FRD")
    extra = sorted(set(raw) - set(EDITABLE_KEYS) - set(METADATA_KEYS))
    if extra:
        raise GeometryError("未知字段：%s" % "、".join(extra))
    editable = {key: raw[key] for key in EDITABLE_KEYS if key in raw}
    saved_at = raw.get("saved_at")
    return validate_editable(editable), saved_at if isinstance(saved_at, str) else None


def _sub(a, b):
    return [round(x - y, 4) + 0.0 for x, y in zip(a, b)]


def derive(config):
    eye = RIGHT_EYE[config["camera_right_eye"]]
    half = BASELINE_M / 2.0
    top = config["camera_top_m"]
    optical = [top[0], top[1], top[2] + OPTICAL_CENTER_BELOW_TOP_M]  # 相机朝天：光心在顶面下方
    left_eye = [round(c - half * a, 4) + 0.0 for c, a in zip(optical, eye["axis"])]
    fc = config["flight_controller_m"]
    gps = _sub(config["rtk_antenna_m"], fc)
    return {
        "right_eye_label": eye["label"],
        "left_eye_m": left_eye,
        "camera_translation_body_m": _sub(left_eye, fc),
        "camera_q_body_to_optical_wxyz": list(eye["q_wxyz"]),
        "muzzle_offset_body_m": _sub([0.0, 0.0, 0.0], fc),
        "px4_gps_pos_m": {
            "EKF2_GPS_POS_X": gps[0],
            "EKF2_GPS_POS_Y": gps[1],
            "EKF2_GPS_POS_Z": gps[2],
        },
    }


def launch_arguments(derived):
    """netgun_real.launch.py 参数（name, 文本值），顺序固定。"""
    t = derived["camera_translation_body_m"]
    q = derived["camera_q_body_to_optical_wxyz"]
    m = derived["muzzle_offset_body_m"]
    pairs = [
        ("camera_translation_body_x_m", t[0]),
        ("camera_translation_body_y_m", t[1]),
        ("camera_translation_body_z_m", t[2]),
        ("camera_q_body_to_optical_w", q[0]),
        ("camera_q_body_to_optical_x", q[1]),
        ("camera_q_body_to_optical_y", q[2]),
        ("camera_q_body_to_optical_z", q[3]),
        ("muzzle_offset_body_x_m", m[0]),
        ("muzzle_offset_body_y_m", m[1]),
        ("muzzle_offset_body_z_m", m[2]),
    ]
    # repr(float) 为最短可回读文本且总带小数点（launch 侧按 float 解析）
    return [(name, repr(float(value) + 0.0)) for name, value in pairs]


def load():
    """返回 (config, source, saved_at)；source = file | default。文件无效抛 GeometryError。"""
    path = config_path()
    try:
        with open(path, encoding="utf-8") as stream:
            text = stream.read(REQUEST_LIMIT_BYTES + 1)
    except FileNotFoundError:
        return validate_editable(DEFAULT_CONFIG), "default", None
    except OSError as exc:
        raise GeometryError("安装参数文件读取失败：%s" % exc)
    try:
        if len(text) > REQUEST_LIMIT_BYTES:
            raise GeometryError("文件过大")
        config, saved_at = _validate_file(json.loads(text))
    except (GeometryError, ValueError) as exc:
        raise GeometryError("安装参数文件无效（%s）：%s" % (path, exc))
    return config, "file", saved_at


def describe():
    config, source, saved_at = load()
    return {
        "ok": True,
        "schema_version": SCHEMA_VERSION,
        "reference": REFERENCE,
        "frame": "body_frd",
        "path": config_path(),
        "source": source,
        "saved_at": saved_at,
        "config": config,
        "derived": derive(config),
    }


def _write_atomic(path, text):
    directory = os.path.dirname(path)
    os.makedirs(directory, exist_ok=True)
    handle, temporary = tempfile.mkstemp(prefix=".mounting_geometry.", suffix=".tmp", dir=directory)
    try:
        with os.fdopen(handle, "w", encoding="utf-8") as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def _archive_previous(path, stamp):
    try:
        with open(path, encoding="utf-8") as stream:
            previous = stream.read(REQUEST_LIMIT_BYTES + 1)
    except FileNotFoundError:
        return
    history = os.path.join(os.path.dirname(path), "history")
    os.makedirs(history, exist_ok=True)
    base = os.path.join(history, "mounting_geometry_%s" % stamp)
    target, index = base + ".json", 1
    while os.path.exists(target):
        target, index = "%s_%d.json" % (base, index), index + 1
    _write_atomic(target, previous)
    kept = sorted(name for name in os.listdir(history) if name.startswith("mounting_geometry_"))
    for name in kept[:-HISTORY_KEEP]:
        try:
            os.unlink(os.path.join(history, name))
        except OSError:
            pass


def save(raw):
    config = validate_editable(raw)
    now = datetime.datetime.now().astimezone()
    document = {
        "schema_version": SCHEMA_VERSION,
        "reference": REFERENCE,
        "frame": "body_frd",
        "saved_at": now.isoformat(timespec="seconds"),
    }
    document.update(config)
    path = config_path()
    _archive_previous(path, now.strftime("%Y%m%d_%H%M%S"))
    _write_atomic(path, json.dumps(document, ensure_ascii=False, indent=2) + "\n")
    return describe()


def summary_record():
    """启动日志用的单行记录（interceptor_status.py 解析 NETGUN_GEOMETRY 行）。"""
    info = describe()
    derived = info["derived"]
    return {
        "source": info["source"],
        "saved_at": info["saved_at"],
        "config": info["config"],
        "camera_translation_body_m": derived["camera_translation_body_m"],
        "camera_q_body_to_optical_wxyz": derived["camera_q_body_to_optical_wxyz"],
        "muzzle_offset_body_m": derived["muzzle_offset_body_m"],
    }


def _cm(vector):
    return "前 %.1f 右 %.1f 下 %.1f cm" % tuple(value * 100.0 for value in vector)


def brief():
    info = describe()
    config = info["config"]
    head = "默认值（未保存）" if info["source"] == "default" else "已保存 %s" % (info["saved_at"] or "时间未知")
    return "安装参数 %s：相机顶面 %s，右目%s；RTK %s；飞控 %s" % (
        head,
        _cm(config["camera_top_m"]),
        info["derived"]["right_eye_label"],
        _cm(config["rtk_antenna_m"]),
        _cm(config["flight_controller_m"]),
    )


def _read_request():
    text = sys.stdin.read(REQUEST_LIMIT_BYTES + 1)
    if len(text) > REQUEST_LIMIT_BYTES:
        raise GeometryError("请求内容过大")
    request = json.loads(text)
    if not isinstance(request, dict) or set(request) != {"config"}:
        raise GeometryError("请求格式无效")
    return request["config"]


def main(argv):
    command = argv[1] if len(argv) > 1 else "show"
    try:
        if command == "show":
            output = describe()
        elif command == "save":
            output = save(_read_request())
        elif command == "launch-args":
            for name, value in launch_arguments(describe()["derived"]):
                print("%s:=%s" % (name, value))
            return 0
        elif command == "summary":
            print("NETGUN_GEOMETRY " + json.dumps(summary_record(), ensure_ascii=False, separators=(",", ":")))
            return 0
        elif command == "brief":
            print(brief())
            return 0
        else:
            raise GeometryError("未知子命令：%s" % command)
    except (GeometryError, ValueError, OSError) as exc:
        if command in ("launch-args", "summary", "brief"):
            print("安装参数无效：%s" % exc, file=sys.stderr)
            return 2
        print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False))
        return 1
    print(json.dumps(output, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
