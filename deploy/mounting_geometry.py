#!/usr/bin/env python3
"""网枪拦截器安装几何（2026-10-22）：以网枪中心为基准的相机、RTK 天线与飞控位置。

坐标为机体 FRD（前、右、下为正，米），原点 = 网枪中心（网枪装在拦截器中轴线
上，枪轴标称竖直朝天 = 机体 -Z，与相机光轴同向）。面板「拦截任务 → 安装参数」
经本脚本读写；启动拦截链时 run_interceptor_stack.sh 用 launch-args 换算成
netgun_real.launch.py 参数。运行中修改不影响已启动的链，重启拦截链后生效。

存储：~/netgun_config/mounting_geometry.json（运行配置，不入 Git；目录可用环境
变量 NETGUN_CONFIG_DIR 覆盖）。文件不存在时用 DEFAULT_CONFIG；文件存在但无效
时 launch-args 失败、拦截链拒绝启动（不静默退回默认值）。旧文件留档 history/。

派生量（拦截链内部原点仍是飞控，与 PX4 姿态、本地位置同一原点）：
  左目（ZED 坐标原点）= 相机中心 − 半基线 × 右目方向（相机朝天）
  估计器 camera_translation_body = 左目 − 飞控；camera_q_body_to_optical 按右目方向查表
  火控 muzzle_offset_body = 网枪中心 − 飞控；boresight_pitch/yaw 由枪轴前偏/右偏换算
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

SCHEMA_VERSION = 1
BASELINE_M = 0.05  # ZED X Mini 双目基线 50 mm；左目光心是 ZED 坐标原点
POSITION_LIMIT_M = 0.5
TILT_LIMIT_DEG = 10.0
HISTORY_KEEP = 20
REQUEST_LIMIT_BYTES = 65536
EDITABLE_KEYS = (
    "camera_center_m",
    "camera_right_eye",
    "rtk_antenna_m",
    "flight_controller_m",
    "gun_axis_tilt_deg",
)
METADATA_KEYS = ("schema_version", "reference", "frame", "saved_at")
FIELD_NAMES = {
    "camera_center_m": "相机中心",
    "camera_right_eye": "右目朝向",
    "rtk_antenna_m": "RTK 天线",
    "flight_controller_m": "飞控",
    "gun_axis_tilt_deg": "枪轴偏角",
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
# 2026-10-22 用户口径：相机中心在网枪轴线右侧 6 cm、右目朝前；RTK 天线约在相机
# 正下方 8.5 cm；飞控位置未给，默认与网枪中心重合（同在中轴线上）；高度差待实测。
DEFAULT_CONFIG = {
    "camera_center_m": [0.0, 0.06, 0.0],
    "camera_right_eye": "forward",
    "rtk_antenna_m": [0.0, 0.06, 0.085],
    "flight_controller_m": [0.0, 0.0, 0.0],
    "gun_axis_tilt_deg": [0.0, 0.0],
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
        "camera_center_m": _vector(raw["camera_center_m"], "camera_center_m", 3, POSITION_LIMIT_M, 4, axes),
        "camera_right_eye": eye,
        "rtk_antenna_m": _vector(raw["rtk_antenna_m"], "rtk_antenna_m", 3, POSITION_LIMIT_M, 4, axes),
        "flight_controller_m": _vector(raw["flight_controller_m"], "flight_controller_m", 3, POSITION_LIMIT_M, 4, axes),
        "gun_axis_tilt_deg": _vector(raw["gun_axis_tilt_deg"], "gun_axis_tilt_deg", 2, TILT_LIMIT_DEG, 2, ("前偏", "右偏")),
    }


def _validate_file(raw):
    if not isinstance(raw, dict):
        raise GeometryError("内容不是对象")
    if raw.get("schema_version") != SCHEMA_VERSION:
        raise GeometryError("schema_version 需为 %d" % SCHEMA_VERSION)
    if raw.get("reference") != "netgun_center" or raw.get("frame") != "body_frd":
        raise GeometryError("基准需为网枪中心、机体 FRD")
    extra = sorted(set(raw) - set(EDITABLE_KEYS) - set(METADATA_KEYS))
    if extra:
        raise GeometryError("未知字段：%s" % "、".join(extra))
    editable = {key: raw[key] for key in EDITABLE_KEYS if key in raw}
    saved_at = raw.get("saved_at")
    return validate_editable(editable), saved_at if isinstance(saved_at, str) else None


def _sub(a, b):
    return [round(x - y, 4) + 0.0 for x, y in zip(a, b)]


def gun_axis_from_tilt(tilt_deg):
    """枪轴前偏/右偏（°，相对机体 -Z）→ 机体系单位向量与火控 boresight pitch/yaw。

    火控 gun_axis_body(p, y) = Rz(y)·Ry(p)·(0,0,-1) = (-sin p cos y, -sin p sin y, -cos p)。
    取 p = -atan(hypot(tan f, tan r))、y = atan2(tan r, tan f)，即得
    normalize(tan f, tan r, -1)；偏角为 0 时 p = y = 0（与"枪管同轴光轴"旧口径逐位一致）。
    """
    tf = math.tan(math.radians(tilt_deg[0]))
    tr = math.tan(math.radians(tilt_deg[1]))
    spread = math.hypot(tf, tr)
    norm = math.sqrt(1.0 + spread * spread)
    axis = [tf / norm + 0.0, tr / norm + 0.0, -1.0 / norm]
    if spread < 1e-12:
        return axis, 0.0, 0.0
    return axis, -math.atan(spread), math.atan2(tr, tf)


def derive(config):
    eye = RIGHT_EYE[config["camera_right_eye"]]
    half = BASELINE_M / 2.0
    center = config["camera_center_m"]
    left_eye = [round(c - half * a, 4) + 0.0 for c, a in zip(center, eye["axis"])]
    fc = config["flight_controller_m"]
    axis, pitch, yaw = gun_axis_from_tilt(config["gun_axis_tilt_deg"])
    gps = _sub(config["rtk_antenna_m"], fc)
    return {
        "right_eye_label": eye["label"],
        "left_eye_m": left_eye,
        "camera_translation_body_m": _sub(left_eye, fc),
        "camera_q_body_to_optical_wxyz": list(eye["q_wxyz"]),
        "muzzle_offset_body_m": _sub([0.0, 0.0, 0.0], fc),
        "gun_axis_body": axis,
        "boresight_pitch_rad": pitch,
        "boresight_yaw_rad": yaw,
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
        ("boresight_pitch_rad", derived["boresight_pitch_rad"]),
        ("boresight_yaw_rad", derived["boresight_yaw_rad"]),
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
        "reference": "netgun_center",
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
        "reference": "netgun_center",
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
        "boresight_pitch_rad": derived["boresight_pitch_rad"],
        "boresight_yaw_rad": derived["boresight_yaw_rad"],
    }


def _cm(vector):
    return "前 %.1f 右 %.1f 下 %.1f cm" % tuple(value * 100.0 for value in vector)


def brief():
    info = describe()
    config = info["config"]
    head = "默认值（未保存）" if info["source"] == "default" else "已保存 %s" % (info["saved_at"] or "时间未知")
    return "安装参数 %s：相机中心 %s，右目%s；RTK %s；飞控 %s；枪轴偏角 前 %.2f° 右 %.2f°" % (
        head,
        _cm(config["camera_center_m"]),
        info["derived"]["right_eye_label"],
        _cm(config["rtk_antenna_m"]),
        _cm(config["flight_controller_m"]),
        config["gun_axis_tilt_deg"][0],
        config["gun_axis_tilt_deg"][1],
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
