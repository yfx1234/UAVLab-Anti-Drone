"""PerceptionFrame(v2, JSON 通道) -> DepthObservation(V1) 的纯映射逻辑。

不依赖 rclpy/ROS 消息类型，可在无 ROS 环境下用 pytest 单测。

JSON 契约来源：zed_perception_standalone 的 /perception/observations_json
（schema_version=2，与 /perception/observations 同帧同数据，不适用字段为 null）。
输出契约来源：anti_drone_interfaces/msg/DepthObservation.msg（V1 稳定接口，
header.stamp 必须是图像曝光时刻、frame 必须是标准 optical 系 +X右 +Y下 +Z光轴）。

字段对应：
  stamp_ns                          -> header.stamp（透传原图时间戳，禁止重打）
  frame_id                          -> header.frame_id（zed_left_camera_optical_frame）
  sequence                          -> sequence
  targets[i].track_id               -> track_id（屏蔽 generic 的 0x80000000 标志位）
  targets[i].confidence             -> confidence
  image_width / image_height        -> 同名
  targets[i].pixel_u / pixel_v      -> 同名（估计器预门要求有限）
  targets[i].bbox_xyxy              -> bounding_box_{width,height}_px
  targets[i].horizontal/vertical_angle_rad -> 同名（angles_valid 恒 True）
  targets[i].depth_measured_valid   -> depth_valid。仅实测深度置位；size_estimate
                                       估距降级为 bearing-only（估计器原生支持该
                                       退化路径，nominal_range 12m）
  targets[i].depth_z_m / distance_m -> depth_z_m / range_m（仅 depth_valid 时取值）
  targets[i].position_camera        -> position_optical（同一相机光学系，仅
                                       depth_valid 时填充，否则零向量）

协方差为常数近似（横向 5cm、深度 30cm，不随距离变化）：估计器在协方差
有限且对角为正时会直接采用该值（relative_estimator
measurement_covariance_ned），精确噪声模型待外场标定（R1-4 备注）。
"""

import math

SUPPORTED_SCHEMA_VERSION = 2
OUTPUT_INTERFACE_VERSION = 1

# generic 检测轨道 ID 的标志位（zed_perception_standalone.cpp association.assign）
GENERIC_TRACK_FLAG = 0x80000000

# rad^2，约 0.005 rad (0.29°) 1σ/轴，行优先 2x2
DEFAULT_ANGLE_COVARIANCE = (2.5e-5, 0.0, 0.0, 2.5e-5)
# m^2，横向 5cm、深度 30cm 1σ，行优先 3x3
DEFAULT_POSITION_COVARIANCE = (
    0.0025, 0.0, 0.0,
    0.0, 0.0025, 0.0,
    0.0, 0.0, 0.09,
)

SELECTION_POLICIES = ("confidence", "nearest", "largest")


class BridgeMappingError(ValueError):
    """映射失败（字段形态不符合契约）。"""


def _finite_or_none(value):
    if isinstance(value, bool):  # bool 是 int 子类，不算数值
        return None
    if isinstance(value, (int, float)) and math.isfinite(value):
        return float(value)
    return None


def _target_valid(target):
    """估计器预门的最低要求：像素坐标为有限数值。"""
    return _finite_or_none(target.get("pixel_u")) is not None and \
        _finite_or_none(target.get("pixel_v")) is not None


def _bbox_area(target):
    bbox = target.get("bbox_xyxy")
    if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
        return 0.0
    return max(0.0, float(bbox[2]) - float(bbox[0])) * \
        max(0.0, float(bbox[3]) - float(bbox[1]))


def frame_reject_reason(frame):
    """帧级校验。返回 None 表示可处理，否则返回拒绝原因（进 bridge_state）。"""
    if not isinstance(frame, dict):
        return "not_object"
    version = frame.get("schema_version")
    if version != SUPPORTED_SCHEMA_VERSION:
        return "schema_version=%s" % (version,)
    for key in ("stamp_ns", "sequence", "image_width", "image_height", "targets"):
        if key not in frame:
            return "missing:%s" % key
    if not isinstance(frame["targets"], list):
        return "targets_not_list"
    if not isinstance(frame["stamp_ns"], int) or frame["stamp_ns"] <= 0:
        return "bad_stamp_ns"
    if not isinstance(frame["image_width"], int) or frame["image_width"] <= 0:
        return "bad_image_width"
    if not isinstance(frame["image_height"], int) or frame["image_height"] <= 0:
        return "bad_image_height"
    return None


def frame_is_fresh(frame):
    """转发门：识别开启且观测新鲜（不新鲜帧 targets 已被感知侧清空）。"""
    return bool(frame.get("observations_fresh")) and \
        bool(frame.get("detector_enabled"))


def select_target(targets, policy="confidence", allow_generic=False,
                  previous=None):
    """按策略从 targets 选出唯一目标。

    返回 (target, reason)：target 为 None 时 reason 说明原因。
    默认排除 generic 轨道（class_confirmed=False），只认 YOLO 类别确认目标。
    previous（R1-4 sticky）：上一帧转发的 {"track_id","u","v","age_s"}——
    同 ID 直接续选；0.3s 内像素距离 ≤150px 的目标视为同一目标，防止双
    目标/误检在场时逐帧跳变把估计打断。
    """
    candidates = [t for t in targets if isinstance(t, dict) and _target_valid(t)]
    if not allow_generic:
        confirmed = [t for t in candidates if t.get("class_confirmed")]
    else:
        confirmed = candidates
    if not confirmed:
        if candidates:
            return None, "no_class_confirmed_target"
        return None, "no_valid_target"
    # R1-4 sticky：先续上一帧目标，再做策略排序
    if previous is not None:
        prev_id = previous.get("track_id")
        if prev_id is not None:
            for t in confirmed:
                if (int(t.get("track_id", 0)) & 0x7FFFFFFF) == prev_id:
                    return t, None
        age = previous.get("age_s")
        pu, pv = previous.get("u"), previous.get("v")
        if age is not None and 0.0 <= age <= 0.3 and \
            pu is not None and pv is not None:
            near = []
            for t in confirmed:
                u = _finite_or_none(t.get("pixel_u"))
                v = _finite_or_none(t.get("pixel_v"))
                if u is None or v is None:
                    continue
                d = math.hypot(u - pu, v - pv)
                if d <= 150.0:
                    near.append((d, t))
            if near:
                near.sort(key=lambda item: item[0])
                return near[0][1], None
    if policy == "nearest":
        ranged = [
            t for t in confirmed
            if _finite_or_none(t.get("distance_m")) is not None
        ]
        if ranged:
            return min(ranged, key=lambda t: t["distance_m"]), None
        # 无有效距离信息时退化为 confidence 策略
    if policy == "largest":
        return max(confirmed, key=_bbox_area), None
    # 默认 confidence：置信度优先，并列取框面积大者
    return max(
        confirmed,
        key=lambda t: (
            _finite_or_none(t.get("confidence")) or 0.0,
            _bbox_area(t),
        ),
    ), None


def build_observation(frame, target):
    """把选中的目标映射为 DepthObservation 字段 dict。

    返回 dict 的键与 DepthObservation.msg 一一对应（header 拆为
    stamp_ns/frame_id）。字段形态不符时抛 BridgeMappingError，由调用方丢弃
    该帧并在 bridge_state 中计数。
    """
    pixel_u = _finite_or_none(target.get("pixel_u"))
    pixel_v = _finite_or_none(target.get("pixel_v"))
    if pixel_u is None or pixel_v is None:
        raise BridgeMappingError("bad_pixel")

    bbox = target.get("bbox_xyxy")
    if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
        raise BridgeMappingError("bad_bbox")
    try:
        bbox = [float(x) for x in bbox]
    except (TypeError, ValueError):
        raise BridgeMappingError("bad_bbox")

    horizontal = _finite_or_none(target.get("horizontal_angle_rad"))
    vertical = _finite_or_none(target.get("vertical_angle_rad"))
    angles_valid = horizontal is not None and vertical is not None

    depth_valid = bool(target.get("depth_measured_valid"))
    depth_z = _finite_or_none(target.get("depth_z_m"))
    distance = _finite_or_none(target.get("distance_m"))
    if depth_valid and (depth_z is None or depth_z <= 0.0):
        # 深度标志与数值不一致：降级为 bearing-only，不转发可疑距离
        depth_valid = False
    if depth_valid and distance is None:
        depth_valid = False

    position = [0.0, 0.0, 0.0]
    depth_inconsistent = False
    if depth_valid:
        raw = target.get("position_camera")
        values = None
        if isinstance(raw, (list, tuple)) and len(raw) == 3:
            values = [_finite_or_none(x) for x in raw]
        if values is None or not all(v is not None for v in values):
            # R1-4：深度有效但相机系位置缺失 → 同样不信深度（旧实现会
            # 发零向量位置，估计器把 (0,0,0) 当成真量测）
            depth_valid = False
            depth_inconsistent = True
        else:
            position = values
            if abs(position[2] - depth_z) > 0.05 + 0.05 * abs(depth_z):
                depth_valid = False
                depth_inconsistent = True
        if not depth_valid:
            position = [0.0, 0.0, 0.0]

    track_id = target.get("track_id", 0)
    if not isinstance(track_id, int):
        track_id = 0

    return {
        "interface_version": OUTPUT_INTERFACE_VERSION,
        "stamp_ns": int(frame["stamp_ns"]),
        "frame_id": str(frame.get("frame_id", "zed_left_camera_optical_frame")),
        "sequence": int(frame["sequence"]),
        "track_id": track_id & 0x7FFFFFFF,
        "visible": True,
        "confidence": _finite_or_none(target.get("confidence")) or 0.0,
        "image_width": int(frame["image_width"]),
        "image_height": int(frame["image_height"]),
        "pixel_u": pixel_u,
        "pixel_v": pixel_v,
        "bounding_box_valid": True,
        "bounding_box_width_px": max(0.0, bbox[2] - bbox[0]),
        "bounding_box_height_px": max(0.0, bbox[3] - bbox[1]),
        "angles_valid": angles_valid,
        "horizontal_angle_rad": horizontal if angles_valid else 0.0,
        "vertical_angle_rad": vertical if angles_valid else 0.0,
        "angle_covariance": list(DEFAULT_ANGLE_COVARIANCE),
        "depth_valid": depth_valid,
        "depth_z_m": depth_z if depth_valid else 0.0,
        "range_m": distance if depth_valid else 0.0,
        "position_optical": position,
        "depth_inconsistent": depth_inconsistent,
        "position_covariance": list(DEFAULT_POSITION_COVARIANCE),
    }
