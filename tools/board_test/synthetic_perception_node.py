#!/usr/bin/env python3
"""虚拟相机感知注入节点（无硬件全链路测试用，2026-10-15）。

订阅 mock_px4 的 /test/mock_px4/state（位姿），把参数指定的固定世界目标
（NED 坐标）经相机模型投影为 PerceptionFrame 合同 v2 的 JSON 帧，发布到
/perception/observations_json（与真实 zed_perception_standalone 相同话题/
QoS/字段），由真实的 zed_perception_bridge 消费——即测试走的是真实桥、
真实估计器、真实制导/火控，仅感知源为受控合成。

相机模型：ZED 竖直向上安装，body->optical 旋转 q=(0,1,0,0)
（与 netgun_real.launch.py 的 camera_q_body_to_optical 一致）；
内参默认取 2026-10-15 板上实测值。深度在 0.35–10.0m 内报 mode=depth，
否则降级 bearing_only（对齐真实感知的深度质量门 0.3–10.5m）。
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

Q_BODY_TO_OPTICAL = (0.0, 1.0, 0.0, 0.0)


def quat_rotate(q, v):
    w, x, y, z = q
    vx, vy, vz = v
    # v' = q * v * q^-1（Hamilton）
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def quat_conjugate(q):
    return (q[0], -q[1], -q[2], -q[3])


class SyntheticPerceptionNode(Node):

    def __init__(self):
        super().__init__("synthetic_perception")
        self.declare_parameter("target_ned", "0.0,0.0,-11.6")
        self.declare_parameter("rate_hz", 15.0)
        self.declare_parameter("target_size_m", 0.30)
        self.declare_parameter("camera_fx", 1266.5)
        self.declare_parameter("camera_fy", 1266.5)
        self.declare_parameter("camera_cx", 1011.7)
        self.declare_parameter("camera_cy", 538.4)
        self.declare_parameter("image_width", 1920)
        self.declare_parameter("image_height", 1080)
        self.declare_parameter("run_id", "fullchain-test")
        self.declare_parameter(
            "topic", "/test/perception/observations_json")

        self._target = [
            float(v) for v in
            str(self.get_parameter("target_ned").value).split(",")]
        self._seq = 0
        self._state = None
        self._state_time_s = -1e9

        self.create_subscription(
            String, "/test/mock_px4/state", self._on_state, 10)
        self._pub = self.create_publisher(
            String, str(self.get_parameter("topic").value),
            qos_profile_sensor_data)

        period = 1.0 / float(self.get_parameter("rate_hz").value)
        self.create_timer(period, self._publish_frame)
        self.get_logger().info(
            "synthetic_perception ready: target_ned=%s" % self._target)

    def _on_state(self, msg):
        try:
            self._state = json.loads(msg.data)
            self._state_time_s = time.monotonic()
        except (json.JSONDecodeError, ValueError):
            pass

    def _publish_frame(self):
        stamp_ns = time.time_ns()
        if self._state is None or time.monotonic() - self._state_time_s > 1.0:
            return
        state = self._state
        pos = state.get("pos", [0.0, 0.0, 0.0])
        q_bn = tuple(state.get("q_body_to_ned", [1.0, 0.0, 0.0, 0.0]))

        rel_ned = [self._target[i] - pos[i] for i in range(3)]
        rel_body = quat_rotate(quat_conjugate(q_bn), rel_ned)
        rel_opt = quat_rotate(Q_BODY_TO_OPTICAL, rel_body)

        fx = float(self.get_parameter("camera_fx").value)
        fy = float(self.get_parameter("camera_fy").value)
        cx = float(self.get_parameter("camera_cx").value)
        cy = float(self.get_parameter("camera_cy").value)
        width = int(self.get_parameter("image_width").value)
        height = int(self.get_parameter("image_height").value)
        size_m = float(self.get_parameter("target_size_m").value)

        targets = []
        z = rel_opt[2]
        if z > 0.5:
            u = fx * rel_opt[0] / z + cx
            v = fy * rel_opt[1] / z + cy
            margin = 40.0
            if 0.0 - margin < u < width + margin and 0.0 - margin < v < height + margin:
                w_px = size_m / z * fx
                h_px = size_m / z * fy
                distance = math.sqrt(sum(c * c for c in rel_opt))
                nx = rel_opt[0] / z
                ny = rel_opt[1] / z
                depth_ok = 0.35 < z < 10.0
                targets.append({
                    "track_id": 1,
                    "class_id": 0,
                    "confidence": 0.95,
                    "bbox_xyxy": [u - w_px / 2.0, v - h_px / 2.0,
                                  u + w_px / 2.0, v + h_px / 2.0],
                    "detector_source": "yolo",
                    "score_kind": "class_confidence",
                    "class_confirmed": True,
                    "camera_motion_compensated": False,
                    "size_prior_m": size_m,
                    "pixel_u": u,
                    "pixel_v": v,
                    "unit_los": [rel_opt[0] / distance, rel_opt[1] / distance,
                                 rel_opt[2] / distance],
                    "horizontal_angle_rad": math.atan2(nx, 1.0),
                    "vertical_angle_rad": math.atan2(
                        ny, math.sqrt(1.0 + nx * nx)),
                    "mode": "depth" if depth_ok else "bearing_only",
                    "position_valid": depth_ok,
                    "depth_measured_valid": depth_ok,
                    "covariance_valid": False,
                    "depth_z_m": z if depth_ok else None,
                    "estimated_depth_z_m": None,
                    "distance_m": distance if depth_ok else None,
                    "position_camera": list(rel_opt) if depth_ok else None,
                    "depth_valid_pixels": 500,
                    "depth_valid_ratio": 0.6,
                    "depth_iqr_m": 0.05,
                    "reason": "accepted",
                    "calibration_id": "initial_prior",
                    "size_estimate_calibrated": False,
                    "position_definition": "supported_surface_center_ray",
                })

        self._seq += 1
        frame = {
            "schema_version": 2,
            "run_id": str(self.get_parameter("run_id").value),
            "boot_id": "fullchain-test-boot",
            "frame_id": "zed_left_camera_optical_frame",
            "frame_key": "fullchain_%d" % self._seq,
            "sequence": self._seq,
            "stamp_ns": int(stamp_ns),
            "monotonic_s": time.monotonic(),
            "image_width": width,
            "image_height": height,
            "model_id": "synthetic",
            "camera_serial": "synthetic",
            "detector_enabled": True,
            "yolo_enabled": True,
            "detector_state": 2,
            "generic_detector_enabled": False,
            "generic_detector_state": 0,
            "generic_sensitivity": "standard",
            "detector_generation": 1,
            "camera_motion_valid": False,
            "session_id": "",
            "segment_id": "",
            "recorded": None,
            "record_submitted": False,
            "targets": targets,
            "intrinsics": {"fx": fx, "fy": fy, "cx": cx, "cy": cy},
            "processing_latency_s": 0.01,
            "observations_fresh": True,
        }
        msg = String()
        msg.data = json.dumps(frame)
        self._pub.publish(msg)


def main(argv=None):
    rclpy.init(args=argv)
    node = SyntheticPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
