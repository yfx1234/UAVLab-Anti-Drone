#!/usr/bin/env python3
"""感知桥节点：PerceptionFrame v2(JSON) -> DepthObservation V1。

订阅 /perception/observations_json（std_msgs/String，SensorDataQoS，与
/perception/observations 同帧同数据），按策略选出唯一目标后映射发布到
/perception/depth/observation（anti_drone_interfaces/DepthObservation V1，
SensorDataQoS），供 netgun 拦截链 relative_estimator 消费。

订阅 JSON 通道而非自定义消息通道，使拦截工作区无需在构建期依赖
zed_perception_interfaces（消费端免重建消息包）。

1Hz 发布 /debug/bridge_state（std_msgs/String，key=value），风格对齐
/debug/{guidance,strike,estimator}_state，便于按 estimator->strike->guidance
之外的桥接环节归因。其中 intrinsics_fx/fy/cx/cy 透传感知上报的相机内参，
供核对 relative_estimator 的 camera_fx 等 launch 参数（现用 1274 假设值）。
"""

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from anti_drone_interfaces.msg import DepthObservation

from . import mapping

_RATE_WINDOW_S = 3.0


class PerceptionBridgeNode(Node):

    def __init__(self):
        super().__init__("zed_perception_bridge")
        self.declare_parameter("input_json_topic", "/perception/observations_json")
        self.declare_parameter("output_topic", "/perception/depth/observation")
        self.declare_parameter("debug_topic", "/debug/bridge_state")
        self.declare_parameter("selection", "confidence")
        self.declare_parameter("allow_generic", False)
        self.declare_parameter("min_confidence", 0.0)
        self.declare_parameter("debug_period_s", 1.0)

        self._counters = {}
        self._last_reason = "init"
        self._last_mode = ""
        self._last_track_id = 0
        self._last_conf = 0.0
        self._last_depth_valid = False
        self._last_stamp_age_ms = -1.0
        self._last_intrinsics = {}
        self._stamps = {"received": [], "forwarded": []}
        self._warned = set()

        input_topic = str(self.get_parameter("input_json_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        debug_topic = str(self.get_parameter("debug_topic").value)
        debug_period = float(self.get_parameter("debug_period_s").value)

        self.create_subscription(
            String, input_topic, self._on_frame, qos_profile_sensor_data
        )
        self._obs_pub = self.create_publisher(
            DepthObservation, output_topic, qos_profile_sensor_data
        )
        self._debug_pub = self.create_publisher(String, debug_topic, 10)
        self.create_timer(debug_period, self._publish_debug)
        self.get_logger().info(
            "bridge: %s -> %s (selection=%s)"
            % (input_topic, output_topic, self.get_parameter("selection").value)
        )

    # ---- 主路径 ----

    def _on_frame(self, msg):
        now_ns = self.get_clock().now().nanoseconds
        self._tick("received", now_ns)
        try:
            frame = json.loads(msg.data)
        except (json.JSONDecodeError, ValueError) as exc:
            self._drop("json_error:%s" % exc)
            return

        reason = mapping.frame_reject_reason(frame)
        if reason is not None:
            self._drop("schema:" + reason)
            return

        intrinsics = frame.get("intrinsics")
        if isinstance(intrinsics, dict):
            self._last_intrinsics = intrinsics

        if not mapping.frame_is_fresh(frame):
            self._drop("not_fresh")
            return

        policy = str(self.get_parameter("selection").value)
        allow_generic = bool(self.get_parameter("allow_generic").value)
        target, reason = mapping.select_target(
            frame["targets"], policy=policy, allow_generic=allow_generic
        )
        if target is None:
            self._drop(reason)
            return

        min_confidence = float(self.get_parameter("min_confidence").value)
        try:
            confidence = float(target.get("confidence", 0.0))
        except (TypeError, ValueError):
            confidence = 0.0
        if confidence < min_confidence:
            self._drop("low_confidence")
            return

        try:
            fields = mapping.build_observation(frame, target)
        except mapping.BridgeMappingError as exc:
            self._drop("mapping:" + str(exc))
            return

        out = DepthObservation()
        out.interface_version = fields["interface_version"]
        out.header.stamp = rclpy.time.Time(
            nanoseconds=fields["stamp_ns"]
        ).to_msg()
        out.header.frame_id = fields["frame_id"]
        out.sequence = fields["sequence"]
        out.track_id = fields["track_id"]
        out.visible = fields["visible"]
        out.confidence = fields["confidence"]
        out.image_width = fields["image_width"]
        out.image_height = fields["image_height"]
        out.pixel_u = fields["pixel_u"]
        out.pixel_v = fields["pixel_v"]
        out.bounding_box_valid = fields["bounding_box_valid"]
        out.bounding_box_width_px = fields["bounding_box_width_px"]
        out.bounding_box_height_px = fields["bounding_box_height_px"]
        out.angles_valid = fields["angles_valid"]
        out.horizontal_angle_rad = fields["horizontal_angle_rad"]
        out.vertical_angle_rad = fields["vertical_angle_rad"]
        out.angle_covariance = fields["angle_covariance"]
        out.depth_valid = fields["depth_valid"]
        out.depth_z_m = fields["depth_z_m"]
        out.range_m = fields["range_m"]
        out.position_optical.x = fields["position_optical"][0]
        out.position_optical.y = fields["position_optical"][1]
        out.position_optical.z = fields["position_optical"][2]
        out.position_covariance = fields["position_covariance"]

        self._obs_pub.publish(out)
        self._tick("forwarded", now_ns)
        self._last_reason = "ok"
        self._last_mode = str(target.get("mode", ""))
        self._last_track_id = out.track_id
        self._last_conf = out.confidence
        self._last_depth_valid = out.depth_valid
        self._last_stamp_age_ms = (now_ns - fields["stamp_ns"]) / 1e6

    # ---- 调试与计数 ----

    def _tick(self, key, now_ns):
        self._counters[key] = self._counters.get(key, 0) + 1
        self._stamps.setdefault(key, []).append(now_ns)
        window = [t for t in self._stamps[key] if now_ns - t <= _RATE_WINDOW_S * 1e9]
        self._stamps[key] = window[-100:]

    def _rate(self, key):
        now_ns = self.get_clock().now().nanoseconds
        window = [t for t in self._stamps.get(key, []) if now_ns - t <= _RATE_WINDOW_S * 1e9]
        return len(window) / _RATE_WINDOW_S

    def _drop(self, reason):
        self._counters["dropped"] = self._counters.get("dropped", 0) + 1
        self._last_reason = reason
        if reason not in self._warned:
            self._warned.add(reason)
            self.get_logger().warning("drop frame: %s" % reason)

    def _publish_debug(self):
        dropped = self._counters.get("dropped", 0)
        intr = self._last_intrinsics
        parts = [
            "received=%d" % self._counters.get("received", 0),
            "in_hz=%.1f" % self._rate("received"),
            "forwarded=%d" % self._counters.get("forwarded", 0),
            "out_hz=%.1f" % self._rate("forwarded"),
            "dropped_total=%d" % dropped,
            "last_reason=%s" % self._last_reason,
            "last_mode=%s" % (self._last_mode or "none"),
            "last_track_id=%d" % self._last_track_id,
            "last_conf=%.3f" % self._last_conf,
            "last_depth_valid=%d" % int(self._last_depth_valid),
            "last_stamp_age_ms=%.0f" % self._last_stamp_age_ms,
            "intrinsics_fx=%.1f" % float(intr.get("fx", 0.0)),
            "intrinsics_fy=%.1f" % float(intr.get("fy", 0.0)),
            "intrinsics_cx=%.1f" % float(intr.get("cx", 0.0)),
            "intrinsics_cy=%.1f" % float(intr.get("cy", 0.0)),
        ]
        msg = String()
        msg.data = " ".join(parts)
        self._debug_pub.publish(msg)


def main(argv=None):
    rclpy.init(args=argv)
    node = PerceptionBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
