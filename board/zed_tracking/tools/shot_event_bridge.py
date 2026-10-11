#!/usr/bin/env python3
"""shot_event_bridge:track_hold 射击 JSON -> 虚拟弹丸裁判接口桥。

把 /zed_tracking/shot_event(JSON)翻译成裁判要求的 VirtualShotEvent,
并持续发布目标相对真值 Odometry 与任务原点。
真值来源参数化:
  truth_source=sim     使用 /sim/target_truth(闭环仿真,绝对位置);
  truth_source=track   使用滤波输出(实飞自证模式)。
弹速必须与裁判配置一致(默认 400 m/s)。
"""

from __future__ import annotations

import json
import math


def shot_direction_from_yaw(yaw_rad):
    """物理枪轴 = 机头方向在 NED 下的单位向量(裁判 physical 轴)。"""
    return (math.cos(float(yaw_rad)), math.sin(float(yaw_rad)), 0.0)


def sight_direction(pursuer_ned, target_ned):
    """虚拟枪指向 = 目标视线方向(等效于带俯仰指向的枪架,裁判 solver 轴)。"""
    delta = [float(target_ned[i]) - float(pursuer_ned[i]) for i in range(3)]
    norm = math.sqrt(sum(value * value for value in delta))
    if norm < 1.0e-6:
        return (1.0, 0.0, 0.0)
    return (delta[0] / norm, delta[1] / norm, delta[2] / norm)


def main(args=None):
    import rclpy
    from geometry_msgs.msg import PointStamped
    from nav_msgs.msg import Odometry
    from px4_msgs.msg import VehicleLocalPosition
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from virtual_interception_interfaces.msg import VirtualShotEvent

    class ShotEventBridge(Node):
        def __init__(self):
            super().__init__("shot_event_bridge")
            self.declare_parameter("projectile_speed_mps", 400.0)
            self.declare_parameter("truth_source", "sim")  # sim | track
            self.declare_parameter("truth_rate_hz", 50.0)
            self.declare_parameter("sim_truth_topic", "/sim/target_truth")
            self.declare_parameter("aim_threshold_deg", 10.0)
            number = lambda name: float(self.get_parameter(name).value)  # noqa: E731
            self._speed = number("projectile_speed_mps")
            self._truth_source = str(self.get_parameter("truth_source").value)
            self._aim_threshold = math.radians(number("aim_threshold_deg"))

            self._own = None
            self._target_abs = None      # 目标绝对 NED(track 或 sim 真值)
            self._target_vel = None
            self._last_shot_id = 0

            sensor_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE)
            self.create_subscription(
                VehicleLocalPosition, "/fmu/out/vehicle_local_position",
                self._on_local, sensor_qos)
            self.create_subscription(
                String, "/zed_tracking/shot_event", self._on_shot, 10)
            if self._truth_source == "sim":
                self.create_subscription(
                    String, str(self.get_parameter("sim_truth_topic").value),
                    self._on_sim_truth, 10)
            else:
                self.create_subscription(
                    String, "/zed_tracking/track_json", self._on_track, 10)

            origin_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self._origin_pub = self.create_publisher(
                PointStamped, "/tracking/task_origin_ned", origin_qos)
            self._truth_pub = self.create_publisher(
                Odometry, "/evaluation/relative_truth", 20)
            self._shot_pub = self.create_publisher(
                VirtualShotEvent, "/tracking/virtual_shot_event", 10)
            self.create_timer(
                1.0 / max(1.0, number("truth_rate_hz")), self._publish_truth)
            self.get_logger().warning(
                f"shot bridge ready speed={self._speed} truth={self._truth_source}")

        def _now(self):
            return self.get_clock().now()

        def _on_local(self, message):
            self._own = {
                "x": float(message.x), "y": float(message.y), "z": float(message.z),
                "vx": float(message.vx), "vy": float(message.vy),
                "vz": float(message.vz),
            }

        def _on_sim_truth(self, message):
            try:
                payload = json.loads(message.data)
            except ValueError:
                return
            if payload.get("pos_ned"):
                self._target_abs = list(payload["pos_ned"])
                self._target_vel = [0.0, 0.0, 0.0]

        def _on_track(self, message):
            try:
                payload = json.loads(message.data)
            except ValueError:
                return
            if payload.get("valid") and payload.get("pos_ned"):
                self._target_abs = list(payload["pos_ned"])
                self._target_vel = list(payload.get("vel_ned") or [0.0, 0.0, 0.0])

        def _publish_truth(self):
            origin = PointStamped()
            origin.header.stamp = self._now().to_msg()
            origin.point.x = 0.0
            origin.point.y = 0.0
            origin.point.z = 0.0
            self._origin_pub.publish(origin)
            if self._target_abs is None or self._own is None:
                return
            truth = Odometry()
            truth.header.stamp = self._now().to_msg()
            truth.header.frame_id = "pursuer_local_ned"
            truth.pose.pose.position.x = self._target_abs[0] - self._own["x"]
            truth.pose.pose.position.y = self._target_abs[1] - self._own["y"]
            truth.pose.pose.position.z = self._target_abs[2] - self._own["z"]
            self._truth_pub.publish(truth)

        def _on_shot(self, message):
            try:
                shot = json.loads(message.data)
            except ValueError:
                return
            if self._own is None or not shot.get("target_pos_ned"):
                return
            aim_error = shot.get("aim_error_deg")
            range_m = shot.get("range_m")
            valid = (
                aim_error is not None and abs(aim_error) <= math.degrees(self._aim_threshold)
                and range_m is not None and 6.5 <= range_m <= 8.5
            )
            tof = max(1.0e-3, float(range_m) / self._speed)
            target_pos = list(shot["target_pos_ned"])
            target_vel = list(shot.get("target_vel_ned") or [0.0, 0.0, 0.0])
            # shot_direction 喂视线方向:裁判 solver 轴沿视线飞行,
            # 命中与否由指向误差决定;机头物理轴由裁判自行从姿态取。
            direction = sight_direction(
                [self._own["x"], self._own["y"], self._own["z"]], target_pos)
            event = VirtualShotEvent()
            event.interface_version = VirtualShotEvent.INTERFACE_VERSION_V1
            event.header.stamp = self._now().to_msg()
            event.header.frame_id = "pursuer_local_ned"
            self._last_shot_id += 1
            event.shot_id = self._last_shot_id
            event.projectile_speed_mps = self._speed
            event.predicted_time_of_flight_s = tof
            event.shot_direction_ned.x, event.shot_direction_ned.y, \
                event.shot_direction_ned.z = direction
            event.pursuer_position_ned.x = self._own["x"]
            event.pursuer_position_ned.y = self._own["y"]
            event.pursuer_position_ned.z = self._own["z"]
            event.pursuer_velocity_ned.x = self._own["vx"]
            event.pursuer_velocity_ned.y = self._own["vy"]
            event.pursuer_velocity_ned.z = self._own["vz"]
            event.estimated_target_position_ned.x = target_pos[0]
            event.estimated_target_position_ned.y = target_pos[1]
            event.estimated_target_position_ned.z = target_pos[2]
            event.estimated_target_velocity_ned.x = target_vel[0]
            event.estimated_target_velocity_ned.y = target_vel[1]
            event.estimated_target_velocity_ned.z = target_vel[2]
            event.predicted_intercept_point_ned.x = target_pos[0] + target_vel[0] * tof
            event.predicted_intercept_point_ned.y = target_pos[1] + target_vel[1] * tof
            event.predicted_intercept_point_ned.z = target_pos[2] + target_vel[2] * tof
            event.estimated_range_m = float(range_m)
            event.actual_attack_axis_error_rad = math.radians(aim_error or 0.0)
            event.predicted_lateral_sigma_m = 0.3
            event.maximum_position_variance_m2 = 1.0
            event.solution_valid = bool(valid)
            self._shot_pub.publish(event)
            self.get_logger().warning(
                f"bridge shot #{self._last_shot_id} valid={valid} "
                f"range={range_m:.2f} aim={aim_error:.1f}deg")

    rclpy.init(args=args)
    node = None
    try:
        node = ShotEventBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
