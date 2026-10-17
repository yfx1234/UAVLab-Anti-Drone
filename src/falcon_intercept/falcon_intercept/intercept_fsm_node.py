from __future__ import annotations

import json

from .detection_utils import fresh_center_error_px
from .detection_utils import ros_stamp_to_seconds as _stamp_to_seconds
from .guidance import GuidanceConfig, compute_search_yaw_rate
from .intercept_fsm import FsmConfig, InterceptFsm, TargetObservation



def target_health_allows_state(
    health_json: str | None, state_stamp_s: float, maximum_stamp_delta_s: float = 0.05
) -> bool:
    if health_json is None:
        return False
    try:
        payload = json.loads(health_json)
        if payload.get("valid") is not True:
            return False
        measurement_stamp = float(payload["last_measurement_timestamp_s"])
        return abs(measurement_stamp - float(state_stamp_s)) <= float(maximum_stamp_delta_s)
    except (TypeError, ValueError, KeyError, json.JSONDecodeError):
        return False


def main(args=None) -> int:
    import rclpy
    from geometry_msgs.msg import Vector3Stamped
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from std_msgs.msg import Bool, Float32
    from std_msgs.msg import String

    class InterceptFsmNode(Node):
        def __init__(self) -> None:
            super().__init__("intercept_fsm_node")

            self.declare_parameter("target_state_topic", "/target/relative_state")
            self.declare_parameter("target_health_topic", "/target/state_health")
            self.declare_parameter("require_target_health", False)
            self.declare_parameter("target_health_stamp_tolerance_sec", 0.05)
            self.declare_parameter("front_detections_topic", "/stereo/front/detections_json")
            self.declare_parameter("back_detections_topic", "/stereo/back/detections_json")
            self.declare_parameter("state_topic", "/intercept/state")
            self.declare_parameter("fire_event_topic", "/intercept/fire_event")
            self.declare_parameter("cmd_velocity_topic", "/intercept/cmd_velocity")
            self.declare_parameter("abort_topic", "/intercept/abort")
            self.declare_parameter("mission_enable_topic", "/intercept/mission_enable")
            self.declare_parameter("vehicle_landed_topic", "/vehicle/landed")
            self.declare_parameter("search_detections_topic", "/search/detections_json")
            self.declare_parameter("yaw_rate_topic", "/intercept/cmd_yaw_rate")
            self.declare_parameter("land_request_topic", "/intercept/land_request")
            self.declare_parameter("command_frame_id", "body")
            self.declare_parameter("rate_hz", 10.0)
            self.declare_parameter("image_width", 640)
            self.declare_parameter("image_height", 480)
            self.declare_parameter("detection_conf_threshold", 0.5)
            self.declare_parameter("detection_timeout_sec", 0.5)
            self.declare_parameter("target_timeout_sec", 0.5)
            self.declare_parameter("lost_timeout_sec", 1.0)
            self.declare_parameter("fire_distance_m", 2.0)
            self.declare_parameter("center_threshold_px", 80.0)
            self.declare_parameter("fire_hold_frames", 3)
            self.declare_parameter("max_search_rounds", 0)
            self.declare_parameter("search_round_period_sec", 1.0)
            self.declare_parameter("track_confirm_frames", 1)
            self.declare_parameter("land_hold_sec", 1.0)
            self.declare_parameter("require_center_for_fire", True)
            self.declare_parameter("require_land_confirmation", True)
            self.declare_parameter("start_enabled", False)
            self.declare_parameter("kp_xy", 0.4)
            self.declare_parameter("kp_z", 0.3)
            self.declare_parameter("max_speed_xy", 0.8)
            self.declare_parameter("max_speed_z", 0.5)
            self.declare_parameter("search_yaw_kp", 0.004)
            self.declare_parameter("max_search_yaw_rate_rps", 0.35)
            self.declare_parameter("scan_yaw_rate_rps", 0.20)
            self.declare_parameter("max_task_time_sec", 120.0)

            guidance = GuidanceConfig(
                kp_xy=float(self.get_parameter("kp_xy").value),
                kp_z=float(self.get_parameter("kp_z").value),
                max_speed_xy=float(self.get_parameter("max_speed_xy").value),
                max_speed_z=float(self.get_parameter("max_speed_z").value),
                desired_distance_m=float(self.get_parameter("fire_distance_m").value),
                search_yaw_kp=float(self.get_parameter("search_yaw_kp").value),
                max_search_yaw_rate_rps=float(self.get_parameter("max_search_yaw_rate_rps").value),
                scan_yaw_rate_rps=float(self.get_parameter("scan_yaw_rate_rps").value),
            )
            config = FsmConfig(
                target_timeout_sec=float(self.get_parameter("target_timeout_sec").value),
                lost_timeout_sec=float(self.get_parameter("lost_timeout_sec").value),
                fire_distance_m=float(self.get_parameter("fire_distance_m").value),
                center_threshold_px=float(self.get_parameter("center_threshold_px").value),
                fire_hold_frames=int(self.get_parameter("fire_hold_frames").value),
                max_search_rounds=int(self.get_parameter("max_search_rounds").value),
                search_round_period_sec=float(self.get_parameter("search_round_period_sec").value),
                track_confirm_frames=int(self.get_parameter("track_confirm_frames").value),
                land_hold_sec=float(self.get_parameter("land_hold_sec").value),
                require_center_for_fire=bool(self.get_parameter("require_center_for_fire").value),
                require_land_confirmation=bool(self.get_parameter("require_land_confirmation").value),
                mission_start_enabled=bool(self.get_parameter("start_enabled").value),
                max_task_time_sec=float(self.get_parameter("max_task_time_sec").value),
                guidance=guidance,
            )
            self._fsm = InterceptFsm(config)
            self._command_frame_id = str(self.get_parameter("command_frame_id").value)
            self._image_width = int(self.get_parameter("image_width").value)
            self._image_height = int(self.get_parameter("image_height").value)
            self._min_conf = float(self.get_parameter("detection_conf_threshold").value)
            self._detection_timeout_sec = float(self.get_parameter("detection_timeout_sec").value)
            self._require_target_health = bool(
                self.get_parameter("require_target_health").value
            )
            self._target_health_stamp_tolerance_s = float(
                self.get_parameter("target_health_stamp_tolerance_sec").value
            )
            self._target_health_json: str | None = None
            self._pending_target_state: Odometry | None = None
            self._front_json: str | None = None
            self._back_json: str | None = None
            self._search_json: str | None = None
            self._abort_requested = False
            self._last_logged_state = ""

            event_qos = QoSProfile(
                depth=10,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._state_pub = self.create_publisher(String, str(self.get_parameter("state_topic").value), 10)
            self._fire_pub = self.create_publisher(
                String,
                str(self.get_parameter("fire_event_topic").value),
                event_qos,
            )
            self._cmd_pub = self.create_publisher(
                Vector3Stamped,
                str(self.get_parameter("cmd_velocity_topic").value),
                10,
            )
            self._yaw_pub = self.create_publisher(
                Float32, str(self.get_parameter("yaw_rate_topic").value), 10
            )
            self._land_pub = self.create_publisher(
                Bool, str(self.get_parameter("land_request_topic").value), event_qos
            )
            self.create_subscription(
                Odometry,
                str(self.get_parameter("target_state_topic").value),
                self._on_target_state,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                String,
                str(self.get_parameter("target_health_topic").value),
                self._on_target_health,
                10,
            )
            self.create_subscription(
                String,
                str(self.get_parameter("front_detections_topic").value),
                self._on_front_detections,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                String,
                str(self.get_parameter("back_detections_topic").value),
                self._on_back_detections,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Bool,
                str(self.get_parameter("abort_topic").value),
                self._on_abort,
                10,
            )
            self.create_subscription(
                Bool,
                str(self.get_parameter("mission_enable_topic").value),
                lambda msg: self._fsm.set_mission_enabled(bool(msg.data)),
                10,
            )
            self.create_subscription(
                Bool,
                str(self.get_parameter("vehicle_landed_topic").value),
                lambda msg: self._fsm.set_landed(bool(msg.data)),
                10,
            )
            self.create_subscription(
                String,
                str(self.get_parameter("search_detections_topic").value),
                self._on_search_detections,
                qos_profile_sensor_data,
            )

            rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
            self.create_timer(1.0 / rate_hz, self._on_timer)
            self.get_logger().info("intercept_fsm_node ready")

        def _on_front_detections(self, msg: String) -> None:
            self._front_json = str(msg.data)

        def _on_back_detections(self, msg: String) -> None:
            self._back_json = str(msg.data)

        def _on_search_detections(self, msg: String) -> None:
            self._search_json = str(msg.data)

        def _on_abort(self, msg: Bool) -> None:
            if bool(msg.data):
                self._abort_requested = True

        def _on_target_health(self, msg: String) -> None:
            self._target_health_json = str(msg.data)
            if self._require_target_health:
                try:
                    health_valid = json.loads(self._target_health_json).get("valid") is True
                    if not health_valid:
                        self._pending_target_state = None
                        self._fsm.invalidate_target()
                    elif self._pending_target_state is not None:
                        pending_stamp_s = _stamp_to_seconds(
                            self._pending_target_state.header.stamp
                        )
                        if target_health_allows_state(
                            self._target_health_json,
                            pending_stamp_s,
                            self._target_health_stamp_tolerance_s,
                        ):
                            pending = self._pending_target_state
                            self._pending_target_state = None
                            self._consume_target_state(pending)
                except (TypeError, json.JSONDecodeError):
                    self._pending_target_state = None
                    self._fsm.invalidate_target()

        def _on_target_state(self, msg: Odometry) -> None:
            target_stamp_s = _stamp_to_seconds(msg.header.stamp)
            if self._require_target_health and not target_health_allows_state(
                self._target_health_json,
                target_stamp_s,
                self._target_health_stamp_tolerance_s,
            ):
                self._pending_target_state = msg
                return
            self._pending_target_state = None
            self._consume_target_state(msg)

        def _consume_target_state(self, msg: Odometry) -> None:
            target_stamp_s = _stamp_to_seconds(msg.header.stamp)
            front_error = (
                None
                if self._front_json is None
                else fresh_center_error_px(
                    self._front_json,
                    self._image_width,
                    self._image_height,
                    self._min_conf,
                    target_stamp_s,
                    self._detection_timeout_sec,
                )
            )
            back_error = (
                None
                if self._back_json is None
                else fresh_center_error_px(
                    self._back_json,
                    self._image_width,
                    self._image_height,
                    self._min_conf,
                    target_stamp_s,
                    self._detection_timeout_sec,
                )
            )
            self._fsm.update_target(
                TargetObservation(
                    stamp_s=target_stamp_s,
                    position_m=(
                        float(msg.pose.pose.position.x),
                        float(msg.pose.pose.position.y),
                        float(msg.pose.pose.position.z),
                    ),
                    velocity_mps=(
                        float(msg.twist.twist.linear.x),
                        float(msg.twist.twist.linear.y),
                        float(msg.twist.twist.linear.z),
                    ),
                    front_center_error_px=front_error,
                    back_center_error_px=back_error,
                )
            )

        def _on_timer(self) -> None:
            now = self.get_clock().now()
            now_s = _stamp_to_seconds(now.to_msg())
            output = self._fsm.abort(now_s) if self._abort_requested else self._fsm.step(now_s)

            state_msg = String()
            state_msg.data = output.state_json
            self._state_pub.publish(state_msg)
            if output.state.value != self._last_logged_state:
                self.get_logger().info("FSM_STATE %s" % output.state.value)
                self._last_logged_state = output.state.value

            search_error = (
                None
                if self._search_json is None
                else fresh_center_error_px(
                    self._search_json,
                    self._image_width,
                    self._image_height,
                    self._min_conf,
                    now_s,
                    self._detection_timeout_sec,
                )
            )
            yaw_msg = Float32()
            yaw_msg.data = 0.0
            if output.state.value == "SEARCH":
                yaw_msg.data = compute_search_yaw_rate(
                    None if search_error is None else search_error[0], self._fsm.config.guidance
                )
            self._yaw_pub.publish(yaw_msg)

            land_msg = Bool()
            land_msg.data = bool(output.land_request)
            self._land_pub.publish(land_msg)

            if output.fire_event_json is not None:
                fire_msg = String()
                fire_msg.data = output.fire_event_json
                self._fire_pub.publish(fire_msg)
                self.get_logger().info("FIRE_EVENT %s" % output.fire_event_json)

            if output.cmd_velocity is not None:
                cmd = Vector3Stamped()
                cmd.header.stamp = now.to_msg()
                cmd.header.frame_id = self._command_frame_id
                cmd.vector.x = float(output.cmd_velocity[0])
                cmd.vector.y = float(output.cmd_velocity[1])
                cmd.vector.z = float(output.cmd_velocity[2])
                self._cmd_pub.publish(cmd)

    rclpy.init(args=args)
    node = None
    try:
        node = InterceptFsmNode()
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
