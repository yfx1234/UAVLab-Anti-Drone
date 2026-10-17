"""Safety adapter for field-identical upstream PX4 setpoint passthrough."""

from __future__ import annotations

import json
import math

from .px4_adapter_node import evaluate_adapter_gate, normalize_mode


def selected_setpoint_kind(
    *,
    position: bool,
    velocity: bool,
    acceleration: bool,
    attitude: bool,
    body_rate: bool,
    thrust_and_torque: bool,
    direct_actuator: bool,
) -> str | None:
    """Return the only two control classes approved by the integration contract."""

    if body_rate or thrust_and_torque or direct_actuator:
        return None
    if attitude and not (position or velocity or acceleration):
        return "attitude"
    if not attitude and (position or velocity or acceleration):
        return "trajectory"
    return None


def finite_vector(values, length: int) -> bool:
    try:
        sequence = tuple(float(value) for value in values)
    except (TypeError, ValueError):
        return False
    return len(sequence) >= length and all(math.isfinite(value) for value in sequence[:length])


def valid_attitude_payload(q_d, thrust_body) -> bool:
    if not finite_vector(q_d, 4) or not finite_vector(thrust_body, 3):
        return False
    quaternion = tuple(float(value) for value in q_d[:4])
    return sum(value * value for value in quaternion) > 1.0e-8


def valid_trajectory_payload(message, mode) -> bool:
    selected = []
    if bool(mode.position):
        selected.append(getattr(message, "position", ()))
    if bool(mode.velocity):
        selected.append(getattr(message, "velocity", ()))
    if bool(mode.acceleration):
        selected.append(getattr(message, "acceleration", ()))
    return bool(selected) and all(finite_vector(vector, 3) for vector in selected)


def main(args=None) -> int:
    import rclpy
    from px4_msgs.msg import (
        OffboardControlMode,
        TrajectorySetpoint,
        VehicleAttitude,
        VehicleAttitudeSetpoint,
        VehicleLandDetected,
        VehicleLocalPosition,
        VehicleStatus,
    )
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, String

    class Px4UpstreamAdapter(Node):
        def __init__(self) -> None:
            super().__init__("px4_upstream_adapter")
            self.declare_parameter("mode", "shadow")
            self.declare_parameter("allow_live_control", False)
            self.declare_parameter("calibration_ready", False)
            self.declare_parameter("rate_hz", 50.0)
            self.declare_parameter("command_timeout_s", 0.10)
            self.declare_parameter("attitude_timeout_s", 0.25)
            self.declare_parameter("status_timeout_s", 1.5)
            self.declare_parameter("px4_namespace", "/fmu")
            self.declare_parameter("upstream_prefix", "/intercept/upstream_px4")

            self._mode = normalize_mode(str(self.get_parameter("mode").value))
            self._allow_live = bool(self.get_parameter("allow_live_control").value)
            self._calibration_ready = bool(self.get_parameter("calibration_ready").value)
            self._rate_hz = max(20.0, float(self.get_parameter("rate_hz").value))
            self._command_timeout_s = max(
                0.02, float(self.get_parameter("command_timeout_s").value)
            )
            self._attitude_timeout_s = max(
                0.02, float(self.get_parameter("attitude_timeout_s").value)
            )
            self._status_timeout_s = max(
                0.1, float(self.get_parameter("status_timeout_s").value)
            )
            if self._mode == "live" and (not self._allow_live or not self._calibration_ready):
                raise RuntimeError(
                    "live mode refused: allow_live_control and calibration_ready must both be true"
                )

            self._control_mode = None
            self._control_mode_received_s = None
            self._trajectory = None
            self._trajectory_received_s = None
            self._attitude_setpoint = None
            self._attitude_setpoint_received_s = None
            self._attitude_received_s = None
            self._local_position = None
            self._local_position_received_s = None
            self._status = None
            self._status_received_s = None
            self._landed = True
            self._land_detected_received = False
            self._takeover_latched = False
            self._offboard_seen = False
            self._last_reason = ""

            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            upstream = str(self.get_parameter("upstream_prefix").value).rstrip("/")
            px4 = str(self.get_parameter("px4_namespace").value).rstrip("/")
            self.create_subscription(
                OffboardControlMode,
                f"{upstream}/offboard_control_mode",
                self._on_control_mode,
                qos,
            )
            self.create_subscription(
                TrajectorySetpoint,
                f"{upstream}/trajectory_setpoint",
                self._on_trajectory,
                qos,
            )
            self.create_subscription(
                VehicleAttitudeSetpoint,
                f"{upstream}/vehicle_attitude_setpoint",
                self._on_attitude_setpoint,
                qos,
            )
            self.create_subscription(
                VehicleAttitude, f"{px4}/out/vehicle_attitude", self._on_attitude, qos
            )
            self.create_subscription(
                VehicleLocalPosition,
                f"{px4}/out/vehicle_local_position",
                self._on_local_position,
                qos,
            )
            self.create_subscription(
                VehicleStatus, f"{px4}/out/vehicle_status", self._on_status, qos
            )
            self.create_subscription(
                VehicleLandDetected,
                f"{px4}/out/vehicle_land_detected",
                self._on_land_detected,
                qos,
            )
            self.create_subscription(Bool, "/intercept/reset", self._on_reset, 10)
            self._shadow_pub = self.create_publisher(
                String, "/intercept/upstream_px4_shadow", 10
            )
            self._mode_pub = None
            self._trajectory_pub = None
            self._attitude_pub = None
            if self._mode == "live":
                self._mode_pub = self.create_publisher(
                    OffboardControlMode, f"{px4}/in/offboard_control_mode", qos
                )
                self._trajectory_pub = self.create_publisher(
                    TrajectorySetpoint, f"{px4}/in/trajectory_setpoint", qos
                )
                self._attitude_pub = self.create_publisher(
                    VehicleAttitudeSetpoint, f"{px4}/in/vehicle_attitude_setpoint", qos
                )
            self.create_timer(1.0 / self._rate_hz, self._on_timer)
            self.get_logger().warning(
                "upstream adapter ready mode=%s; VehicleCommand is permanently blocked"
                % self._mode
            )

        def _now_s(self) -> float:
            return self.get_clock().now().nanoseconds / 1.0e9

        def _on_control_mode(self, message) -> None:
            self._control_mode = message
            self._control_mode_received_s = self._now_s()

        def _on_trajectory(self, message) -> None:
            self._trajectory = message
            self._trajectory_received_s = self._now_s()

        def _on_attitude_setpoint(self, message) -> None:
            self._attitude_setpoint = message
            self._attitude_setpoint_received_s = self._now_s()

        def _on_attitude(self, _message) -> None:
            self._attitude_received_s = self._now_s()

        def _on_local_position(self, message) -> None:
            self._local_position = message
            self._local_position_received_s = self._now_s()

        def _flags(self) -> tuple[bool, bool]:
            if self._status is None:
                return False, False
            return (
                int(self._status.arming_state) == int(VehicleStatus.ARMING_STATE_ARMED),
                int(self._status.nav_state) == int(VehicleStatus.NAVIGATION_STATE_OFFBOARD),
            )

        def _on_status(self, message) -> None:
            _, previous_offboard = self._flags()
            self._status = message
            self._status_received_s = self._now_s()
            armed, offboard = self._flags()
            if not self._land_detected_received:
                self._landed = not armed
            if offboard:
                self._offboard_seen = True
            elif previous_offboard and armed and not self._landed:
                self._takeover_latched = True

        def _on_land_detected(self, message) -> None:
            self._land_detected_received = True
            self._landed = bool(message.landed)

        def _on_reset(self, message) -> None:
            armed, _ = self._flags()
            if bool(message.data) and self._landed and not armed:
                self._takeover_latched = False
                self._offboard_seen = False

        def _selection(self):
            if self._control_mode is None:
                return None, None, None
            mode = self._control_mode
            kind = selected_setpoint_kind(
                position=bool(mode.position),
                velocity=bool(mode.velocity),
                acceleration=bool(mode.acceleration),
                attitude=bool(mode.attitude),
                body_rate=bool(mode.body_rate),
                thrust_and_torque=bool(getattr(mode, "thrust_and_torque", False)),
                direct_actuator=bool(getattr(mode, "direct_actuator", False)),
            )
            if kind == "attitude":
                return kind, self._attitude_setpoint, self._attitude_setpoint_received_s
            if kind == "trajectory":
                return kind, self._trajectory, self._trajectory_received_s
            return None, None, None

        def _on_timer(self) -> None:
            now_s = self._now_s()
            kind, setpoint, setpoint_received_s = self._selection()
            timestamps = (self._control_mode_received_s, setpoint_received_s)
            present = kind is not None and setpoint is not None and all(
                value is not None for value in timestamps
            )
            age_s = math.inf if not present else max(now_s - float(value) for value in timestamps)
            valid = False
            if present and kind == "attitude":
                valid = valid_attitude_payload(setpoint.q_d, setpoint.thrust_body)
            elif present and kind == "trajectory":
                valid = valid_trajectory_payload(setpoint, self._control_mode)
            attitude_fresh = self._attitude_received_s is not None and (
                0.0 <= now_s - self._attitude_received_s <= self._attitude_timeout_s
            )
            local_fresh = self._local_position_received_s is not None and (
                0.0 <= now_s - self._local_position_received_s <= self._status_timeout_s
            ) and bool(getattr(self._local_position, "v_xy_valid", False)) and bool(
                getattr(self._local_position, "v_z_valid", False)
            )
            status_fresh = self._status_received_s is not None and (
                0.0 <= now_s - self._status_received_s <= self._status_timeout_s
            )
            armed, offboard = self._flags()
            failsafe = bool(getattr(self._status, "failsafe", False)) if self._status else False
            gate = evaluate_adapter_gate(
                mode=self._mode,
                command_present=present,
                command_valid=valid,
                command_age_s=age_s,
                command_timeout_s=self._command_timeout_s,
                invalid_elapsed_s=self._command_timeout_s + 1.0,
                zero_hold_s=0.0,
                attitude_fresh=attitude_fresh,
                local_position_fresh=local_fresh,
                status_fresh=status_fresh,
                failsafe=failsafe,
                takeover_latched=self._takeover_latched,
                armed=armed,
                landed=self._landed,
                offboard=offboard,
            )
            shadow = String()
            shadow.data = json.dumps(
                {
                    "timestamp_s": now_s,
                    "mode": self._mode,
                    "gate_reason": gate.reason,
                    "setpoint_kind": kind,
                    "setpoint_age_s": None if not math.isfinite(age_s) else age_s,
                    "payload_valid": valid,
                    "would_forward": bool(gate.apply_command),
                    "would_prestream": bool(gate.publish_heartbeat and not offboard and valid),
                    "vehicle_command_forwarded": False,
                },
                separators=(",", ":"),
            )
            self._shadow_pub.publish(shadow)
            if gate.reason != self._last_reason:
                self.get_logger().warning("PX4_UPSTREAM_GATE %s" % gate.reason)
                self._last_reason = gate.reason
            if self._mode != "live":
                return
            heartbeat_fresh = self._control_mode_received_s is not None and (
                0.0 <= now_s - self._control_mode_received_s <= self._command_timeout_s
            )
            preheat = (
                not offboard
                and heartbeat_fresh
                and not failsafe
                and not self._takeover_latched
                and status_fresh
                and attitude_fresh
                and local_fresh
                and armed
                and not self._landed
            )
            if preheat:
                # Upstream withholds setpoints until PX4 enters Offboard, so the
                # entry preheat forwards the heartbeat alone (field fix 2026-08-13).
                self._mode_pub.publish(self._control_mode)
                return
            if not valid or not gate.publish_heartbeat:
                return
            # Before the pilot enters Offboard these messages are only PX4's
            # required pre-stream; once Offboard is active the payload remains
            # byte-field-equivalent to the upstream output.
            self._mode_pub.publish(self._control_mode)
            if kind == "attitude":
                self._attitude_pub.publish(setpoint)
            elif kind == "trajectory":
                self._trajectory_pub.publish(setpoint)

    rclpy.init(args=args)
    node = None
    try:
        node = Px4UpstreamAdapter()
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
