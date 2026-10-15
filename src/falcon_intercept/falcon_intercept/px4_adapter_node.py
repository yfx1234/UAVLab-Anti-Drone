from __future__ import annotations

import json
import math
from dataclasses import dataclass


VALID_MODES = {"dry-run", "shadow", "live"}


def normalize_mode(value: str) -> str:
    mode = str(value).strip().lower().replace("_", "-")
    if mode not in VALID_MODES:
        raise ValueError(f"mode must be one of {sorted(VALID_MODES)}, got {value!r}")
    return mode


def quaternion_rotate_body_frd_to_ned(
    quaternion_wxyz: tuple[float, float, float, float],
    vector_body_frd: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Rotate a body-FRD vector into local NED using PX4 VehicleAttitude.q."""

    w, x, y, z = (float(value) for value in quaternion_wxyz)
    vx, vy, vz = (float(value) for value in vector_body_frd)
    values = (w, x, y, z, vx, vy, vz)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("quaternion and velocity must be finite")
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm < 1e-9:
        raise ValueError("attitude quaternion is zero")
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    # R(q_body_to_ned) * v_body.
    return (
        (1.0 - 2.0 * (y * y + z * z)) * vx
        + 2.0 * (x * y - z * w) * vy
        + 2.0 * (x * z + y * w) * vz,
        2.0 * (x * y + z * w) * vx
        + (1.0 - 2.0 * (x * x + z * z)) * vy
        + 2.0 * (y * z - x * w) * vz,
        2.0 * (x * z - y * w) * vx
        + 2.0 * (y * z + x * w) * vy
        + (1.0 - 2.0 * (x * x + y * y)) * vz,
    )


def flu_to_frd(vector_body_flu: tuple[float, float, float]) -> tuple[float, float, float]:
    """Convert ROS body FLU (+forward,+left,+up) to PX4 body FRD."""
    x, y, z = (float(value) for value in vector_body_flu)
    if not all(math.isfinite(value) for value in (x, y, z)):
        raise ValueError("body FLU velocity must be finite")
    return x, -y, -z


def fallback_landed_from_vehicle_status(*, armed: bool, takeoff_time_us: int) -> bool:
    """Conservative fallback when VehicleLandDetected is not in DDS outputs.

    PX4 resets ``takeoff_time`` on disarm and sets it on the first detected
    takeoff. Treat landing as complete only after disarm when the dedicated
    land-detector topic is unavailable.
    """

    return (not bool(armed)) or int(takeoff_time_us) <= 0


@dataclass(frozen=True)
class AdapterGate:
    publish_heartbeat: bool
    apply_command: bool
    reason: str


def evaluate_adapter_gate(
    *,
    mode: str,
    command_present: bool,
    command_valid: bool,
    command_age_s: float,
    command_timeout_s: float,
    invalid_elapsed_s: float,
    zero_hold_s: float,
    attitude_fresh: bool,
    local_position_fresh: bool,
    status_fresh: bool,
    failsafe: bool,
    takeover_latched: bool,
    armed: bool,
    landed: bool,
    offboard: bool,
) -> AdapterGate:
    mode = normalize_mode(mode)
    if mode != "live":
        return AdapterGate(False, False, f"{mode} never writes PX4 input topics")
    if failsafe:
        return AdapterGate(False, False, "PX4 failsafe active")
    if takeover_latched:
        return AdapterGate(False, False, "human takeover latched")
    if not status_fresh:
        return AdapterGate(False, False, "PX4 vehicle status unavailable or stale")
    if not attitude_fresh:
        return AdapterGate(False, False, "PX4 attitude unavailable or stale")
    if not local_position_fresh:
        return AdapterGate(False, False, "PX4 local velocity unavailable, invalid, or stale")
    # A live adapter must never make a ground vehicle eligible for Offboard or
    # autonomously initiate takeoff. The operator must arm and take off first.
    if not armed or landed:
        return AdapterGate(False, False, "manual arm and takeoff required")
    if not command_present:
        return AdapterGate(False, False, "waiting for first guidance command")
    if command_age_s < 0.0 or command_age_s > command_timeout_s:
        hold = invalid_elapsed_s <= zero_hold_s
        return AdapterGate(hold, False, "guidance command stale")
    if not offboard:
        # Manual Offboard entry requires a continuous >2 Hz heartbeat before
        # the operator flips the mode switch.  The preheat is always zero and
        # cannot arm, take off, or apply a mission command.
        return AdapterGate(True, False, "zero-setpoint Offboard preheat")
    if not command_valid:
        hold = invalid_elapsed_s <= zero_hold_s
        return AdapterGate(hold, False, "guidance command invalid")
    return AdapterGate(True, True, "live velocity command allowed")


def main(args=None) -> int:
    import rclpy
    from falcon_intercept_msgs.msg import GuidanceCommand, VehicleInterfaceStatus
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, String

    try:
        from px4_msgs.msg import (
            OffboardControlMode,
            TrajectorySetpoint,
            VehicleAttitude,
            VehicleCommand,
            VehicleCommandAck,
            VehicleLandDetected,
            VehicleLocalPosition,
            VehicleStatus,
        )

        px4_available = True
    except ImportError:
        OffboardControlMode = TrajectorySetpoint = VehicleAttitude = None
        VehicleCommand = VehicleCommandAck = VehicleLandDetected = None
        VehicleLocalPosition = VehicleStatus = None
        px4_available = False

    class Px4VelocityAdapter(Node):
        def __init__(self) -> None:
            super().__init__("px4_velocity_adapter")
            self.declare_parameter("guidance_command_topic", "/intercept/guidance_command")
            self.declare_parameter("mode", "shadow")
            self.declare_parameter("rate_hz", 20.0)
            self.declare_parameter("command_timeout_s", 0.25)
            self.declare_parameter("zero_hold_s", 0.5)
            self.declare_parameter("attitude_timeout_s", 0.25)
            # Commander publishes VehicleStatus at 2 Hz. Allow three nominal
            # periods so normal scheduler/DDS jitter cannot flap the live gate.
            self.declare_parameter("status_timeout_s", 1.5)
            self.declare_parameter("land_retry_interval_s", 1.0)
            self.declare_parameter("allow_live_control", False)
            self.declare_parameter("calibration_ready", False)
            self.declare_parameter("expected_frame_id", "pursuer_base_flu")
            self.declare_parameter("px4_namespace", "/fmu")
            self.declare_parameter("target_system", 1)
            self.declare_parameter("target_component", 1)
            self.declare_parameter("source_system", 1)
            self.declare_parameter("source_component", 1)

            self._mode = normalize_mode(str(self.get_parameter("mode").value))
            self._rate_hz = max(20.0, float(self.get_parameter("rate_hz").value))
            self._command_timeout_s = max(
                0.01, float(self.get_parameter("command_timeout_s").value)
            )
            self._zero_hold_s = max(0.0, float(self.get_parameter("zero_hold_s").value))
            self._attitude_timeout_s = max(
                0.01, float(self.get_parameter("attitude_timeout_s").value)
            )
            self._status_timeout_s = max(
                0.01, float(self.get_parameter("status_timeout_s").value)
            )
            self._land_retry_interval_s = max(
                0.2, float(self.get_parameter("land_retry_interval_s").value)
            )
            self._allow_live = bool(self.get_parameter("allow_live_control").value)
            self._calibration_ready = bool(self.get_parameter("calibration_ready").value)
            self._expected_frame = str(self.get_parameter("expected_frame_id").value)
            namespace = str(self.get_parameter("px4_namespace").value).rstrip("/")
            self._target_system = int(self.get_parameter("target_system").value)
            self._target_component = int(self.get_parameter("target_component").value)
            self._source_system = int(self.get_parameter("source_system").value)
            self._source_component = int(self.get_parameter("source_component").value)

            if self._mode == "live" and not px4_available:
                raise RuntimeError("live mode requires the px4_msgs package")
            if self._mode == "live" and (not self._allow_live or not self._calibration_ready):
                raise RuntimeError(
                    "live mode refused: allow_live_control and calibration_ready must both be true"
                )

            self._command = None
            self._command_received_s: float | None = None
            self._last_sequence: int | None = None
            self._invalid_since_s: float | None = None
            self._attitude_q: tuple[float, float, float, float] | None = None
            self._attitude_received_s: float | None = None
            self._status = None
            self._status_received_s: float | None = None
            self._local_position = None
            self._local_position_received_s: float | None = None
            self._landed = True
            self._land_detected_received = False
            self._takeover_latched = False
            self._offboard_seen = False
            self._status_sequence = 0
            self._land_sent = False
            self._last_land_sent_s: float | None = None
            self._land_ack_received = False
            self._land_ack_accepted = False
            self._land_ack_result = 255
            self._last_gate_reason = ""

            sensor_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(
                GuidanceCommand,
                str(self.get_parameter("guidance_command_topic").value),
                self._on_command,
                1,
            )
            self.create_subscription(Bool, "/intercept/reset", self._on_reset, 10)
            self._shadow_pub = self.create_publisher(String, "/intercept/px4_shadow", 10)
            self._landed_pub = self.create_publisher(Bool, "/vehicle/landed", 10)
            self._interface_status_pub = self.create_publisher(
                VehicleInterfaceStatus, "/control/px4_status", 10
            )

            self._offboard_pub = None
            self._setpoint_pub = None
            self._vehicle_command_pub = None
            if px4_available:
                self.create_subscription(
                    VehicleAttitude,
                    f"{namespace}/out/vehicle_attitude",
                    self._on_attitude,
                    sensor_qos,
                )
                self.create_subscription(
                    VehicleLocalPosition,
                    f"{namespace}/out/vehicle_local_position",
                    self._on_local_position,
                    sensor_qos,
                )
                self.create_subscription(
                    VehicleStatus,
                    f"{namespace}/out/vehicle_status",
                    self._on_status,
                    sensor_qos,
                )
                self.create_subscription(
                    VehicleLandDetected,
                    f"{namespace}/out/vehicle_land_detected",
                    self._on_land_detected,
                    sensor_qos,
                )
                self.create_subscription(
                    VehicleCommandAck,
                    f"{namespace}/out/vehicle_command_ack",
                    self._on_command_ack,
                    sensor_qos,
                )
                if self._mode == "live":
                    self._offboard_pub = self.create_publisher(
                        OffboardControlMode, f"{namespace}/in/offboard_control_mode", sensor_qos
                    )
                    self._setpoint_pub = self.create_publisher(
                        TrajectorySetpoint, f"{namespace}/in/trajectory_setpoint", sensor_qos
                    )
                    self._vehicle_command_pub = self.create_publisher(
                        VehicleCommand, f"{namespace}/in/vehicle_command", sensor_qos
                    )

            self.create_timer(1.0 / self._rate_hz, self._on_timer)
            self.get_logger().warning(
                "PX4 velocity adapter ready mode=%s px4_msgs=%s allow_live=%s calibration_ready=%s"
                % (
                    self._mode,
                    str(px4_available).lower(),
                    str(self._allow_live).lower(),
                    str(self._calibration_ready).lower(),
                )
            )

        def _now_s(self) -> float:
            return self.get_clock().now().nanoseconds / 1_000_000_000.0

        def _timestamp_us(self) -> int:
            return self.get_clock().now().nanoseconds // 1000

        def _on_command(self, message: GuidanceCommand) -> None:
            sequence = int(message.sequence)
            if self._last_sequence is not None and sequence <= self._last_sequence:
                self.get_logger().warning(
                    f"rejected non-increasing guidance sequence {sequence} <= {self._last_sequence}"
                )
                return
            self._last_sequence = sequence
            self._command = message
            self._command_received_s = self._now_s()
            frame_ok = str(message.header.frame_id) == self._expected_frame
            if bool(message.valid) and frame_ok:
                self._invalid_since_s = None
            elif self._invalid_since_s is None:
                self._invalid_since_s = self._command_received_s
            if not bool(message.land_request):
                self._land_sent = False
                self._last_land_sent_s = None
                self._land_ack_received = False
                self._land_ack_accepted = False
                self._land_ack_result = 255

        def _on_attitude(self, message) -> None:
            if len(message.q) >= 4:
                quaternion = tuple(float(value) for value in message.q[:4])
                norm_squared = sum(value * value for value in quaternion)
                if all(math.isfinite(value) for value in quaternion) and norm_squared > 1e-12:
                    self._attitude_q = quaternion
                    self._attitude_received_s = self._now_s()

        def _on_reset(self, message: Bool) -> None:
            armed, _ = self._status_flags()
            if bool(message.data) and self._landed and not armed:
                self._takeover_latched = False
                self._offboard_seen = False
                self.get_logger().warning(
                    "adapter takeover latch reset while landed and disarmed"
                )

        def _on_local_position(self, message) -> None:
            self._local_position = message
            self._local_position_received_s = self._now_s()

        def _on_status(self, message) -> None:
            previous_offboard = self._status_flags()[1]
            self._status = message
            self._status_received_s = self._now_s()
            armed, offboard = self._status_flags()
            if not self._land_detected_received:
                self._landed = fallback_landed_from_vehicle_status(
                    armed=armed,
                    takeoff_time_us=int(getattr(message, "takeoff_time", 0)),
                )
                landed = Bool()
                landed.data = self._landed
                self._landed_pub.publish(landed)
            if offboard:
                self._offboard_seen = True
            elif previous_offboard and armed and not self._landed and not self._land_sent:
                self._takeover_latched = True

        def _on_land_detected(self, message) -> None:
            self._land_detected_received = True
            landed = Bool()
            landed.data = bool(getattr(message, "landed", False))
            self._landed = landed.data
            self._landed_pub.publish(landed)

        def _on_command_ack(self, message) -> None:
            if VehicleCommand is None:
                return
            land_command = int(getattr(VehicleCommand, "VEHICLE_CMD_NAV_LAND", 21))
            if int(getattr(message, "command", -1)) == land_command:
                result = int(getattr(message, "result", 255))
                accepted = int(getattr(VehicleCommandAck, "VEHICLE_CMD_RESULT_ACCEPTED", 0))
                in_progress = int(getattr(VehicleCommandAck, "VEHICLE_CMD_RESULT_IN_PROGRESS", 5))
                self._land_ack_received = True
                self._land_ack_result = result
                self._land_ack_accepted = result in (accepted, in_progress)
                if not self._land_ack_accepted:
                    # Keep the bounded retry timer, but allow another native request.
                    self._land_sent = False
                self.get_logger().warning(
                    "PX4 LAND command ack result=%s accepted=%s"
                    % (result, str(self._land_ack_accepted).lower())
                )

        def _status_flags(self) -> tuple[bool, bool]:
            if self._status is None or VehicleStatus is None:
                return False, False
            armed_value = int(getattr(VehicleStatus, "ARMING_STATE_ARMED", 2))
            offboard_value = int(getattr(VehicleStatus, "NAVIGATION_STATE_OFFBOARD", 14))
            return (
                int(getattr(self._status, "arming_state", -1)) == armed_value,
                int(getattr(self._status, "nav_state", -1)) == offboard_value,
            )

        def _publish_shadow(
            self,
            *,
            now_s: float,
            gate: AdapterGate,
            velocity_ned: tuple[float, float, float] | None,
        ) -> None:
            command = self._command
            def finite_or_none(value):
                value = float(value)
                return value if math.isfinite(value) else None

            message = String()
            message.data = json.dumps(
                {
                    "timestamp_s": now_s,
                    "mode": self._mode,
                    "gate_reason": gate.reason,
                    "would_publish_heartbeat": gate.publish_heartbeat,
                    "would_apply_command": gate.apply_command,
                    "sequence": None if command is None else int(command.sequence),
                    "mission_state": None if command is None else int(command.mission_state),
                    "velocity_body_flu": None
                    if command is None
                    else [
                        finite_or_none(command.velocity_body_flu.x),
                        finite_or_none(command.velocity_body_flu.y),
                        finite_or_none(command.velocity_body_flu.z),
                    ],
                    "velocity_ned": velocity_ned,
                    "yaw_rate_flu": None if command is None else finite_or_none(command.yaw_rate_flu),
                    "land_request": False if command is None else bool(command.land_request),
                },
                separators=(",", ":"),
                allow_nan=False,
            )
            self._shadow_pub.publish(message)

        def _publish_offboard(self) -> None:
            message = OffboardControlMode()
            message.timestamp = self._timestamp_us()
            for field, value in (
                ("position", False),
                ("velocity", True),
                ("acceleration", False),
                ("attitude", False),
                ("body_rate", False),
                ("thrust_and_torque", False),
                ("direct_actuator", False),
            ):
                if hasattr(message, field):
                    setattr(message, field, value)
            self._offboard_pub.publish(message)

        def _publish_setpoint(
            self, velocity_ned: tuple[float, float, float], yaw_rate: float
        ) -> None:
            message = TrajectorySetpoint()
            message.timestamp = self._timestamp_us()
            nan = float("nan")
            if hasattr(message, "position"):
                message.position = [nan, nan, nan]
            message.velocity = [float(value) for value in velocity_ned]
            if hasattr(message, "acceleration"):
                message.acceleration = [nan, nan, nan]
            if hasattr(message, "jerk"):
                message.jerk = [nan, nan, nan]
            if hasattr(message, "yaw"):
                message.yaw = nan
            message.yawspeed = float(yaw_rate)
            self._setpoint_pub.publish(message)

        def _send_land_once(self) -> None:
            now_s = self._now_s()
            if self._vehicle_command_pub is None or self._land_ack_accepted:
                return
            if self._last_land_sent_s is not None and now_s - self._last_land_sent_s < self._land_retry_interval_s:
                return
            message = VehicleCommand()
            message.timestamp = self._timestamp_us()
            message.command = int(getattr(VehicleCommand, "VEHICLE_CMD_NAV_LAND", 21))
            message.target_system = self._target_system
            message.target_component = self._target_component
            message.source_system = self._source_system
            message.source_component = self._source_component
            message.from_external = True
            for index in range(1, 8):
                field = f"param{index}"
                if hasattr(message, field):
                    setattr(message, field, float("nan"))
            self._vehicle_command_pub.publish(message)
            self._land_sent = True
            self._last_land_sent_s = now_s
            self.get_logger().warning("PX4 native LAND requested once")

        def _on_timer(self) -> None:
            now_s = self._now_s()
            command_present = self._command is not None and self._command_received_s is not None
            command_age_s = (
                math.inf if not command_present else now_s - float(self._command_received_s)
            )
            command_ttl_s = self._command_timeout_s
            if command_present and float(self._command.valid_for_s) > 0.0:
                command_ttl_s = min(command_ttl_s, float(self._command.valid_for_s))
            command_valid = bool(command_present and self._command.valid)
            command_finite = bool(
                command_present
                and all(
                    math.isfinite(value)
                    for value in (
                        float(self._command.velocity_body_flu.x),
                        float(self._command.velocity_body_flu.y),
                        float(self._command.velocity_body_flu.z),
                        float(self._command.yaw_rate_flu),
                    )
                )
            )
            command_valid = command_valid and command_finite
            if command_present and str(self._command.header.frame_id) != self._expected_frame:
                command_valid = False
            if (not command_valid or command_age_s > command_ttl_s) and self._invalid_since_s is None:
                self._invalid_since_s = now_s
            invalid_elapsed_s = (
                0.0 if self._invalid_since_s is None else now_s - self._invalid_since_s
            )
            attitude_fresh = (
                self._attitude_q is not None
                and self._attitude_received_s is not None
                and 0.0 <= now_s - self._attitude_received_s <= self._attitude_timeout_s
            )
            status_fresh = (
                self._status is not None
                and self._status_received_s is not None
                and 0.0 <= now_s - self._status_received_s <= self._status_timeout_s
            )
            local_position_fresh = (
                self._local_position is not None
                and self._local_position_received_s is not None
                and 0.0 <= now_s - self._local_position_received_s <= self._status_timeout_s
                and bool(getattr(self._local_position, "v_xy_valid", False))
                and bool(getattr(self._local_position, "v_z_valid", False))
            )
            armed, offboard = self._status_flags()
            failsafe = bool(getattr(self._status, "failsafe", False)) if self._status else False
            gate = evaluate_adapter_gate(
                mode=self._mode,
                command_present=command_present,
                command_valid=command_valid,
                command_age_s=command_age_s,
                command_timeout_s=command_ttl_s,
                invalid_elapsed_s=invalid_elapsed_s,
                zero_hold_s=self._zero_hold_s,
                attitude_fresh=attitude_fresh,
                local_position_fresh=local_position_fresh,
                status_fresh=status_fresh,
                failsafe=failsafe,
                takeover_latched=self._takeover_latched,
                armed=armed,
                landed=self._landed,
                offboard=offboard,
            )

            # Manual mode switch remains the only way into Offboard. While airborne and
            # armed, continuously pre-stream a zero setpoint so PX4 can accept that switch.
            prestreaming = bool(
                self._mode == "live" and armed and not self._landed
                and attitude_fresh and local_position_fresh and status_fresh
                and not failsafe and not self._takeover_latched
                and not self._land_sent and not self._land_ack_accepted
            )
            if prestreaming and not offboard:
                gate = AdapterGate(True, False, "zero-setpoint Offboard prestream")
            if self._takeover_latched:
                gate = AdapterGate(False, False, "human takeover latched")
            elif (self._land_sent or self._land_ack_accepted) and not offboard:
                gate = AdapterGate(False, False, "PX4 native LAND active")

            velocity_ned = None
            if command_present and attitude_fresh:
                try:
                    velocity_ned = quaternion_rotate_body_frd_to_ned(
                        self._attitude_q,
                        flu_to_frd((
                            float(self._command.velocity_body_flu.x),
                            float(self._command.velocity_body_flu.y),
                            float(self._command.velocity_body_flu.z),
                        )),
                    )
                except ValueError as error:
                    gate = AdapterGate(gate.publish_heartbeat, False, str(error))

            self._publish_shadow(now_s=now_s, gate=gate, velocity_ned=velocity_ned)
            interface = VehicleInterfaceStatus()
            interface.header.stamp = self.get_clock().now().to_msg()
            interface.header.frame_id = "px4_local_ned"
            interface.sequence = self._status_sequence
            self._status_sequence += 1
            interface.px4_msgs_available = px4_available
            interface.attitude_fresh = attitude_fresh
            interface.local_position_fresh = local_position_fresh
            interface.vehicle_status_fresh = status_fresh
            interface.armed = armed
            interface.landed = self._landed
            interface.offboard = offboard
            interface.failsafe = failsafe
            interface.takeover_latched = self._takeover_latched
            interface.prestreaming = prestreaming
            interface.ready_for_mission = bool(
                armed and not self._landed and offboard and attitude_fresh
                and local_position_fresh and status_fresh and not interface.failsafe
                and not self._takeover_latched
            )
            interface.land_command_sent = self._land_sent
            interface.land_ack_received = self._land_ack_received
            interface.land_ack_accepted = self._land_ack_accepted
            interface.land_ack_result = self._land_ack_result
            if self._local_position is not None:
                interface.position_ned.x = float(getattr(self._local_position, "x", 0.0))
                interface.position_ned.y = float(getattr(self._local_position, "y", 0.0))
                interface.position_ned.z = float(getattr(self._local_position, "z", 0.0))
                interface.velocity_ned.x = float(getattr(self._local_position, "vx", 0.0))
                interface.velocity_ned.y = float(getattr(self._local_position, "vy", 0.0))
                interface.velocity_ned.z = float(getattr(self._local_position, "vz", 0.0))
            if self._attitude_q is not None:
                w, x, y, z = self._attitude_q
                interface.yaw_ned = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            interface.nav_state = int(getattr(self._status, "nav_state", 0)) if self._status else 0
            interface.reason = gate.reason
            self._interface_status_pub.publish(interface)
            if gate.reason != self._last_gate_reason:
                self.get_logger().warning(f"PX4_GATE {gate.reason}")
                self._last_gate_reason = gate.reason

            if self._mode != "live" or not gate.publish_heartbeat:
                return
            self._publish_offboard()
            if gate.apply_command and velocity_ned is not None:
                self._publish_setpoint(velocity_ned, -float(self._command.yaw_rate_flu))
                if bool(self._command.land_request) and armed and offboard:
                    self._send_land_once()
            else:
                self._publish_setpoint((0.0, 0.0, 0.0), 0.0)

    rclpy.init(args=args)
    node = None
    try:
        node = Px4VelocityAdapter()
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
