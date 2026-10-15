"""Read-only, transition-oriented field monitor for the real closed loop."""

from __future__ import annotations

import json


def main(args=None) -> int:
    import rclpy
    from px4_offboard_tracking.msg import PureInterceptorStatus
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from std_msgs.msg import String
    from virtual_interception_interfaces.msg import VirtualShotEvent

    class FieldMonitor(Node):
        def __init__(self) -> None:
            super().__init__("field_monitor_node")
            self._state = None
            self._safe_to_exit = False
            self._gate_reason = None
            self.create_subscription(
                PureInterceptorStatus,
                "/tracking/pure_interceptor/status",
                self._on_status,
                10,
            )
            self.create_subscription(
                VirtualShotEvent,
                "/tracking/virtual_shot_event",
                self._on_shot,
                10,
            )
            self.create_subscription(
                String,
                "/intercept/upstream_px4_shadow",
                self._on_gate,
                10,
            )
            self.get_logger().warning(
                "FIELD MONITOR READY; any anomaly: pilot exits Offboard first"
            )

        def _on_status(self, message) -> None:
            state = str(message.state_name)
            if state != self._state:
                self.get_logger().warning(
                    "MISSION %s capture=%d/%d"
                    % (
                        state,
                        int(message.capture_count),
                        int(message.required_capture_count),
                    )
                )
                self._state = state
            safe = bool(message.safe_to_exit_offboard)
            if safe and not self._safe_to_exit:
                self.get_logger().warning(
                    "SAFE_TO_EXIT_OFFBOARD=true: pilot may exit after visual hover confirmation"
                )
            self._safe_to_exit = safe

        def _on_shot(self, message) -> None:
            self.get_logger().warning("VIRTUAL_SHOT shot_id=%d" % int(message.shot_id))

        def _on_gate(self, message) -> None:
            try:
                item = json.loads(message.data)
            except (TypeError, ValueError, json.JSONDecodeError):
                self.get_logger().error("invalid adapter status JSON")
                return
            reason = str(item.get("gate_reason", "unknown"))
            if reason != self._gate_reason:
                self.get_logger().warning(
                    "GATE %s kind=%s payload_valid=%s would_forward=%s vehicle_command=%s"
                    % (
                        reason,
                        item.get("setpoint_kind"),
                        item.get("payload_valid"),
                        item.get("would_forward"),
                        item.get("vehicle_command_forwarded"),
                    )
                )
                self._gate_reason = reason

    rclpy.init(args=args)
    node = None
    try:
        node = FieldMonitor()
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
