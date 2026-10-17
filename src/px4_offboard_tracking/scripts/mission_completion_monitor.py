#!/usr/bin/env python3
"""Exit after a latched mission-complete signal so launch can shut down cleanly."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class MissionCompletionMonitor(Node):
    def __init__(self) -> None:
        super().__init__("mission_completion_monitor")
        self._delay = float(self.declare_parameter("shutdown_delay_s", 2.0).value)
        self._timer = None
        self.create_subscription(Bool, "/tracking/mission/complete", self._on_complete, 10)

    def _on_complete(self, message: Bool) -> None:
        if not message.data or self._timer is not None:
            return
        self.get_logger().info(
            f"mission complete; launch shutdown requested after {self._delay:.1f} s"
        )
        self._timer = self.create_timer(max(0.01, self._delay), self._finish)

    def _finish(self) -> None:
        self.get_logger().info("mission-complete delay elapsed; exiting monitor")
        raise SystemExit(0)


def main() -> None:
    rclpy.init()
    node = MissionCompletionMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
