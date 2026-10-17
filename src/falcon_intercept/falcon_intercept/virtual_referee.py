from __future__ import annotations

import math
from dataclasses import dataclass

from .virtual_target import rotate_body_frd_to_ned


@dataclass(frozen=True)
class RefereeResult:
    hit: bool
    miss_distance_m: float
    closest_time_s: float


def evaluate_virtual_shot(
    *,
    shooter_position_ned: tuple[float, float, float],
    shooter_q_body_to_ned: tuple[float, float, float, float],
    target_position_ned: tuple[float, float, float],
    target_velocity_ned: tuple[float, float, float],
    projectile_speed_mps: float = 400.0,
    gravity_mps2: float = 9.80665,
    maximum_time_s: float = 0.25,
    hit_radius_m: float = 1.0,
) -> RefereeResult:
    direction = rotate_body_frd_to_ned(
        shooter_q_body_to_ned, (1.0, 0.0, 0.0)
    )
    samples = 500
    best_distance = math.inf
    best_time = 0.0
    for index in range(samples + 1):
        time_s = float(maximum_time_s) * index / samples
        projectile = (
            shooter_position_ned[0] + projectile_speed_mps * direction[0] * time_s,
            shooter_position_ned[1] + projectile_speed_mps * direction[1] * time_s,
            shooter_position_ned[2]
            + projectile_speed_mps * direction[2] * time_s
            + 0.5 * gravity_mps2 * time_s * time_s,
        )
        target = tuple(
            target_position_ned[axis] + target_velocity_ned[axis] * time_s
            for axis in range(3)
        )
        distance = math.sqrt(
            sum((projectile[axis] - target[axis]) ** 2 for axis in range(3))
        )
        if distance < best_distance:
            best_distance = distance
            best_time = time_s
    return RefereeResult(
        hit=best_distance <= float(hit_radius_m),
        miss_distance_m=best_distance,
        closest_time_s=best_time,
    )


def main(args=None) -> int:
    import json

    import rclpy
    from nav_msgs.msg import Odometry
    from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String

    class VirtualShotReferee(Node):
        def __init__(self) -> None:
            super().__init__("virtual_shot_referee")
            self.declare_parameter("projectile_speed_mps", 400.0)
            self.declare_parameter("gravity_mps2", 9.80665)
            self.declare_parameter("maximum_time_s", 0.25)
            self.declare_parameter("hit_radius_m", 1.0)
            self.declare_parameter("truth_timeout_s", 0.20)
            self._speed = float(self.get_parameter("projectile_speed_mps").value)
            self._gravity = float(self.get_parameter("gravity_mps2").value)
            self._maximum_time = float(self.get_parameter("maximum_time_s").value)
            self._hit_radius = float(self.get_parameter("hit_radius_m").value)
            self._timeout = float(self.get_parameter("truth_timeout_s").value)
            self._position = self._attitude = self._target_position = None
            self._target_velocity = None
            self._position_time = self._attitude_time = self._truth_time = -math.inf
            self.create_subscription(
                VehicleLocalPosition,
                "/fmu/out/vehicle_local_position",
                self._on_position,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                VehicleAttitude,
                "/fmu/out/vehicle_attitude",
                self._on_attitude,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Odometry,
                "/virtual_target/truth",
                self._on_truth,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                String, "/intercept/fire_event", self._on_fire, 10
            )
            self._publisher = self.create_publisher(
                String, "/virtual_target/hit_result", 10
            )

        def _now(self) -> float:
            return self.get_clock().now().nanoseconds / 1_000_000_000.0

        def _on_position(self, message) -> None:
            if bool(message.xy_valid) and bool(message.z_valid):
                value = (float(message.x), float(message.y), float(message.z))
                if all(math.isfinite(item) for item in value):
                    self._position = value
                    self._position_time = self._now()

        def _on_attitude(self, message) -> None:
            if len(message.q) >= 4:
                value = tuple(float(item) for item in message.q[:4])
                if all(math.isfinite(item) for item in value):
                    self._attitude = value
                    self._attitude_time = self._now()

        def _on_truth(self, message) -> None:
            self._target_position = (
                float(message.pose.pose.position.x),
                float(message.pose.pose.position.y),
                float(message.pose.pose.position.z),
            )
            self._target_velocity = (
                float(message.twist.twist.linear.x),
                float(message.twist.twist.linear.y),
                float(message.twist.twist.linear.z),
            )
            self._truth_time = self._now()

        def _on_fire(self, message) -> None:
            now = self._now()
            try:
                event = json.loads(str(message.data))
            except (TypeError, ValueError):
                event = {"event": str(message.data), "shot_id": 0}
            fresh = (
                self._position is not None
                and self._attitude is not None
                and self._target_position is not None
                and self._target_velocity is not None
                and now - self._position_time <= self._timeout
                and now - self._attitude_time <= self._timeout
                and now - self._truth_time <= self._timeout
            )
            output = {
                "shot_id": int(event.get("shot_id", 0)),
                "valid": fresh,
                "truth_isolated": True,
            }
            if fresh:
                result = evaluate_virtual_shot(
                    shooter_position_ned=self._position,
                    shooter_q_body_to_ned=self._attitude,
                    target_position_ned=self._target_position,
                    target_velocity_ned=self._target_velocity,
                    projectile_speed_mps=self._speed,
                    gravity_mps2=self._gravity,
                    maximum_time_s=self._maximum_time,
                    hit_radius_m=self._hit_radius,
                )
                output.update(
                    {
                        "hit": result.hit,
                        "miss_distance_m": result.miss_distance_m,
                        "closest_time_s": result.closest_time_s,
                    }
                )
            else:
                output.update({"hit": False, "reason": "truth or PX4 state stale"})
            result_message = String()
            result_message.data = json.dumps(output, separators=(",", ":"))
            self._publisher.publish(result_message)

    rclpy.init(args=args)
    node = None
    try:
        node = VirtualShotReferee()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
