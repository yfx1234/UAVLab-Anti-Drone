#!/usr/bin/env python3
"""Live PX4 pursuer attitude and smoothness visualization."""

from __future__ import annotations

import argparse
import math
import threading
from collections import deque
from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from px4_msgs.msg import VehicleAngularVelocity, VehicleAttitude
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def quaternion_to_euler(q: list[float] | tuple[float, ...]) -> np.ndarray:
    """Convert PX4 [w, x, y, z] body-to-NED quaternion to roll/pitch/yaw."""
    w, x, y, z = (float(value) for value in q)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("invalid attitude quaternion")
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.array([roll, pitch, yaw], dtype=float)


def quaternion_rotation_matrix(q: list[float] | tuple[float, ...]) -> np.ndarray:
    w, x, y, z = (float(value) for value in q)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("invalid attitude quaternion")
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


@dataclass
class Snapshot:
    time: np.ndarray
    euler: np.ndarray
    rates: np.ndarray
    angular_acceleration: np.ndarray
    quaternion: np.ndarray
    message_age: float


class AttitudeBuffer:
    def __init__(self, window_seconds: float) -> None:
        self.window_seconds = window_seconds
        self._time: deque[float] = deque()
        self._euler: deque[np.ndarray] = deque()
        self._rates: deque[np.ndarray] = deque()
        self._acceleration: deque[float] = deque()
        self._quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        self._last_rate_time: float | None = None
        self._last_rate: np.ndarray | None = None
        self._last_message_time = 0.0
        self._lock = threading.Lock()

    def add_attitude(self, now: float, quaternion: list[float]) -> None:
        try:
            euler = quaternion_to_euler(quaternion)
        except ValueError:
            return
        with self._lock:
            if self._euler:
                previous = self._euler[-1]
                euler[2] = previous[2] + math.atan2(
                    math.sin(euler[2] - previous[2]), math.cos(euler[2] - previous[2])
                )
            self._time.append(now)
            self._euler.append(euler)
            self._rates.append(self._last_rate.copy() if self._last_rate is not None else np.zeros(3))
            self._acceleration.append(0.0)
            self._quaternion = np.asarray(quaternion, dtype=float)
            self._last_message_time = now
            self._trim(now)

    def add_rates(self, now: float, rates: list[float]) -> None:
        rate = np.asarray(rates, dtype=float)
        if rate.shape != (3,) or not np.all(np.isfinite(rate)):
            return
        with self._lock:
            acceleration = 0.0
            if self._last_rate is not None and self._last_rate_time is not None:
                delta_time = now - self._last_rate_time
                if 1e-4 < delta_time < 1.0:
                    acceleration = float(np.linalg.norm(rate - self._last_rate) / delta_time)
            self._last_rate = rate
            self._last_rate_time = now
            if self._rates:
                self._rates[-1] = rate.copy()
                self._acceleration[-1] = acceleration
            self._last_message_time = now

    def snapshot(self, now: float) -> Snapshot:
        with self._lock:
            return Snapshot(
                np.asarray(self._time, dtype=float),
                np.asarray(self._euler, dtype=float),
                np.asarray(self._rates, dtype=float),
                np.asarray(self._acceleration, dtype=float),
                self._quaternion.copy(),
                max(0.0, now - self._last_message_time) if self._last_message_time else math.inf,
            )

    def _trim(self, now: float) -> None:
        while self._time and now - self._time[0] > self.window_seconds:
            self._time.popleft()
            self._euler.popleft()
            self._rates.popleft()
            self._acceleration.popleft()


class AttitudeMonitorNode(Node):
    def __init__(self, arguments: argparse.Namespace) -> None:
        super().__init__("realtime_attitude_monitor")
        self.buffer = AttitudeBuffer(arguments.window)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(VehicleAttitude, arguments.attitude_topic, self._attitude, qos)
        self.create_subscription(
            VehicleAngularVelocity, arguments.angular_velocity_topic, self._angular_velocity, qos
        )

    def _attitude(self, message: VehicleAttitude) -> None:
        self.buffer.add_attitude(self.get_clock().now().nanoseconds * 1e-9, list(message.q))

    def _angular_velocity(self, message: VehicleAngularVelocity) -> None:
        self.buffer.add_rates(
            self.get_clock().now().nanoseconds * 1e-9, list(message.xyz)
        )


class LivePlot:
    def __init__(self, node: AttitudeMonitorNode, arguments: argparse.Namespace) -> None:
        self.node = node
        self.arguments = arguments
        self.figure = plt.figure(figsize=(14, 8))
        grid = self.figure.add_gridspec(2, 2)
        self.angle_axes = self.figure.add_subplot(grid[0, 0])
        self.rate_axes = self.figure.add_subplot(grid[1, 0])
        self.smooth_axes = self.figure.add_subplot(grid[0, 1])
        self.body_axes = self.figure.add_subplot(grid[1, 1], projection="3d")
        self.figure.canvas.manager.set_window_title("22001 Pursuer Attitude Monitor")

    def update(self) -> None:
        snapshot = self.node.buffer.snapshot(
            self.node.get_clock().now().nanoseconds * 1e-9
        )
        for axes in (self.angle_axes, self.rate_axes, self.smooth_axes, self.body_axes):
            axes.clear()
        if snapshot.time.size == 0:
            self.angle_axes.text(0.5, 0.5, "Waiting for PX4 attitude...", ha="center")
            self.figure.canvas.draw_idle()
            return

        elapsed = snapshot.time - snapshot.time[-1]
        colors = ("tab:red", "tab:green", "tab:blue")
        labels = ("roll", "pitch", "yaw")
        for index, label in enumerate(labels):
            self.angle_axes.plot(elapsed, np.rad2deg(snapshot.euler[:, index]), label=label,
                                 color=colors[index])
            self.rate_axes.plot(elapsed, snapshot.rates[:, index], label=label,
                                color=colors[index])
        self.angle_axes.set_ylabel("attitude [deg]")
        self.rate_axes.set_ylabel("body rate [rad/s]")
        self.rate_axes.axhline(self.arguments.rate_warning, color="orange", linestyle="--")
        self.rate_axes.axhline(-self.arguments.rate_warning, color="orange", linestyle="--")
        self.rate_axes.set_xlabel("time from now [s]")
        self.angle_axes.legend(loc="upper left", ncol=3)
        self.rate_axes.legend(loc="upper left", ncol=3)

        self.smooth_axes.plot(elapsed, snapshot.angular_acceleration, color="tab:purple")
        self.smooth_axes.axhline(
            self.arguments.acceleration_warning, color="orange", linestyle="--",
            label="warning threshold"
        )
        self.smooth_axes.set_ylabel("|angular acceleration| [rad/s²]")
        self.smooth_axes.set_xlabel("time from now [s]")
        self.smooth_axes.legend(loc="upper left")
        recent = snapshot.angular_acceleration[elapsed >= -2.0]
        rms = math.sqrt(float(np.mean(recent * recent))) if recent.size else 0.0
        peak_rate = float(np.max(np.linalg.norm(snapshot.rates, axis=1)))
        risky = peak_rate > self.arguments.rate_warning or rms > self.arguments.acceleration_warning
        status = "WARNING" if risky else "SMOOTH"
        self.smooth_axes.set_title(
            f"{status} | 2 s angular-accel RMS={rms:.2f} rad/s² | peak rate={peak_rate:.2f} rad/s",
            color="tab:red" if risky else "tab:green",
        )

        rotation = quaternion_rotation_matrix(snapshot.quaternion)
        for index, (color, label) in enumerate(zip(colors, ("body X", "body Y", "body Z"))):
            vector = rotation[:, index]
            self.body_axes.quiver(0, 0, 0, *vector, color=color, length=1.0, label=label)
        self.body_axes.set_xlim(-1, 1)
        self.body_axes.set_ylim(-1, 1)
        self.body_axes.set_zlim(1, -1)  # Display NED down axis downward.
        self.body_axes.set_xlabel("North")
        self.body_axes.set_ylabel("East")
        self.body_axes.set_zlabel("Down")
        self.body_axes.legend(loc="upper left")
        age_color = "tab:red" if snapshot.message_age > self.arguments.stale_timeout else "black"
        self.body_axes.set_title(f"body axes in NED | data age={snapshot.message_age:.2f} s",
                                 color=age_color)
        self.figure.tight_layout()
        self.figure.canvas.draw_idle()


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--attitude-topic", default="/px4_1/fmu/out/vehicle_attitude")
    parser.add_argument(
        "--angular-velocity-topic", default="/px4_1/fmu/out/vehicle_angular_velocity"
    )
    parser.add_argument("--window", type=float, default=15.0, help="history window [s]")
    parser.add_argument("--refresh-rate", type=float, default=10.0, help="plot refresh rate [Hz]")
    parser.add_argument("--rate-warning", type=float, default=2.0, help="body-rate warning [rad/s]")
    parser.add_argument(
        "--acceleration-warning", type=float, default=8.0,
        help="2 s angular-acceleration RMS warning [rad/s^2]",
    )
    parser.add_argument("--stale-timeout", type=float, default=0.5, help="stale data warning [s]")
    arguments = parser.parse_args()
    if arguments.window <= 0 or arguments.refresh_rate <= 0:
        parser.error("window and refresh-rate must be positive")
    return arguments


def main() -> None:
    arguments = parse_arguments()
    rclpy.init()
    node = AttitudeMonitorNode(arguments)
    plot = LivePlot(node, arguments)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        while rclpy.ok() and plt.fignum_exists(plot.figure.number):
            plot.update()
            plt.pause(1.0 / arguments.refresh_rate)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)
        node.destroy_node()
        plt.close("all")


if __name__ == "__main__":
    main()
