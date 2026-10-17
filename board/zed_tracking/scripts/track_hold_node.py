#!/usr/bin/env python3
"""track_hold_node:跟踪保持控制节点(M3)。

任务:本机保持在目标斜下方/正下方 7~8 m,机头(yaw,枪轴)指向目标,
满足"距离窗口 + 指向误差"后触发虚拟射击事件。不再撞击。

输入:滤波节点 track_json(目标 NED 位置/速度)、飞控 local_position/
attitude/status。
输出:与旧撞击链路一致的上游通道 /upstream/fmu/in/(offboard_control_mode
+ trajectory_setpoint,速度+yaw),经原有 upstream_gate 与
px4_upstream_adapter 两道安全阀门转发;本节点不解锁、不切模式。

纯函数区(速度律/限幅/yaw/状态机转移)由单元测试覆盖。
"""

from __future__ import annotations

import json
import math

# ---------------------------------------------------------------------------
# 纯函数区
# ---------------------------------------------------------------------------


def clamp(value, minimum, maximum):
    return max(float(minimum), min(float(maximum), float(value)))


def velocity_towards_hold(
    rel_ned, target_vel_ned, hold_distance_m, hold_back_m, gain,
    max_speed_mps, max_vertical_mps,
):
    """保持距离速度律。

    hold_back_m > 0(斜下方模式):期望本机位于目标水平后方 hold_back_m、
    下方 sqrt(hold^2-back^2),视线恒有水平分量,机头指向判定永远有定义
    (正下方保持时视线竖直,yaw 无参考,2026-10-07 仿真结论)。
    hold_back_m = 0:沿视线方向退 hold_distance(纯测距保持)。
    速度 = 增益 x 位置误差 + 目标速度前馈,再分轴限幅。
    返回 (velocity_ned, range_m)。rel 为 目标 - 本机。
    """
    rel = [float(v) for v in rel_ned]
    distance = math.sqrt(sum(v * v for v in rel))
    if distance < 1.0e-3:
        return [0.0, 0.0, 0.0], distance
    if float(hold_back_m) > 0.0:
        horizontal = math.hypot(rel[0], rel[1])
        if horizontal < 1.0e-3:
            unit = [1.0, 0.0]
        else:
            unit = [rel[0] / horizontal, rel[1] / horizontal]
        back = float(hold_back_m)
        vertical = math.sqrt(max(float(hold_distance_m) ** 2 - back * back, 0.25))
        desired_rel = [unit[0] * back, unit[1] * back, -vertical]
        # rel = target - self,d(rel)/dt = -v:速度 = gain*(rel - desired)。
        error = [rel[i] - desired_rel[i] for i in range(3)]
    else:
        unit = [v / distance for v in rel]
        error = [unit[i] * (distance - float(hold_distance_m)) for i in range(3)]
    feedforward = [float(v) for v in target_vel_ned]
    command = [
        clamp(float(gain) * error[i] + feedforward[i], -1.0e3, 1.0e3)
        for i in range(3)
    ]
    horizontal_speed = math.hypot(command[0], command[1])
    if horizontal_speed > float(max_speed_mps) and horizontal_speed > 1.0e-9:
        scale = float(max_speed_mps) / horizontal_speed
        command[0] *= scale
        command[1] *= scale
    command[2] = clamp(command[2], -float(max_vertical_mps), float(max_vertical_mps))
    return command, distance


def retreat_velocity(rel_ned, retreat_speed_mps):
    """过近保护:沿视线后退(远离目标方向)。"""
    rel = [float(v) for v in rel_ned]
    distance = math.sqrt(sum(v * v for v in rel))
    if distance < 1.0e-3:
        return [0.0, 0.0, 0.0]
    return [-v / distance * float(retreat_speed_mps) for v in rel]


def rate_limit_velocity(command, previous, dt_s, max_accel_mps2, max_vertical_mps2):
    """加速度限幅:限制速度指令变化率,间接限制机体倾斜(小火箭构型
    视场保护的核心;倾斜 ~ atan(a/g),a=4 m/s^2 约 22 度,默认更宽松)。"""
    dt = max(1.0e-3, float(dt_s))
    max_delta = float(max_accel_mps2) * dt
    max_delta_z = float(max_vertical_mps2) * dt
    limited = []
    for axis in range(2):
        delta = clamp(command[axis] - previous[axis], -max_delta, max_delta)
        limited.append(previous[axis] + delta)
    delta_z = clamp(command[2] - previous[2], -max_delta_z, max_delta_z)
    limited.append(previous[2] + delta_z)
    return limited


def yaw_towards(rel_ned):
    """机头指向目标方位(NED yaw,弧度)。"""
    return math.atan2(float(rel_ned[1]), float(rel_ned[0]))


def wrap_pi(angle):
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def aim_error_rad(rel_ned, yaw_rad):
    """枪轴(机头)与目标视线的水平指向误差角。"""
    return wrap_pi(yaw_towards(rel_ned) - float(yaw_rad))


def step_state_machine(state, *, track_valid, range_m, aim_error_rad_value,
                       fire_cooldown_active, params):
    """状态机转移。返回 (新状态, 动作列表)。

    SEARCH    无有效跟踪,悬停等待
    APPROACH  距离 > hold_enter_m,接近
    HOLD      距离窗口内保持
    AIM       HOLD 且指向误差达标,累计稳定时间
    FIRE      满足射击条件(一次),随后冷却
    LOST      跟踪短暂丢失(< lost_hold_s),按最后已知方向保持
    """
    hold_min = float(params["hold_min_m"])
    hold_max = float(params["hold_max_m"])
    aim_threshold = float(params["aim_error_threshold_deg"]) * math.pi / 180.0
    actions = []
    if state == "FIRE":
        actions.append("emit_shot")
        return "HOLD", actions
    if not track_valid:
        if state in ("HOLD", "AIM", "APPROACH", "LOST"):
            return "LOST", actions
        return "SEARCH", actions
    if state == "SEARCH" or state == "LOST":
        return ("APPROACH" if range_m > hold_max else "HOLD"), actions
    if state == "APPROACH":
        if range_m <= float(params["approach_exit_m"]):
            return "HOLD", actions
        return "APPROACH", actions
    if state == "HOLD":
        if range_m < hold_min:
            actions.append("retreat")
            return "HOLD", actions
        if range_m > hold_max:
            return "APPROACH", actions
        if abs(aim_error_rad_value) <= aim_threshold and not fire_cooldown_active:
            return "AIM", actions
        return "HOLD", actions
    if state == "AIM":
        if range_m < hold_min or range_m > hold_max:
            return "HOLD", actions
        if abs(aim_error_rad_value) > aim_threshold * 1.5:
            return "HOLD", actions
        return "AIM", actions  # 稳定时长由节点计时,满足后切 FIRE
    return "SEARCH", actions


# ---------------------------------------------------------------------------
# 节点
# ---------------------------------------------------------------------------


def main(args=None):
    import rclpy
    from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                              VehicleAttitude, VehicleLocalPosition,
                              VehicleStatus)
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    from std_msgs.msg import String

    class TrackHoldNode(Node):
        def __init__(self):
            super().__init__("track_hold_node")
            self.declare_parameter("rate_hz", 30.0)
            self.declare_parameter("track_topic", "/zed_tracking/track_json")
            self.declare_parameter("upstream_prefix", "/upstream/fmu/in")
            self.declare_parameter("hold_distance_m", 7.5)
            self.declare_parameter("hold_back_m", 2.5)
            self.declare_parameter("hold_min_m", 6.5)
            self.declare_parameter("hold_max_m", 8.0)
            self.declare_parameter("approach_exit_m", 8.5)
            self.declare_parameter("hold_gain", 0.8)
            self.declare_parameter("approach_max_mps", 3.5)
            self.declare_parameter("hold_max_mps", 2.5)
            self.declare_parameter("max_vertical_mps", 1.5)
            self.declare_parameter("retreat_speed_mps", 1.0)
            self.declare_parameter("max_accel_mps2", 6.0)
            self.declare_parameter("max_vertical_accel_mps2", 3.0)
            # 静态目标 5° 即可;机动目标受 yaw 动态滞后影响,放宽到 8°(可调)。
            self.declare_parameter("aim_error_threshold_deg", 8.0)
            self.declare_parameter("aim_hold_s", 0.3)
            self.declare_parameter("fire_cooldown_s", 2.0)
            self.declare_parameter("lost_hold_s", 1.0)
            self.declare_parameter("enable_output", True)
            self.declare_parameter("shot_topic", "/zed_tracking/shot_event")
            self.declare_parameter("status_topic", "/zed_tracking/track_hold_status")
            number = lambda name: float(self.get_parameter(name).value)  # noqa: E731

            self._rate_hz = max(5.0, number("rate_hz"))
            self._params = {
                name: number(name) for name in (
                    "hold_min_m", "hold_max_m", "approach_exit_m",
                    "aim_error_threshold_deg",
                )
            }
            self._hold_distance = number("hold_distance_m")
            self._hold_back = number("hold_back_m")
            self._gain = number("hold_gain")
            self._approach_max = number("approach_max_mps")
            self._hold_max = number("hold_max_mps")
            self._max_vertical = number("max_vertical_mps")
            self._retreat_speed = number("retreat_speed_mps")
            self._max_accel = number("max_accel_mps2")
            self._max_vertical_accel = number("max_vertical_accel_mps2")
            self._aim_hold = number("aim_hold_s")
            self._fire_cooldown = number("fire_cooldown_s")
            self._lost_hold = number("lost_hold_s")
            self._enable_output = bool(self.get_parameter("enable_output").value)

            self._state = "SEARCH"
            self._aim_stable_s = 0.0
            self._last_fire_s = None
            self._track = None          # 最新滤波输出 dict
            self._track_time_s = None
            self._local = None
            self._yaw = 0.0
            self._armed = False
            self._offboard = False
            self._last_command = [0.0, 0.0, 0.0]
            self._shots = 0

            sensor_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.create_subscription(
                String, str(self.get_parameter("track_topic").value),
                self._on_track, sensor_qos)
            self.create_subscription(
                VehicleLocalPosition, "/fmu/out/vehicle_local_position",
                self._on_local, sensor_qos)
            self.create_subscription(
                VehicleAttitude, "/fmu/out/vehicle_attitude",
                self._on_attitude, sensor_qos)
            self.create_subscription(
                VehicleStatus, "/fmu/out/vehicle_status",
                self._on_status, sensor_qos)

            upstream = str(self.get_parameter("upstream_prefix").value).rstrip("/")
            self._mode_pub = self.create_publisher(
                OffboardControlMode, f"{upstream}/offboard_control_mode", sensor_qos)
            self._setpoint_pub = self.create_publisher(
                TrajectorySetpoint, f"{upstream}/trajectory_setpoint", sensor_qos)
            self._shot_pub = self.create_publisher(
                String, str(self.get_parameter("shot_topic").value), 10)
            self._status_pub = self.create_publisher(
                String, str(self.get_parameter("status_topic").value), 10)

            self._period = 1.0 / self._rate_hz
            self.create_timer(self._period, self._on_timer)
            self.get_logger().warning(
                f"track_hold ready hold={self._hold_distance} m, "
                f"output={'on' if self._enable_output else 'shadow'}"
            )

        def _now_s(self):
            return self.get_clock().now().nanoseconds / 1.0e9

        def _on_track(self, message):
            try:
                payload = json.loads(message.data)
            except ValueError:
                return
            self._track = payload
            self._track_time_s = self._now_s()

        def _on_local(self, message):
            self._local = {
                "x": float(message.x), "y": float(message.y),
                "z": float(message.z),
                "vx": float(message.vx), "vy": float(message.vy),
                "vz": float(message.vz),
            }

        def _on_attitude(self, message):
            w, x, y, z = (float(v) for v in message.q[:4])
            self._yaw = math.atan2(
                2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

        def _on_status(self, message):
            self._armed = int(message.arming_state) == int(
                VehicleStatus.ARMING_STATE_ARMED)
            self._offboard = int(message.nav_state) == int(
                VehicleStatus.NAVIGATION_STATE_OFFBOARD)

        def _track_valid_now(self):
            if self._track is None or self._track_time_s is None:
                return None
            if not self._track.get("valid"):
                return None
            position = self._track.get("pos_ned")
            if not position or len(position) != 3:
                return None
            age = self._now_s() - self._track_time_s
            if age > 0.5:
                return None
            return self._track

        def _on_timer(self):
            now = self._now_s()
            track = self._track_valid_now()
            command = [0.0, 0.0, 0.0]
            yaw = self._yaw
            range_m = None
            aim_error = None
            if track is not None:
                target = track["pos_ned"]
                velocity = track.get("vel_ned") or [0.0, 0.0, 0.0]
                if self._local:
                    rel = [
                        target[0] - self._local["x"],
                        target[1] - self._local["y"],
                        target[2] - self._local["z"],
                    ]
                else:
                    rel = target
                range_m = math.sqrt(sum(v * v for v in rel))
                yaw = yaw_towards(rel)
                aim_error = aim_error_rad(rel, self._yaw)
                max_speed = (self._approach_max if self._state == "APPROACH"
                             else self._hold_max)
                command, _ = velocity_towards_hold(
                    rel, velocity, self._hold_distance, self._hold_back,
                    self._gain, max_speed, self._max_vertical)
            elif self._state == "LOST":
                command = [0.0, 0.0, 0.0]

            previous_state = self._state
            cooldown_active = (self._last_fire_s is not None and
                               now - self._last_fire_s < self._fire_cooldown)
            self._state, actions = step_state_machine(
                self._state,
                track_valid=track is not None,
                range_m=range_m if range_m is not None else 1.0e9,
                aim_error_rad_value=aim_error if aim_error is not None else 9.9,
                fire_cooldown_active=cooldown_active,
                params=self._params,
            )
            if "retreat" in actions:
                if range_m is not None and self._local:
                    target = track["pos_ned"]
                    rel = [
                        target[0] - self._local["x"],
                        target[1] - self._local["y"],
                        target[2] - self._local["z"],
                    ]
                    command = retreat_velocity(rel, self._retreat_speed)
            if self._state == "AIM" and previous_state == "AIM":
                self._aim_stable_s += self._period
            elif self._state == "AIM":
                self._aim_stable_s = 0.0
            if self._state == "AIM" and self._aim_stable_s >= self._aim_hold:
                self._state = "FIRE"

            command = rate_limit_velocity(
                command, self._last_command, self._period,
                self._max_accel, self._max_vertical_accel)
            self._last_command = command
            self._publish(command, yaw)
            self._publish_status(now, range_m, aim_error)

            if self._state == "FIRE":
                self._emit_shot(now, track, range_m, aim_error)
                self._last_fire_s = now
                self._aim_stable_s = 0.0

        def _publish(self, velocity, yaw):
            if not self._enable_output:
                return
            mode = OffboardControlMode()
            mode.velocity = True
            self._mode_pub.publish(mode)
            setpoint = TrajectorySetpoint()
            setpoint.velocity = [float(v) for v in velocity]
            setpoint.yaw = float(yaw)
            self._setpoint_pub.publish(setpoint)

        def _emit_shot(self, now, track, range_m, aim_error):
            payload = {
                "stamp_s": now,
                "sequence": self._shots + 1,
                "range_m": range_m,
                "aim_error_deg": (math.degrees(aim_error)
                                  if aim_error is not None else None),
                "target_pos_ned": track.get("pos_ned") if track else None,
                "target_vel_ned": track.get("vel_ned") if track else None,
                "shooter_pos_ned": [
                    self._local["x"], self._local["y"], self._local["z"]
                ] if self._local else None,
                "shooter_yaw_rad": self._yaw,
            }
            message = String()
            message.data = json.dumps(payload, separators=(",", ":"))
            self._shot_pub.publish(message)
            self._shots += 1
            self.get_logger().warning(
                f"SHOT #{self._shots}: range={range_m:.2f} m "
                f"aim_err={math.degrees(aim_error or 0):.1f} deg"
            )

        def _publish_status(self, now, range_m, aim_error):
            payload = {
                "stamp_s": now,
                "state": self._state,
                "range_m": range_m,
                "aim_error_deg": (math.degrees(aim_error)
                                  if aim_error is not None else None),
                "yaw_command": self._last_command,
                "shots": self._shots,
            }
            message = String()
            message.data = json.dumps(payload, separators=(",", ":"))
            self._status_pub.publish(message)

    rclpy.init(args=args)
    node = None
    try:
        node = TrackHoldNode()
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
