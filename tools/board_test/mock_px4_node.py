#!/usr/bin/env python3
"""模拟 PX4 飞控节点（无硬件全链路测试用，2026-10-15）。

在飞控未上电时为拦截链提供 /fmu/out/* 输入，并闭环消费 /fmu/in/* 指令：

  订阅  /fmu/in/trajectory_setpoint   位置/速度(+AIM加速度前馈)设定点
        /fmu/in/offboard_control_mode /fmu/in/vehicle_command（rc_gated
        模式下应为 0 条，收到即计数告警）
        /test/mock_px4/engage          std_msgs/Bool：true=armed+offboard
  发布  /fmu/out/vehicle_attitude     50Hz，板时戳（us），q 为 FRD body->NED
        /fmu/out/vehicle_local_position 30Hz（x/y/z NED，z 负为高）
        /fmu/out/vehicle_status        10Hz（arming_state/nav_state）
        /test/mock_px4/state           30Hz JSON（供虚拟相机/编排器读取位姿）

运动学：一阶位置/速度跟踪（max 10m/s、6.5m/s^2）+ yaw 伺服（3rad/s）+
倾转模型（theta=atan(|a|/g)，方向取指令加速度方向，钳 0.35rad）。
倾转平滑 0.25s。这是运动学近似（非 PX4 SITL），用于验证链路逻辑闭环。
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus

ARMING_STATE_STANDBY = 1
ARMING_STATE_ARMED = 2
NAVIGATION_STATE_MANUAL = 0
NAVIGATION_STATE_OFFBOARD = 14

G_MPS2 = 9.81


def px4_qos():
    qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT
    qos.durability = DurabilityPolicy.VOLATILE
    return qos


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_normalize(q):
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q)


def quat_from_axis_angle(axis, angle):
    n = math.sqrt(sum(c * c for c in axis))
    if n < 1e-9:
        return (1.0, 0.0, 0.0, 0.0)
    s = math.sin(angle / 2.0) / n
    return (math.cos(angle / 2.0), axis[0] * s, axis[1] * s, axis[2] * s)


def quat_from_yaw(yaw):
    return (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))


def clamp(value, limit):
    return max(-limit, min(limit, value))


def wrap_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class MockPx4Node(Node):

    def __init__(self):
        super().__init__("mock_px4")
        self.declare_parameter("max_speed_mps", 10.0)
        self.declare_parameter("max_acc_mps2", 6.5)
        self.declare_parameter("max_yaw_rate_rad_s", 3.0)
        self.declare_parameter("max_tilt_rad", 0.35)
        self.declare_parameter("start_height_m", 0.0)  # 初始悬停高度（0=地面）
        # 手动阶段模拟：armed 且未切 offboard 时，模拟飞手爬升到该高度悬停
        self.declare_parameter("manual_takeoff_height_m", 5.5)

        self._pos = [0.0, 0.0, -float(self.get_parameter("start_height_m").value)]
        self._vel = [0.0, 0.0, 0.0]
        self._yaw = 0.0
        self._tilt = 0.0
        self._tilt_dir = 0.0  # 倾转方向（body 系内角度，弧度）
        self._armed = False
        self._offboard = False
        self._sp = None
        self._sp_time_s = -1e9
        self._sp_count = 0
        self._ob_count = 0
        self._cmd_count = 0
        self._seq = 0

        self.create_subscription(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint",
            self._on_setpoint, px4_qos())
        self.create_subscription(
            OffboardControlMode, "/fmu/in/offboard_control_mode",
            self._on_ob_mode, px4_qos())
        self.create_subscription(
            VehicleCommand, "/fmu/in/vehicle_command",
            self._on_command, px4_qos())
        self.create_subscription(
            Bool, "/test/mock_px4/engage", self._on_engage, 10)
        self.create_subscription(
            Bool, "/test/mock_px4/arm", self._on_arm, 10)

        self._att_pub = self.create_publisher(
            VehicleAttitude, "/fmu/out/vehicle_attitude", px4_qos())
        self._lp_pub = self.create_publisher(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position", px4_qos())
        self._st_pub = self.create_publisher(
            VehicleStatus, "/fmu/out/vehicle_status", px4_qos())
        self._state_pub = self.create_publisher(
            String, "/test/mock_px4/state", 10)

        self._last_tick = time.monotonic()
        self._tick_count = 0
        self.create_timer(0.02, self._tick)  # 50Hz 运动学
        self.get_logger().info(
            "mock_px4 ready (start_height=%.1fm)" % -self._pos[2])

    # ---- 输入 ----

    def _on_setpoint(self, msg):
        self._sp = msg
        self._sp_time_s = time.monotonic()
        self._sp_count += 1

    def _on_ob_mode(self, msg):
        self._ob_count += 1

    def _on_command(self, msg):
        self._cmd_count += 1
        self.get_logger().warning(
            "unexpected VehicleCommand (rc_gated 应永不下发): command=%s"
            % getattr(msg, "command", "?"))

    def _on_engage(self, msg):
        self._armed = bool(msg.data)
        self._offboard = bool(msg.data)
        self.get_logger().info(
            "engage=%s (sp_received=%d)" % (msg.data, self._sp_count))

    def _on_arm(self, msg):
        """仅解锁（手动模式，模拟飞手解锁起飞前的状态）。"""
        self._armed = bool(msg.data)
        self._offboard = False
        self.get_logger().info(
            "arm=%s offboard=False (sp_received=%d)" % (msg.data, self._sp_count))

    # ---- 运动学与发布 ----

    def _tick(self):
        now = time.monotonic()
        dt = min(0.1, max(1e-3, now - self._last_tick))
        self._last_tick = now
        self._tick_count += 1
        self._step(dt)
        if self._tick_count % 1 == 0:  # 50Hz
            self._publish_attitude()
        if self._tick_count % 2 == 0:  # 25Hz
            self._publish_local_position()
            self._publish_state()
        if self._tick_count % 5 == 0:  # 10Hz
            self._publish_status()

    def _step(self, dt):
        max_speed = float(self.get_parameter("max_speed_mps").value)
        max_acc = float(self.get_parameter("max_acc_mps2").value)
        max_yaw_rate = float(self.get_parameter("max_yaw_rate_rad_s").value)
        max_tilt = float(self.get_parameter("max_tilt_rad").value)

        target_vel = [0.0, 0.0, 0.0]
        sp_valid = (self._armed and self._offboard and self._sp is not None
                    and now_age(self._sp_time_s) < 0.5)
        if sp_valid:
            sp = self._sp
            pos_finite = all(math.isfinite(v) for v in sp.position)
            vel_finite = all(math.isfinite(v) for v in sp.velocity)
            if pos_finite:
                for i in range(3):
                    error = sp.position[i] - self._pos[i]
                    target_vel[i] = clamp(error * 1.2, max_speed)
                if vel_finite:
                    for i in range(3):
                        target_vel[i] = clamp(
                            target_vel[i] + sp.velocity[i], max_speed)
            elif vel_finite:
                target_vel = [clamp(sp.velocity[i], max_speed)
                              for i in range(3)]
        elif self._armed and not self._offboard:
            # 手动阶段：模拟飞手爬升到手动起飞高度后定点悬停
            manual_h = float(self.get_parameter("manual_takeoff_height_m").value)
            target_vel = [0.0, 0.0, clamp((self._pos[2] + manual_h) * -0.8,
                                           max_speed)]

        acc = [0.0, 0.0, 0.0]
        for i in range(3):
            cmd = (target_vel[i] - self._vel[i]) * 2.0
            limit = max_acc if i < 2 else 2.0
            acc[i] = clamp(cmd, limit)
            self._vel[i] += acc[i] * dt
        for i in range(3):
            self._pos[i] += self._vel[i] * dt

        # AIM 加速度前馈直接映射为倾转指令（θ=a/g，方向取加速度方向）
        acc_ff = [0.0, 0.0]
        if sp_valid and self._sp is not None:
            if math.isfinite(self._sp.acceleration[0]):
                acc_ff[0] = self._sp.acceleration[0]
            if math.isfinite(self._sp.acceleration[1]):
                acc_ff[1] = self._sp.acceleration[1]
        acc_h = [acc[0] + acc_ff[0], acc[1] + acc_ff[1]]
        acc_h_body = rotate_z(-self._yaw, acc_h)
        acc_h_mag = math.hypot(acc_h_body[0], acc_h_body[1])
        tilt_target = min(math.atan2(acc_h_mag, G_MPS2), max_tilt)
        # 一阶平滑 0.25s
        self._tilt += (tilt_target - self._tilt) * min(1.0, dt / 0.25)
        if acc_h_mag > 0.2:
            self._tilt_dir = math.atan2(acc_h_body[1], acc_h_body[0])

        if sp_valid and math.isfinite(self._sp.yaw):
            yaw_error = wrap_angle(self._sp.yaw - self._yaw)
            self._yaw += clamp(yaw_error * 2.0, max_yaw_rate) * dt
            self._yaw = wrap_angle(self._yaw)

    def _attitude_quaternion(self):
        # body->NED = R_yaw * R_tilt；倾转使 body -Z（攻击轴/竖直向上）
        # 朝 body 系加速度方向倾斜：绕 (sin(dir+90°), cos(dir+90°)... ) 轴
        # 即绕垂直于倾转方向的横轴旋转 -tilt。
        axis = (math.sin(self._tilt_dir), -math.cos(self._tilt_dir), 0.0)
        q_tilt = quat_from_axis_angle(axis, -self._tilt)
        return quat_normalize(quat_mul(quat_from_yaw(self._yaw), q_tilt))

    def _stamp(self, msg):
        msg.timestamp = int(time.time_ns() // 1000)
        # VehicleStatus 无 timestamp_sample 字段，其余两条消息有
        if hasattr(msg, "timestamp_sample"):
            msg.timestamp_sample = msg.timestamp
        return msg

    def _publish_attitude(self):
        msg = self._stamp(VehicleAttitude())
        q = self._attitude_quaternion()
        msg.q = [q[0], q[1], q[2], q[3]]
        self._att_pub.publish(msg)

    def _publish_local_position(self):
        msg = self._stamp(VehicleLocalPosition())
        msg.x, msg.y, msg.z = self._pos
        msg.vx, msg.vy, msg.vz = self._vel
        msg.ax, msg.ay, msg.az = 0.0, 0.0, 0.0
        msg.heading = self._yaw
        msg.z_deriv = self._vel[2]
        # 制导节点只接受 xy_valid && z_valid 的位置消息（真实 PX4 会置位）
        msg.xy_valid = True
        msg.z_valid = True
        msg.v_xy_valid = True
        msg.v_z_valid = True
        msg.heading_good_for_control = True
        self._lp_pub.publish(msg)

    def _publish_status(self):
        msg = self._stamp(VehicleStatus())
        msg.arming_state = (
            ARMING_STATE_ARMED if self._armed else ARMING_STATE_STANDBY)
        msg.nav_state = (
            NAVIGATION_STATE_OFFBOARD if self._offboard
            else NAVIGATION_STATE_MANUAL)
        self._st_pub.publish(msg)

    def _publish_state(self):
        q = self._attitude_quaternion()
        self._seq += 1
        state = {
            "t": time.time(),
            "seq": self._seq,
            "pos": self._pos,
            "vel": self._vel,
            "yaw": self._yaw,
            "tilt": self._tilt,
            "q_body_to_ned": list(q),
            "armed": self._armed,
            "offboard": self._offboard,
            "sp_count": self._sp_count,
            "ob_count": self._ob_count,
            "cmd_count": self._cmd_count,
        }
        msg = String()
        msg.data = json.dumps(state)
        self._state_pub.publish(msg)


def now_age(t):
    return time.monotonic() - t


def rotate_z(angle, vector):
    c, s = math.cos(angle), math.sin(angle)
    return [c * vector[0] - s * vector[1], s * vector[0] + c * vector[1]]


def main(argv=None):
    rclpy.init(args=argv)
    node = MockPx4Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
