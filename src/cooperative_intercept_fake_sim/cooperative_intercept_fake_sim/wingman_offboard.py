"""僚机 offboard 控制节点（Wingman Offboard Controller）。

"虚"体现在算法层：僚机**没有感知/估计/跟踪算法**，直接用目标真值
（/tracking/target/state）计算参考位置，把位置/速度设定值发给真实 PX4
（SITL 实例 2/3）执行；起飞、姿态、电机动力学全部由 PX4 飞控完成。

流程（每架僚机独立）：
    1. 起飞：px4-commander --instance N takeoff（高度与包围位 z 一致）；
    2. 切 offboard：稳定后 mode offboard；
    3. 引导：P1 距敌机 35 m 内触发，以 P1 实时方位为基准生成
       ±120°、18~25 m 的近似等边三角形包围位。P2/P3 的 NED-z
       钳在敌机下方 15~25 m 且贴近 P1 高度层；三轴参考均经过
       一阶滤波、速度/加速度限幅及防穿越安全门控。

坐标系：目标真值为 map_ned；僚机 local NED 原点 = 其起飞点
（config initial 水平坐标），设定值 = 参考点 − 起飞点。
"""

import os
import subprocess

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import Odometry
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                                  VehicleCommand, VehicleCommandAck,
                                  VehicleLocalPosition, VehicleStatus)
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from .utils import clamp_f, safe_atan2

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PX4_BIN_DIR = '/home/key/PX4-Autopilot/build/px4_sitl_default/bin'
COMMANDER = os.path.join(PX4_BIN_DIR, 'px4-commander')
PARAM = os.path.join(PX4_BIN_DIR, 'px4-param')



def wrap_pi(angle):
    """将角度包裹到 [-pi, pi)，供方位角平滑插值。"""
    return (float(angle) + np.pi) % (2.0 * np.pi) - np.pi


def formation_layer_z(p1_z, target_z, sep_min, sep_max):
    """NED 高度层：贴近 P1，同时保证在目标下方 sep_min~sep_max。"""
    return float(np.clip(float(p1_z), float(target_z) + float(sep_min),
                         float(target_z) + float(sep_max)))


class WingmanOffboardNode(Node):
    def __init__(self):
        super().__init__('wingman_offboard')

        self.declare_parameter('config_file', '')
        self.declare_parameter('wingman_enabled', True)
        self.wingman_enabled = bool(self.get_parameter('wingman_enabled').value)
        cfg_file = self.get_parameter('config_file').value
        if not cfg_file:
            cfg_file = os.path.join(PKG_ROOT, 'config', 'hybrid.yaml')
        self.cfg = yaml.safe_load(open(cfg_file, encoding='utf-8'))
        self.dt = float(self.cfg['simulation']['dt'])
        sim_cfg = self.cfg['simulation']
        self.engage_delay = float(sim_cfg['engage_delay_s'])
        self.guide_min_target_altitude = float(
            sim_cfg.get('guide_min_target_altitude_m', 20.0))
        self.altitude_tolerance = float(
            sim_cfg.get('wingman_altitude_tolerance_m', 1.5))
        self.altitude_hold_s = float(
            sim_cfg.get('wingman_altitude_hold_s', 1.0))

        # 僚机起飞点（map_ned 水平坐标）→ local 系换算
        init = self.cfg['initial']
        self.origins = {'p2': np.array([init['p2']['x'], init['p2']['y'], 0.0]),
                        'p3': np.array([init['p3']['x'], init['p3']['y'], 0.0])}
        self.instances = {'p2': 2, 'p3': 3}
        self.z_cmd = {'p2': float(self.cfg['roles']['p2']['z_m']),
                      'p3': float(self.cfg['roles']['p3']['z_m'])}

        self.target_odom = None
        self.p1 = None
        self.p1_vel = None
        self.create_subscription(
            Odometry, self.cfg['simulation']['target_state_topic'],
            self._on_target, 10)
        self.create_subscription(
            Odometry, self.cfg['simulation']['relative_truth_topic'],
            self._on_rel_truth, 10)
        # PX4 uXRCE-DDS 话题为 BEST_EFFORT + depth 5（必须匹配，否则收不到）
        self.px4_qos = QoSProfile(depth=5,
                                  reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.wing_local = {'p2': None, 'p3': None}   # 僚机 PX4 位置（local NED）
        self.wing_velocity = {'p2': np.zeros(3), 'p3': np.zeros(3)}
        self.armed = {'p2': False, 'p3': False}       # 僚机 arm 状态
        self.nav_state = {'p2': 0, 'p3': 0}            # 僚机导航模式（使用消息常量判定）
        self.preflight_ok = {'p2': False, 'p3': False}  # PX4 preflight 通过
        self.acks = {'p2': [], 'p3': []}                # 命令 ack（诊断）
        for k in ('p2', 'p3'):
            self.create_subscription(
                VehicleLocalPosition,
                f'/px4_{self.instances[k]}/fmu/out/vehicle_local_position',
                lambda msg, key=k: self._on_wing_local(key, msg),
                self.px4_qos)
            self.create_subscription(
                VehicleStatus,
                f'/px4_{self.instances[k]}/fmu/out/vehicle_status',
                lambda msg, key=k: self._on_wing_status(key, msg),
                self.px4_qos)
            self.create_subscription(
                VehicleCommandAck,
                f'/px4_{self.instances[k]}/fmu/out/vehicle_command_ack',
                lambda msg, key=k: self._on_ack(key, msg),
                self.px4_qos)
        self.setpoint_pubs = {}
        self.offboard_pubs = {}
        self.cmd_pubs = {}
        self.reference_pubs = {}
        for k in ('p2', 'p3'):
            self.setpoint_pubs[k] = self.create_publisher(
                TrajectorySetpoint, f'/px4_{self.instances[k]}/fmu/in/'
                                    f'trajectory_setpoint', self.px4_qos)
            # PX4 offboard 激活需要 offboard_control_mode + setpoint 双流
            self.offboard_pubs[k] = self.create_publisher(
                OffboardControlMode, f'/px4_{self.instances[k]}/fmu/in/'
                                     f'offboard_control_mode', self.px4_qos)
            self.cmd_pubs[k] = self.create_publisher(
                VehicleCommand, f'/px4_{self.instances[k]}/fmu/in/'
                                f'vehicle_command', self.px4_qos)
            self.reference_pubs[k] = self.create_publisher(
                Odometry, f'/intercept/{k}/reference', 10)

        # 方向滤波（目标速度方向）与僚机状态机
        self._dir_u = np.array([1.0, 0.0, 0.0])
        # wait -> offboard_req -> arm_req -> climb -> guide
        self.stage = {'p2': 'wait', 'p3': 'wait'}
        self._altitude_hold = {'p2': 0.0, 'p3': 0.0}
        self._arm_tries = {'p2': 0, 'p3': 0}
        self._t = 0.0
        self._cmd_times = {'p2': -1e9, 'p3': -1e9}
        formation = self.cfg['formation']
        self._formation_triggered = False
        self._formation_blend = 0.0
        self._theta1_cmd = None
        self._ref_pos = {
            k: np.array([self.origins[k][0], self.origins[k][1],
                         float(self.cfg['roles'][k]['z_m'])])
            for k in ('p2', 'p3')
        }
        self._ref_vel = {k: np.zeros(3) for k in ('p2', 'p3')}
        self._last_safety_log = {'p2': -1e9, 'p3': -1e9}

        self.create_timer(1.0 / float(self.cfg['simulation']['rate_hz']),
                          self._step)
        if self.wingman_enabled:
            self.get_logger().info(
                "[Wingman Offboard] 僚机 P2(/px4_2) P3(/px4_3) 就绪："
                "真值→位置指令→PX4 offboard 执行")
        else:
            self.get_logger().info("[Wingman Offboard] 僚机已禁用（对照实验："
                                   "僚机 PX4 停地面，仅验证实机）")

    # ------------------------------------------------------------------
    def _on_target(self, msg):
        self.target_odom = msg

    def _on_rel_truth(self, msg):
        """目标相对 P1 真值 → P1 公共 map_ned 位置（只读，不反馈单机）。"""
        if self.target_odom is None:
            return
        tp = self.target_odom.pose.pose.position
        tv = self.target_odom.twist.twist.linear
        rp = msg.pose.pose.position
        rv = msg.twist.twist.linear
        self.p1 = np.array([tp.x - rp.x, tp.y - rp.y, tp.z - rp.z])
        self.p1_vel = np.array([tv.x - rv.x, tv.y - rv.y, tv.z - rv.z])

    def _on_wing_local(self, key, msg):
        """僚机 PX4 位置（local NED → 保留，转 map 时加起飞点）。"""
        self.wing_local[key] = np.array([msg.x, msg.y, msg.z])
        self.wing_velocity[key] = np.array([msg.vx, msg.vy, msg.vz])

    def _on_wing_status(self, key, msg):
        """僚机 arm 状态与导航模式。"""
        self.armed[key] = bool(
            msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.nav_state[key] = int(msg.nav_state)
        self.preflight_ok[key] = bool(msg.pre_flight_checks_pass)

    def _on_ack(self, key, msg):
        """命令应答（诊断）：result 0=ACCEPTED 1=TEMP_REJECTED 2=DENIED。"""
        self.acks[key].append((msg.command, int(msg.result), int(msg.result_param1)))
        if len(self.acks[key]) <= 8:
            self.get_logger().info(
                f"{key} ACK cmd={msg.command} result={msg.result} "
                f"param1={msg.result_param1}")

    def _vc(self, key, command, param1=0.0, param2=0.0):
        """发送 vehicle_command（与实机 offboard_vehicle 相同机制）。"""
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        # PX4 system id = instance + 1（实机 instance 1 → system 2，同 offboard_vehicle）
        msg.target_system = self.instances[key] + 1
        msg.target_component = 1
        msg.source_system = 255
        msg.source_component = 0
        msg.from_external = True
        self.cmd_pubs[key].publish(msg)

    def _commander(self, instance, *args):
        """调用 px4-commander（SITL 控制台命令）。"""
        try:
            r = subprocess.run([COMMANDER, '--instance', str(instance), *args],
                               capture_output=True, text=True, timeout=10)
            out = (r.stdout or r.stderr or '').strip()
            if r.returncode != 0 and out:
                self.get_logger().warn(f"px4-commander {instance} {args}: {out}")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"px4-commander {instance} {args} 失败: {e}")

    def _setup_params(self, instance):
        """僚机 PX4 参数：解除巡航限速（MPC_XY_CRUISE 默认 5 m/s）并跳过
        preflight 检查（22001 SITL 无 GPS + EKF yaw 未收敛会被自动解除 arm）。"""
        for name, value in (('MPC_XY_CRUISE', '16.0'),
                            ('MPC_XY_VEL_MAX', '18.0'),
                            ('MPC_XY_MAN_EXPO', '1.0'),
                            ('MPC_ACC_HOR_MAX', '12.0'),
                            # 跳过 EKF/无 GPS 的 preflight 拒绝，允许稳定 arm
                            ('COM_ARM_EKF_CHK', '0'),
                            ('COM_ARM_WO_GPS', '1'),
                            # armed 后未离地时不被"落地/起飞前自动解除" disarm
                            ('COM_DISARM_LAND', '0'),
                            ('COM_DISARM_PREFLIGHT', '0'),
                            ('SYS_HITL', '0')):
            try:
                r = subprocess.run(
                    [PARAM, '--instance', str(instance), 'set', name, value],
                    capture_output=True, text=True, timeout=10)
                out = (r.stdout or r.stderr or '').strip()
                if r.returncode != 0:
                    self.get_logger().warn(f"px4-param {instance} {name} 失败: {out}")
                else:
                    self.get_logger().info(f"px4-param {instance} {name}={value} OK")
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f"px4-param {instance} {name} 失败: {e}")

    # ------------------------------------------------------------------
    def _desired_layer_z(self):
        """按 P1 实时高度生成敌机下方安全高度层（NED）。"""
        p_tz = float(self.target_odom.pose.pose.position.z)
        p1_z = float(self.p1[2]) if self.p1 is not None else \
            float(self.cfg['roles']['p2']['z_m'])
        fc = self.cfg['formation']
        return formation_layer_z(
            p1_z, p_tz, fc['vertical_sep_min_m'], fc['vertical_sep_max_m'])

    def _wingman_goal(self, key, p_w):
        """以 P1 方位为基准的目标中心近似等边三角形包围点。"""
        p_t = np.array([self.target_odom.pose.pose.position.x,
                        self.target_odom.pose.pose.position.y,
                        self.target_odom.pose.pose.position.z])
        v_t = np.array([self.target_odom.twist.twist.linear.x,
                        self.target_odom.twist.twist.linear.y,
                        self.target_odom.twist.twist.linear.z])
        v_mag = float(np.linalg.norm(v_t[:2]))
        if v_mag > 0.3:
            new_dir = np.array([v_t[0] / v_mag, v_t[1] / v_mag, 0.0])
            if np.dot(self._dir_u[:2], new_dir[:2]) < 0.0:
                self._dir_u = new_dir
            else:
                alpha = np.exp(-self.dt / 0.25)
                self._dir_u = (self._dir_u * alpha + new_dir * (1.0 - alpha))
                self._dir_u = self._dir_u / np.linalg.norm(self._dir_u)
        if self.p1 is not None:
            r1 = self.p1[:2] - p_t[:2]
            d1 = float(np.linalg.norm(self.p1 - p_t))
            if d1 <= float(self.cfg['formation']['entry_p1_range_m']):
                self._formation_triggered = True
        else:
            r1 = -self._dir_u[:2]
        if np.linalg.norm(r1) < 1e-3:
            r1 = -self._dir_u[:2]

        fc = self.cfg['formation']
        blend_rate = (fc['entry_blend_rate_per_s'] if self._formation_triggered
                      else -fc['exit_blend_rate_per_s'])
        # 每个 timer 周期只由 P2 分支推进公共 blend，确保两侧完全对称。
        if key == 'p2':
            self._formation_blend = clamp_f(
                self._formation_blend + float(blend_rate) * self.dt)

        theta1_measured = safe_atan2(r1[1], r1[0], fallback=0.0)
        if self._theta1_cmd is None:
            self._theta1_cmd = theta1_measured
        if key == 'p2':
            max_step = np.radians(float(
                fc['p1_bearing_rate_limit_deg_s'])) * self.dt
            delta = float(np.clip(wrap_pi(theta1_measured - self._theta1_cmd),
                                  -max_step, max_step))
            self._theta1_cmd = wrap_pi(self._theta1_cmd + delta)
        theta_goal = self._theta1_cmd + (2.0 * np.pi / 3.0 if key == 'p2'
                               else -2.0 * np.pi / 3.0)
        radius = float(np.clip(np.linalg.norm(r1), fc['radius_min_m'],
                               fc['radius_max_m']))

        # 未触发时以各自起飞点相对敌机的极坐标为入口；用极坐标插值而非
        # 两点直线插值，避免参考路径的弦穿过敌机。
        ingress = self.origins[key][:2] - p_t[:2]
        ingress_r = max(float(np.linalg.norm(ingress)), radius)
        ingress_theta = safe_atan2(ingress[1], ingress[0], fallback=theta_goal)
        b = self._formation_blend
        theta = ingress_theta + b * wrap_pi(theta_goal - ingress_theta)
        ring_r = (1.0 - b) * ingress_r + b * radius
        p_ref = np.array([p_t[0] + ring_r * np.cos(theta),
                          p_t[1] + ring_r * np.sin(theta),
                          self._desired_layer_z()])
        p_ref = self._safety_gate(key, p_w, p_ref, p_t)
        yaw = safe_atan2(v_t[1], v_t[0], fallback=0.0)
        return p_ref, yaw

    def _safety_gate(self, key, p_w, desired, p_t):
        """实际状态优先的安全门控：危险时减速、悬停或向外退回。"""
        fc = self.cfg['formation']
        slow = float(fc['safety_slow_inter_pursuer_m'])
        retreat = np.zeros(3)
        reasons = []
        others = []
        if self.p1 is not None:
            others.append(('p1', self.p1,
                           self.p1_vel if self.p1_vel is not None
                           else np.zeros(3)))
        other_key = 'p3' if key == 'p2' else 'p2'
        if self.wing_local[other_key] is not None:
            others.append((other_key, self._wing_pos(other_key),
                           self.wing_velocity[other_key]))
        own_velocity = self.wing_velocity[key]
        for name, other, other_velocity in others:
            delta = p_w - other
            distance = float(np.linalg.norm(delta))
            desired_distance = float(np.linalg.norm(desired - other))
            relative_velocity = own_velocity - other_velocity
            speed_sq = float(np.dot(relative_velocity, relative_velocity))
            horizon = float(fc['safety_prediction_horizon_s'])
            t_ca = float(np.clip(
                -np.dot(delta, relative_velocity) / max(speed_sq, 1e-6),
                0.0, horizon))
            closest_delta = delta + relative_velocity * t_ca
            predicted_distance = float(np.linalg.norm(closest_delta))
            predict_gate = float(fc['safety_prediction_distance_m'])
            if t_ca > 0.05 and predicted_distance < predict_gate:
                direction = delta / max(distance, 1e-6)
                avoid_distance = float(fc['safety_avoid_distance_m'])
                retreat += direction * (
                    avoid_distance - predicted_distance + 2.0)
                reasons.append(
                    f'{name}:预测{predicted_distance:.1f}m/{t_ca:.1f}s')
            if distance < slow:
                direction = delta / max(distance, 1e-6)
                retreat += direction * (slow - distance + 1.0)
                reasons.append(f'{name}:{distance:.1f}m')
            elif desired_distance < float(fc['safety_min_inter_pursuer_m']):
                desired = p_w.copy()
                reasons.append(f'{name}:候选{desired_distance:.1f}m')

        target_delta = p_w - p_t
        target_distance = float(np.linalg.norm(target_delta))
        if target_distance < float(fc['safety_retreat_range_m']):
            direction = target_delta / max(target_distance, 1e-6)
            retreat += direction * (
                float(fc['safety_retreat_range_m']) - target_distance + 1.0)
            reasons.append(f'target:{target_distance:.1f}m')
        if np.linalg.norm(retreat) > 1e-6:
            desired = p_w + retreat

        # 最后一道参考点保护：XY 投影不得穿越目标中心的 18 m 圆。
        rel_xy = desired[:2] - p_t[:2]
        d_xy = float(np.linalg.norm(rel_xy))
        min_ring = float(fc['radius_min_m'])
        if d_xy < min_ring:
            if d_xy < 1e-6:
                rel_xy = p_w[:2] - p_t[:2]
                d_xy = max(float(np.linalg.norm(rel_xy)), 1e-6)
            desired[:2] = p_t[:2] + rel_xy / d_xy * min_ring
            reasons.append('参考防穿越')
        if reasons and self._t - self._last_safety_log[key] >= 1.0:
            self.get_logger().warn(f"{key} 安全门控: {', '.join(reasons)}")
            self._last_safety_log[key] = self._t
        return desired

    def _advance_reference(self, key, desired, feedforward_xy=None):
        """一阶参考滤波 + 分轴速度/加速度限制（NED）。"""
        fc = self.cfg['formation']
        pos = self._ref_pos[key]
        vel = self._ref_vel[key]
        tau_xy = max(float(fc['horizontal_filter_tau_s']), self.dt)
        tau_z = max(float(fc['vertical_filter_tau_s']), self.dt)
        wanted = np.zeros(3)
        wanted[:2] = (desired[:2] - pos[:2]) / tau_xy
        if feedforward_xy is not None:
            wanted[:2] += np.asarray(feedforward_xy, dtype=float)[:2]
        p_t_xy = np.array([self.target_odom.pose.pose.position.x,
                           self.target_odom.pose.pose.position.y])
        rel_now = pos[:2] - p_t_xy
        d_now = float(np.linalg.norm(rel_now))
        min_ring = float(fc['radius_min_m'])
        if d_now <= min_ring + 0.5 and d_now > 1e-6:
            radial = rel_now / d_now
            target_velocity_xy = np.array([
                self.target_odom.twist.twist.linear.x,
                self.target_odom.twist.twist.linear.y])
            inward_speed = -float(np.dot(
                wanted[:2] - target_velocity_xy, radial))
            if inward_speed > 0.0:
                wanted[:2] += radial * inward_speed
        wanted[2] = (desired[2] - pos[2]) / tau_z
        xy_speed = float(np.linalg.norm(wanted[:2]))
        xy_limit = float(fc['horizontal_speed_limit_mps'])
        if xy_speed > xy_limit:
            wanted[:2] *= xy_limit / xy_speed
        wanted[2] = float(np.clip(
            wanted[2], -float(fc['vertical_speed_limit_mps']),
            float(fc['vertical_speed_limit_mps'])))

        dv = wanted - vel
        xy_dv = float(np.linalg.norm(dv[:2]))
        xy_dv_limit = float(fc['horizontal_accel_limit_mps2']) * self.dt
        if xy_dv > xy_dv_limit:
            dv[:2] *= xy_dv_limit / xy_dv
        z_dv_limit = float(fc['vertical_accel_limit_mps2']) * self.dt
        dv[2] = float(np.clip(dv[2], -z_dv_limit, z_dv_limit))
        vel = vel + dv
        pos = pos + vel * self.dt

        # 禁止用几何投影瞬移参考点；防穿越由上方相对径向速度门控、
        # 速度/加速度限幅及实际状态预测避让共同保证。
        self._ref_pos[key] = pos
        self._ref_vel[key] = vel
        return pos.copy(), vel.copy()

    # ------------------------------------------------------------------
    def _step(self):
        if not self.wingman_enabled:
            return
        if self.target_odom is None:
            return
        t = self._t
        for k in ('p2', 'p3'):
            inst = self.instances[k]
            stage = self.stage[k]

            if stage == 'wait':
                # 持续发垂直起飞设定点（offboard 预热流），等 preflight 稳定
                self._publish_setpoint(k, self._takeoff_ref(k), np.zeros(3), 0.0)
                if t >= self.engage_delay and self.preflight_ok[k]:
                    self._setup_params(inst)
                    self.get_logger().info(f"{k} preflight 通过，请求 OFFBOARD")
                    self.stage[k] = 'offboard_req'
                    self._cmd_times[k] = t
            elif stage == 'offboard_req':
                self._publish_setpoint(k, self._takeoff_ref(k), np.zeros(3), 0.0)
                if self.nav_state[k] == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                    self.get_logger().info(f"{k} 已进入 OFFBOARD，请求 arm")
                    self.stage[k] = 'arm_req'
                    self._cmd_times[k] = t
                elif t - self._cmd_times[k] >= 1.0:
                    self._vc(k, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                    self._cmd_times[k] = t
            elif stage == 'arm_req':
                self._publish_setpoint(k, self._takeoff_ref(k), np.zeros(3), 0.0)
                if self.armed[k]:
                    self.get_logger().info(
                        f"{k} 已 arm，先保持水平位置并追踪目标高度")
                    self.stage[k] = 'climb'
                    self._altitude_hold[k] = 0.0
                    self._cmd_times[k] = t
                elif t - self._cmd_times[k] >= 1.0:
                    self._vc(k, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                    self._cmd_times[k] = t
            elif stage == 'climb':
                desired = self._formation_altitude_ref(k)
                p_ref, v_ref = self._advance_reference(k, desired)
                self._publish_setpoint(k, p_ref, v_ref, 0.0)
                if not self.armed[k]:
                    self.get_logger().warn(f"{k} 爬升期间被 disarm，重新 arm")
                    self.stage[k] = 'arm_req'
                    self._cmd_times[k] = t
                elif self.nav_state[k] != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                    self.get_logger().warn(f"{k} 爬升期间丢失 OFFBOARD，重新切换")
                    self.stage[k] = 'offboard_req'
                    self._cmd_times[k] = t
                else:
                    loc = self.wing_local[k]
                    local_z_ref = p_ref[2] - self.origins[k][2]
                    target_altitude_ready = (
                        -float(self.target_odom.pose.pose.position.z)
                        >= self.guide_min_target_altitude)
                    altitude_ready = (
                        loc is not None
                        and abs(float(loc[2]) - local_z_ref)
                        <= self.altitude_tolerance)
                    if target_altitude_ready and altitude_ready:
                        self._altitude_hold[k] += self.dt
                        if self._altitude_hold[k] >= self.altitude_hold_s:
                            self.get_logger().info(
                                f"{k} 高度稳定，开始水平包围引导")
                            self.stage[k] = 'guide'
                    else:
                        self._altitude_hold[k] = 0.0
            elif stage == 'guide':
                if not self.armed[k]:
                    self.get_logger().warn(f"{k} guide 期间被 disarm，重新 arm")
                    self.stage[k] = 'arm_req'
                    self._cmd_times[k] = t
                elif self.nav_state[k] != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                    self.get_logger().warn(f"{k} 丢失 OFFBOARD，重新切换")
                    self.stage[k] = 'offboard_req'
                    self._cmd_times[k] = t
                else:
                    p_w = self._wing_pos(k)
                    loc = self.wing_local[k]
                    if loc is None or loc[2] > -4.5:
                        desired = self._takeoff_ref(k)
                        p_ref, v_ref = self._advance_reference(k, desired)
                        yaw = 0.0
                    else:
                        desired, yaw = self._wingman_goal(k, p_w)
                        target_velocity = np.array([
                            self.target_odom.twist.twist.linear.x,
                            self.target_odom.twist.twist.linear.y])
                        p_ref, v_ref = self._advance_reference(
                            k, desired,
                            target_velocity * self._formation_blend)
                    self._publish_setpoint(k, p_ref, v_ref, yaw)
        self._t += self.dt

    def _takeoff_ref(self, key):
        """起飞过渡参考：原地垂直爬升到包围位高度（map_ned）。"""
        return np.array([self.origins[key][0], self.origins[key][1],
                         self.cfg['roles'][key]['z_m']])

    def _formation_altitude_ref(self, key):
        """水平保持起飞点，平滑进入 P1 附近的敌机下方高度层。"""
        return np.array([self.origins[key][0], self.origins[key][1],
                         self._desired_layer_z()])

    def _publish_setpoint(self, key, p_ref, v_ref, yaw):
        """发布 offboard_control_mode + trajectory_setpoint（位置模式；
        local = map − 起飞点）。"""
        ts = int(self.get_clock().now().nanoseconds / 1000)
        ocm = OffboardControlMode()
        ocm.timestamp = ts
        ocm.position = True
        ocm.velocity = True
        ocm.acceleration = False
        ocm.attitude = False
        ocm.body_rate = False
        ocm.thrust_and_torque = False
        ocm.direct_actuator = False
        self.offboard_pubs[key].publish(ocm)
        sp = TrajectorySetpoint()
        sp.timestamp = ts
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        sp.position[0] = float(p_ref[0] - self.origins[key][0])
        sp.position[1] = float(p_ref[1] - self.origins[key][1])
        sp.position[2] = float(p_ref[2] - self.origins[key][2])
        sp.velocity[0] = float(v_ref[0])
        sp.velocity[1] = float(v_ref[1])
        sp.velocity[2] = float(v_ref[2])
        sp.yaw = float(yaw)
        self.setpoint_pubs[key].publish(sp)

        diag = Odometry()
        diag.header.stamp = self.get_clock().now().to_msg()
        diag.header.frame_id = 'map_ned'
        diag.child_frame_id = f'{key}_reference'
        diag.pose.pose.position.x = float(p_ref[0])
        diag.pose.pose.position.y = float(p_ref[1])
        diag.pose.pose.position.z = float(p_ref[2])
        diag.twist.twist.linear.x = float(v_ref[0])
        diag.twist.twist.linear.y = float(v_ref[1])
        diag.twist.twist.linear.z = float(v_ref[2])
        self.reference_pubs[key].publish(diag)

    def _wing_pos(self, key):
        """僚机当前位置（map_ned）= PX4 local + 起飞点（未就绪时用初始位置）。"""
        loc = self.wing_local[key]
        if loc is None:
            init = self.cfg['initial'][key]
            return np.array([init['x'], init['y'], init['z']])
        return loc + self.origins[key]


def main(args=None):
    rclpy.init(args=args)
    node = WingmanOffboardNode()
    try:
        rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException:
        pass
    except Exception as e:
        # launch 收尾时 context 会先于本 Python executor 失效；这不是飞行故障。
        if rclpy.ok():
            node.get_logger().error(f"Wingman Offboard 失败: {e}")
            raise
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
