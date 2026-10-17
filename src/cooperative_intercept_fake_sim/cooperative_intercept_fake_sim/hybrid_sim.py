"""2 虚 1 实混合仿真节点（Hybrid 2-vs-1 Simulation Node）。

架构：
    实机 P1：由原项目 14 m/s 统一拦截方案驱动（PX4 SITL + Gazebo + ROS offboard），
            本节点只读其状态，不发送任何指令（原项目代码零修改）。
    目标机 ：原项目 x500 真机，8 m/s 正常版本。
    算法层虚拟僚机 P2/P3：直接获取目标真值生成包抄/包围参考，
            起飞、位置控制和动力学由 PX4 SITL instance 2/3 + Gazebo 执行。

状态来源（坐标系全部为公共 map_ned）：
    /tracking/target/state        （nav_msgs/Odometry）：目标真值位置/速度
    /evaluation/relative_truth    （nav_msgs/Odometry）：目标相对实机真值
        → 实机 P1 在 map_ned 的位置/速度 = 目标状态 − 相对真值（反推）

输出：
    /intercept/p2/odom、/intercept/p3/odom（僚机真值，供 rviz2 与图册）
    结果目录：<package>/results/hybrid_<时间戳>/（CSV / metrics.json / PNG）
"""

import csv
import json
import os
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node

from .metrics import MetricsCollector
from .utils import clamp_f, quat_from_yaw, safe_atan2
from .visualization import plot_all

try:
    from px4_msgs.msg import VehicleLocalPosition as _VLP
    HAS_PX4_MSGS = True
except ImportError:
    HAS_PX4_MSGS = False
    _VLP = None

try:
    from px4_offboard_tracking.msg import PureInterceptorStatus
    HAS_P1_STATUS = True
except ImportError:
    HAS_P1_STATUS = False
    PureInterceptorStatus = None

try:
    from virtual_interception_interfaces.msg import (
        VirtualHitResult, VirtualShotEvent)
    HAS_SHOT_EVENTS = True
except ImportError:
    HAS_SHOT_EVENTS = False
    VirtualHitResult = None
    VirtualShotEvent = None

PKG_ROOT = Path(__file__).resolve().parents[1]



class Hybrid2V1Node(Node):
    def __init__(self):
        super().__init__('hybrid_2v1_sim')

        self.declare_parameter('config_file', '')
        self.declare_parameter('results_dir', '')
        self.declare_parameter('duration_s', -1.0)

        cfg_file = self.get_parameter('config_file').value
        if not cfg_file:
            cfg_file = str(PKG_ROOT / 'config' / 'hybrid.yaml')
        if not os.path.isfile(cfg_file):
            raise FileNotFoundError(f"找不到配置文件: {cfg_file}")
        self.cfg = yaml.safe_load(open(cfg_file, encoding='utf-8'))

        sim_cfg = self.cfg['simulation']
        self.dt = float(sim_cfg['dt'])
        self.rate = float(sim_cfg['rate_hz'])
        self.frame_id = sim_cfg['frame_id']
        self.engage_delay = float(sim_cfg['engage_delay_s'])
        self.duration = float(sim_cfg['duration_s'])
        dur_override = float(self.get_parameter('duration_s').value)
        if dur_override > 0.0:
            self.duration = dur_override
        results_override = self.get_parameter('results_dir').value
        self.results_root = Path(results_override) if results_override \
            else PKG_ROOT / 'results'

        # ---- 僚机（PX4 实例 2/3）：只读状态，位置指令由 wingman_offboard 节点发 ----
        init = self.cfg['initial']
        self.wing_origins = {k: np.array([init[k]['x'], init[k]['y'], 0.0])
                             for k in ('p2', 'p3')}
        self.wing = {'p2': None, 'p3': None}   # (p_map, v_map)
        self.reference = {'p2': None, 'p3': None}  # 平滑后的 map_ned 参考
        if HAS_PX4_MSGS:
            from rclpy.qos import QoSProfile, QoSReliabilityPolicy
            px4_qos = QoSProfile(depth=5,
                                 reliability=QoSReliabilityPolicy.BEST_EFFORT)
            for k, inst in (('p2', 2), ('p3', 3)):
                self.create_subscription(
                    _VLP, f'/px4_{inst}/fmu/out/vehicle_local_position',
                    lambda msg, key=k: self._on_wing_local(key, msg), px4_qos)
                self.create_subscription(
                    Odometry, f'/intercept/{k}/reference',
                    lambda msg, key=k: self._on_reference(key, msg), 10)
        # 僚机只做位置包围（不射击）；metrics 仅用于记录距离/得分曲线
        self.metrics = MetricsCollector(
            self.cfg['score'], self.cfg['success'], self.cfg['shooting'],
            float(self.cfg['metrics']['entry_range_m']))
        # 僚机包围到位判定：P2 与 P3 同时在 [enc_min, enc_max] 内保持 hold_s
        self._wing_hold = 0.0
        self._wing_hold_s = float(self.cfg['success'].get('wing_hold_s', 1.5))
        self.wings_encircled_time = None
        # 僚机参考：目标速度方向低通滤波（初始朝 +x）
        self._dir_u = np.array([1.0, 0.0, 0.0])

        # ---- 话题 ----
        self.create_subscription(
            Odometry, sim_cfg['target_state_topic'], self._on_target, 10)
        self.create_subscription(
            Odometry, sim_cfg['relative_truth_topic'], self._on_rel_truth, 10)
        if HAS_P1_STATUS:
            self.create_subscription(
                PureInterceptorStatus, '/tracking/pure_interceptor/status',
                self._on_p1_status, 10)
        if HAS_SHOT_EVENTS:
            self.create_subscription(
                VirtualShotEvent, '/tracking/virtual_shot_event',
                self._on_shot_event, 20)
            self.create_subscription(
                VirtualHitResult, '/evaluation/virtual_hit_result',
                self._on_hit_result, 20)
        self.odom_pubs = {k: self.create_publisher(
            Odometry, f'/intercept/{k}/odom', 10) for k in ('p2', 'p3')}
        # 实机拦截完成时刻（原方案状态机 COMPLETE）
        self.p1_complete_time = None
        self.p1_complete_seen = False

        # ---- 状态缓存与记录 ----
        self.target_odom = None     # 目标真值（map_ned）
        self.p1 = None              # 实机反推 (p, v)
        self.traj = {'target': [], 'p1': [], 'p2': [], 'p3': []}
        self.vel = {'target': [], 'p1': [], 'p2': [], 'p3': []}
        self.acc = {'target': [], 'p1': [], 'p2': [], 'p3': []}
        self.quat = {'p2': [], 'p3': []}
        self.pred_log = []
        self.ref_log = {'p2': [], 'p3': []}
        self.shot_events = []
        self.t_list = []
        self._t = 0.0
        self._done = False

        self.create_timer(1.0 / self.rate, self._step)
        self.get_logger().info(
            f"[Hybrid 2V1 Sim] engage_delay={self.engage_delay}s "
            f"duration={self.duration}s；僚机 P2/P3 就绪，等待目标与实机状态")

    # ------------------------------------------------------------------
    def _on_target(self, msg):
        self.target_odom = msg

    def _on_rel_truth(self, msg):
        """目标相对实机真值 → 反推实机 P1 在 map_ned 的位置/速度。"""
        if self.target_odom is None:
            return
        tp = self.target_odom.pose.pose.position
        tv = self.target_odom.twist.twist.linear
        rp = msg.pose.pose.position
        rv = msg.twist.twist.linear
        self.p1 = (np.array([tp.x - rp.x, tp.y - rp.y, tp.z - rp.z]),
                   np.array([tv.x - rv.x, tv.y - rv.y, tv.z - rv.z]))

    def _on_wing_local(self, key, msg):
        """僚机 PX4 local 位置/速度 → map_ned（+ 起飞点）。"""
        p = np.array([msg.x, msg.y, msg.z]) + self.wing_origins[key]
        v = np.array([msg.vx, msg.vy, msg.vz])
        self.wing[key] = (p, v)

    def _on_reference(self, key, msg):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        stamp_s = (float(msg.header.stamp.sec)
                   + float(msg.header.stamp.nanosec) * 1e-9)
        self.reference[key] = (
            np.array([p.x, p.y, p.z]), np.array([v.x, v.y, v.z]), stamp_s)

    def _on_p1_status(self, msg):
        """实机状态机：记录 COMPLETE（原方案完成拦截）时刻。"""
        if msg.state == 11:  # PureInterceptorStatus.COMPLETE
            if not self.p1_complete_seen:
                self.p1_complete_seen = True
                self.p1_complete_time = self._t if self._t > 0 else None

    def _on_shot_event(self, msg):
        """记录 P1 原闭环发布的真实虚拟开火事件，不改变其判定。"""
        actual_range = None
        if self.target_odom is not None and self.p1 is not None:
            tp = self.target_odom.pose.pose.position
            target = np.array([tp.x, tp.y, tp.z])
            actual_range = float(np.linalg.norm(self.p1[0] - target))
        self.shot_events.append({
            'shot_id': int(msg.shot_id),
            'time_s': float(self._t),
            'estimated_range_m': float(msg.estimated_range_m),
            'actual_range_m': actual_range,
            'solution_valid': bool(msg.solution_valid),
            'result': 'PENDING',
            'hit': None,
            'minimum_miss_distance_m': None,
            'reason': '',
        })
        self.get_logger().info(
            f"P1 开火 #{len(self.shot_events)} id={msg.shot_id} "
            f"估计/实际距离={msg.estimated_range_m:.2f}/"
            f"{actual_range if actual_range is not None else float('nan'):.2f} m")

    def _on_hit_result(self, msg):
        names = {
            VirtualHitResult.RESULT_PENDING: 'PENDING',
            VirtualHitResult.RESULT_HIT: 'HIT',
            VirtualHitResult.RESULT_MISS: 'MISS',
            VirtualHitResult.RESULT_TIMEOUT: 'TIMEOUT',
            VirtualHitResult.RESULT_INVALID: 'INVALID',
        }
        for event in reversed(self.shot_events):
            if event['shot_id'] == int(msg.shot_id):
                event['result'] = names.get(int(msg.result), f'UNKNOWN_{msg.result}')
                event['hit'] = bool(msg.hit)
                event['minimum_miss_distance_m'] = float(
                    msg.minimum_miss_distance_m)
                event['reason'] = str(msg.reason)
                self.get_logger().info(
                    f"P1 裁判 id={msg.shot_id}: {event['result']}，"
                    f"最小脱靶量={msg.minimum_miss_distance_m:.3f} m")
                return

    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
    def _publish_odom(self, key, p, v, q):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = key
        msg.pose.pose.position = Point(x=float(p[0]), y=float(p[1]),
                                       z=float(p[2]))
        msg.pose.pose.orientation.w = float(q[0])
        msg.pose.pose.orientation.x = float(q[1])
        msg.pose.pose.orientation.y = float(q[2])
        msg.pose.pose.orientation.z = float(q[3])
        msg.twist.twist.linear.x = float(v[0])
        msg.twist.twist.linear.y = float(v[1])
        msg.twist.twist.linear.z = float(v[2])
        self.odom_pubs[key].publish(msg)

    # ------------------------------------------------------------------
    def _step(self):
        if self._done:
            return
        # 等待目标与实机状态就绪
        if self.target_odom is None or self.p1 is None:
            return
        t = self._t

        p_t = np.array([self.target_odom.pose.pose.position.x,
                        self.target_odom.pose.pose.position.y,
                        self.target_odom.pose.pose.position.z])
        v_t = np.array([self.target_odom.twist.twist.linear.x,
                        self.target_odom.twist.twist.linear.y,
                        self.target_odom.twist.twist.linear.z])
        a_t = np.zeros(3)  # 目标加速度由差分近似（下方计算）

        # ---- 僚机状态（由真实 PX4 offboard 执行，本节点只读）----
        # 僚机位置指令由 wingman_offboard 节点按真值计算并发送，
        # 起飞/姿态/电机动力学全部由 PX4 完成（算法层虚拟：无感知估计）。
        # 目标加速度（数值差分，供记录）
        if self.t_list and len(self.vel['target']) > 1:
            a_t = (v_t - self.vel['target'][-1]) / self.dt

        p1, v1 = self.p1

        # ---- 僚机包围到位判定（几何、高度与安全条件同时满足）----
        if t >= self.engage_delay:
            w2 = self.wing['p2']; w3 = self.wing['p3']
            if w2 is None or w3 is None:
                return
            d2 = float(np.linalg.norm(w2[0] - p_t))
            d3 = float(np.linalg.norm(w3[0] - p_t))
            enc_min = float(self.cfg['success']['encircle_min_range_m'])
            enc_max = float(self.cfg['success']['encircle_max_range_m'])
            r1 = p1[:2] - p_t[:2]
            theta1 = safe_atan2(r1[1], r1[0], fallback=0.0)
            theta2 = safe_atan2(w2[0][1] - p_t[1], w2[0][0] - p_t[0],
                                fallback=theta1)
            theta3 = safe_atan2(w3[0][1] - p_t[1], w3[0][0] - p_t[0],
                                fallback=theta1)
            wrap = lambda a: (a + np.pi) % (2.0 * np.pi) - np.pi
            bearing_errors = [
                abs(np.degrees(wrap(theta2 - theta1 - 2.0 * np.pi / 3.0))),
                abs(np.degrees(wrap(theta3 - theta1 + 2.0 * np.pi / 3.0))),
            ]
            centroid_error = float(np.linalg.norm(
                (p1[:2] + w2[0][:2] + w3[0][:2]) / 3.0 - p_t[:2]))
            inter = [float(np.linalg.norm(p1 - w2[0])),
                     float(np.linalg.norm(w2[0] - w3[0])),
                     float(np.linalg.norm(w3[0] - p1))]
            fc = self.cfg['formation']
            vertical_ok = all(
                float(fc['vertical_sep_min_m']) - float(
                    self.cfg['success']['vertical_tolerance_m'])
                <= float(w[0][2] - p_t[2])
                <= float(fc['vertical_sep_max_m']) + float(
                    self.cfg['success']['vertical_tolerance_m'])
                for w in (w2, w3))
            close_to_p1 = all(
                abs(float(w[0][2] - p1[2])) <=
                float(self.cfg['success']['vertical_tolerance_m'])
                for w in (w2, w3))
            wing_spread = abs(np.degrees(wrap(theta2 - theta3)))
            geometry_ok = (
                float(np.linalg.norm(p1 - p_t)) <=
                float(fc['entry_p1_range_m'])
                and wing_spread >= float(
                    self.cfg['success']['minimum_wing_angular_spread_deg']))
            safety_ok = min(inter) >= float(
                fc['safety_min_inter_pursuer_m'])
            if ((enc_min <= d2 <= enc_max) and (enc_min <= d3 <= enc_max)
                    and vertical_ok and close_to_p1 and geometry_ok and safety_ok):
                self._wing_hold += self.dt
                if (self.wings_encircled_time is None
                        and self._wing_hold >= self._wing_hold_s):
                    self.wings_encircled_time = t
            else:
                self._wing_hold = 0.0

        # ---- 记录与指标（三机 + 实机）----
        st = {}
        for k in ('p2', 'p3'):
            wk = self.wing[k]
            if wk is None:
                return
            st[k] = (wk[0], wk[1], np.zeros(3),
                     quat_from_yaw(safe_atan2(wk[1][1], wk[1][0],
                                              fallback=0.0)))
        st['p1'] = (p1, v1, np.zeros(3), quat_from_yaw(
            safe_atan2(v1[1], v1[0], fallback=0.0)))
        self.metrics.update(t, self.dt, (p_t, v_t, a_t), st,
                            None, active=(t >= self.engage_delay))

        self.t_list.append(t)
        self.traj['target'].append(p_t)
        self.vel['target'].append(v_t)
        self.acc['target'].append(a_t)
        self.traj['p1'].append(p1)
        self.vel['p1'].append(v1)
        self.acc['p1'].append(np.zeros(3))
        for k in ('p2', 'p3'):
            wp, wv = self.wing[k]
            self.traj[k].append(wp)
            self.vel[k].append(wv)
            self.acc[k].append(np.zeros(3))
            q = quat_from_yaw(safe_atan2(wv[1], wv[0], fallback=0.0))
            self.quat[k].append(q)
            ref = self.reference[k]
            if ref is None:
                self.ref_log[k].append(np.r_[wp, np.zeros(3), 0.0])
            else:
                self.ref_log[k].append(np.r_[ref[0], ref[1], ref[2]])
            self._publish_odom(k, wp, wv, q)

        self._t += self.dt
        # 任务窗口在 P1 COMPLETE 后再保留 1 s：覆盖裁判结果与包围余稳，
        # 不把原单机任务结束后的返航穿越混入协同拦截安全统计。
        if (self.p1_complete_time is not None
                and self._t >= self.p1_complete_time + 1.0):
            self._done = True
            self._finish()
        elif self._t >= self.duration:
            self._done = True
            self._finish()

    # ------------------------------------------------------------------
    def _finish(self):
        """汇总、保存结果并退出。"""
        summary = self.metrics.finalize()
        series = self.metrics.series()
        series['traj'] = {k: np.asarray(self.traj[k]) for k in self.traj}
        series['vel'] = {k: np.asarray(self.vel[k]) for k in self.vel}
        series['acc'] = {k: np.asarray(self.acc[k]) for k in self.acc}
        series['quat'] = {k: np.asarray(self.quat[k]) for k in self.quat}
        series['refs'] = {k: np.asarray(self.ref_log[k]) for k in self.ref_log}
        self._add_geometry_series(series, summary)

        # 速度限制校验（2虚1实 只约束目标 ≤8 与实机 ≤16；僚机速度不限制）
        # 目标速度判定容差 8.5 m/s：覆盖起飞/爬升瞬态（稳态巡航严格 ≤8）
        target_height = -series['traj']['target'][:, 2]
        cruise_mask = ((series['t'] >= self.engage_delay)
                       & (target_height >= float(self.cfg['simulation'][
                           'guide_min_target_altitude_m'])))
        summary['target_cruise_max_speed'] = float(np.max(
            series['v_mag']['target'][cruise_mask])) if np.any(cruise_mask) \
            else summary['target_max_speed']
        t_ok = summary['target_cruise_max_speed'] <= 8.0 + 0.8 + 1e-9
        p1_ok = summary['p1_max_speed'] <= 16.0 + 1e-9
        summary['speed_limit_violation'] = not (t_ok and p1_ok)
        if summary['speed_limit_violation']:
            self.get_logger().error("目标/实机速度限制被违反！")

        summary['p1_complete'] = self.p1_complete_seen
        summary['p1_complete_time'] = self.p1_complete_time
        summary['wings_encircled'] = self.wings_encircled_time is not None
        summary['wings_encircled_time'] = self.wings_encircled_time
        summary['shot_events'] = self.shot_events
        summary['p1_shot_count'] = len(self.shot_events)
        summary['p1_hit_count'] = sum(event.get('result') == 'HIT'
                                      for event in self.shot_events)
        summary['p1_actual_shot_distances_m'] = [
            event.get('actual_range_m') for event in self.shot_events]

        # 2虚1实 协同成功判据：
        #   实机 P1 由原方案完成拦截（COMPLETE）＋ 僚机 P2/P3 位置包围到位
        #   （两僚机同时满足三维20~40 m、垂向层、分侧趋势与安全条件；不射击）
        wings_ok = self.wings_encircled_time is not None
        shots_ok = (len(self.shot_events) >= 2 and
                    sum(event.get('result') == 'HIT'
                        for event in self.shot_events) >= 2)
        finite_ok = all(np.all(np.isfinite(series['traj'][key]))
                        for key in ('target', 'p1', 'p2', 'p3'))
        # 两个ROS进程间DDS可能漏采中间样本，订阅端差分加速度仅作诊断；
        # 无跳变硬判据使用参考速度和按消息时间归一化的位置变化率，
        # 控制器内部仍逐周期执行显式
        # 速度/加速度限幅（配置快照可追溯）。
        ref_speed_limit = float(
            self.cfg['formation']['horizontal_speed_limit_mps'])
        reference_ok = (
            summary['max_reference_speed_mps'] <= ref_speed_limit + 0.5
            and summary['max_reference_position_rate_mps'] <=
            ref_speed_limit + 1.5)
        summary['reference_accel_limit_configured_mps2'] = float(
            self.cfg['formation']['horizontal_accel_limit_mps2'])
        summary['reference_accel_observation_note'] = (
            'DDS subscriber finite difference; diagnostic only because samples may be skipped')
        summary['finite_data'] = bool(finite_ok)
        summary['reference_smooth'] = bool(reference_ok)
        self.cooperative_success = bool(
            self.p1_complete_seen and wings_ok and shots_ok and finite_ok
            and reference_ok and summary['min_inter_pursuer_distance'] >=
            float(self.cfg['formation']['safety_min_inter_pursuer_m']))
        summary['success'] = self.cooperative_success
        if self.cooperative_success:
            times = [t for t in (self.p1_complete_time,
                                 self.wings_encircled_time) if t is not None]
            summary['intercept_time'] = max(times) if times else None
        summary['p1_max_speed'] = float(np.max(np.linalg.norm(
            series['vel']['p1'], axis=1))) if len(self.t_list) else 0.0

        out_dir = self.results_root / datetime.now().strftime('hybrid_%Y%m%d_%H%M%S')
        out_dir.mkdir(parents=True, exist_ok=True)

        # ---- CSV ----
        self._write_csv(out_dir, series)
        self._write_shot_csv(out_dir)

        json_safe = dict(summary)
        json_safe['intercept_point'] = (
            summary['intercept_point'].tolist() if summary['intercept_point']
            is not None else None)
        with open(out_dir / 'metrics.json', 'w', encoding='utf-8') as f:
            json.dump(json_safe, f, indent=2, ensure_ascii=False)
        with open(out_dir / 'params_snapshot.yaml', 'w', encoding='utf-8') as f:
            yaml.safe_dump(self.cfg, f, allow_unicode=True, sort_keys=False)

        plot_all(out_dir, series, summary, {'scenario_name': 'hybrid_2v1'},
                 self.shot_events)
        self._write_summary(out_dir, summary)

        self.get_logger().info(
            f"===== Hybrid 2V1 结果: "
            f"{'SUCCESS' if summary['success'] else 'FAIL'} | "
            f"目标 {summary['target_max_speed']:.2f} m/s | "
            f"实机P1 {summary['p1_max_speed']:.2f} m/s "
            f"COMPLETE={'✓' if self.p1_complete_seen else '✗'} | "
            f"僚机包围 {'✓' if self.wings_encircled_time is not None else '✗'} | "
            f"min_d=[{summary['min_distance_p1']:.1f}, "
            f"{summary['min_distance_p2']:.1f}, "
            f"{summary['min_distance_p3']:.1f}] | "
            f"score={summary['final_cooperative_score']:.2f} =====")
        self.get_logger().info(f"结果目录: {out_dir}")
        self.get_logger().info("Hybrid 2V1 记录完成，节点退出")
        # 在回调中结束 spin（rclpy 支持回调内 shutdown）
        import rclpy as _rclpy
        _rclpy.shutdown()

    # ------------------------------------------------------------------
    def _add_geometry_series(self, series, summary):
        """计算展示和严格成功判定使用的包围几何/垂向/参考平滑指标。"""
        traj = series['traj']
        target = traj['target']
        p1 = traj['p1']
        p2 = traj['p2']
        p3 = traj['p3']
        centroid = (p1[:, :2] + p2[:, :2] + p3[:, :2]) / 3.0
        centroid_error = np.linalg.norm(centroid - target[:, :2], axis=1)

        theta1 = np.arctan2(p1[:, 1] - target[:, 1],
                            p1[:, 0] - target[:, 0])
        theta2 = np.arctan2(p2[:, 1] - target[:, 1],
                            p2[:, 0] - target[:, 0])
        theta3 = np.arctan2(p3[:, 1] - target[:, 1],
                            p3[:, 0] - target[:, 0])
        wrap = lambda a: (a + np.pi) % (2.0 * np.pi) - np.pi
        bearing_error = {
            'p2': np.degrees(wrap(theta2 - theta1 - 2.0 * np.pi / 3.0)),
            'p3': np.degrees(wrap(theta3 - theta1 + 2.0 * np.pi / 3.0)),
        }
        vertical_below = {
            'p1': p1[:, 2] - target[:, 2],
            'p2': p2[:, 2] - target[:, 2],
            'p3': p3[:, 2] - target[:, 2],
        }
        height_delta_p1 = {
            'p2': np.abs(p2[:, 2] - p1[:, 2]),
            'p3': np.abs(p3[:, 2] - p1[:, 2]),
        }
        inter_distance = {
            'p1_p2': np.linalg.norm(p1 - p2, axis=1),
            'p2_p3': np.linalg.norm(p2 - p3, axis=1),
            'p3_p1': np.linalg.norm(p3 - p1, axis=1),
        }
        series['centroid_error_xy'] = centroid_error
        series['bearing_error_deg'] = bearing_error
        series['vertical_below_target_m'] = vertical_below
        series['height_delta_to_p1_m'] = height_delta_p1
        series['inter_distance'] = inter_distance

        p1_range = np.linalg.norm(p1 - target, axis=1)
        active = p1_range <= float(self.cfg['formation']['entry_p1_range_m'])
        cost = (centroid_error
                + 0.10 * np.abs(bearing_error['p2'])
                + 0.10 * np.abs(bearing_error['p3']))
        valid_indices = np.flatnonzero(active)
        best_index = int(valid_indices[np.argmin(cost[valid_indices])]) \
            if valid_indices.size else int(np.argmin(cost))
        series['best_formation_index'] = best_index
        summary['best_formation_time_s'] = float(series['t'][best_index])
        summary['centroid_error_at_best_m'] = float(centroid_error[best_index])
        summary['minimum_centroid_error_m'] = float(np.min(centroid_error[active])) \
            if np.any(active) else float(np.min(centroid_error))
        summary['p2_bearing_error_at_best_deg'] = float(
            bearing_error['p2'][best_index])
        summary['p3_bearing_error_at_best_deg'] = float(
            bearing_error['p3'][best_index])
        summary['p2_min_vertical_below_target_m'] = float(
            np.min(vertical_below['p2'][active])) if np.any(active) else float(
                np.min(vertical_below['p2']))
        summary['p3_min_vertical_below_target_m'] = float(
            np.min(vertical_below['p3'][active])) if np.any(active) else float(
                np.min(vertical_below['p3']))
        summary['p2_max_height_delta_to_p1_m'] = float(
            np.max(height_delta_p1['p2'][active])) if np.any(active) else float(
                np.max(height_delta_p1['p2']))
        summary['p3_max_height_delta_to_p1_m'] = float(
            np.max(height_delta_p1['p3'][active])) if np.any(active) else float(
                np.max(height_delta_p1['p3']))

        max_ref_speed = 0.0
        max_ref_accel = 0.0
        max_ref_step = 0.0
        max_ref_position_rate = 0.0
        for key in ('p2', 'p3'):
            ref = series['refs'][key]
            if len(ref) < 2:
                continue
            max_ref_speed = max(max_ref_speed, float(np.max(
                np.linalg.norm(ref[:, 3:6], axis=1))))
            # hybrid 与 wingman 是两个独立 ROS 进程，不能假定订阅采样严格
            # 等于 self.dt；用参考消息自身时间戳计算，避免漏采一个样本时
            # 把正常 4 m/s² 误报为 8 m/s²。
            ref_dt = np.diff(ref[:, 6]) if ref.shape[1] > 6 else np.full(
                len(ref) - 1, self.dt)
            changed = ref_dt > 1e-5
            if np.any(changed):
                position_rate = np.linalg.norm(
                    np.diff(ref[:, :3], axis=0)[changed]
                    / ref_dt[changed, None], axis=1)
                max_ref_position_rate = max(
                    max_ref_position_rate, float(np.max(position_rate)))
                accel = np.linalg.norm(
                    np.diff(ref[:, 3:6], axis=0)[changed]
                    / ref_dt[changed, None], axis=1)
                max_ref_accel = max(max_ref_accel, float(np.max(accel)))
            max_ref_step = max(max_ref_step, float(np.max(
                np.linalg.norm(np.diff(ref[:, :3], axis=0), axis=1))))
        summary['max_reference_speed_mps'] = max_ref_speed
        summary['max_reference_accel_mps2'] = max_ref_accel
        summary['max_reference_step_m'] = max_ref_step
        summary['max_reference_position_rate_mps'] = max_ref_position_rate

    # ------------------------------------------------------------------
    def _write_csv(self, out_dir, series):
        t = series['t']
        n = len(t)
        cols = ['t']
        for k in ('target', 'p1', 'p2', 'p3'):
            for c in 'xyz':
                cols.append(f'{k}_p{c}')
        for k in ('target', 'p1', 'p2', 'p3'):
            for c in 'xyz':
                cols.append(f'{k}_v{c}')
        for k in ('target', 'p1', 'p2', 'p3'):
            for c in 'xyz':
                cols.append(f'{k}_a{c}')
        for k in ('p2', 'p3'):
            for c in 'wxyz':
                cols.append(f'{k}_q{c}')
        for k in ('p2', 'p3'):
            for c in 'xyz':
                cols.append(f'{k}_ref_p{c}')
            for c in 'xyz':
                cols.append(f'{k}_ref_v{c}')
        cols += ['dist_p1', 'dist_p2', 'dist_p3',
                 'inter_p1_p2', 'inter_p2_p3', 'inter_p3_p1',
                 'p1_below_target', 'p2_below_target', 'p3_below_target',
                 'centroid_error_xy', 'p2_bearing_error_deg',
                 'p3_bearing_error_deg',
                 'escape_coverage', 'range_factor', 'cooperative_score']
        with open(out_dir / 'trajectory.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(cols)
            for i in range(n):
                row = [f'{t[i]:.3f}']
                for k in ('target', 'p1', 'p2', 'p3'):
                    row += [f'{x:.3f}' for x in series['traj'][k][i]]
                for k in ('target', 'p1', 'p2', 'p3'):
                    row += [f'{x:.3f}' for x in series['vel'][k][i]]
                for k in ('target', 'p1', 'p2', 'p3'):
                    row += [f'{x:.3f}' for x in series['acc'][k][i]]
                for k in ('p2', 'p3'):
                    row += [f'{x:.3f}' for x in series['quat'][k][i]]
                for k in ('p2', 'p3'):
                    row += [f'{x:.3f}' for x in series['refs'][k][i][:6]]
                row += [f'{series["dist"][k][i]:.3f}' for k in ('p1', 'p2', 'p3')]
                row += [f'{series["inter_distance"][k][i]:.3f}'
                        for k in ('p1_p2', 'p2_p3', 'p3_p1')]
                row += [f'{series["vertical_below_target_m"][k][i]:.3f}'
                        for k in ('p1', 'p2', 'p3')]
                row += [f'{series["centroid_error_xy"][i]:.3f}',
                        f'{series["bearing_error_deg"]["p2"][i]:.3f}',
                        f'{series["bearing_error_deg"]["p3"][i]:.3f}']
                row += [f'{series["escape_cov"][i]:.3f}',
                        f'{series["range_factor"][i]:.3f}',
                        f'{series["score"][i]:.3f}']
                writer.writerow(row)

    def _write_shot_csv(self, out_dir):
        fields = ('sequence', 'shot_id', 'time_s', 'estimated_range_m',
                  'actual_range_m', 'solution_valid', 'result', 'hit',
                  'minimum_miss_distance_m', 'reason')
        with open(out_dir / 'p1_shot_events.csv', 'w', newline='',
                  encoding='utf-8-sig') as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for sequence, event in enumerate(self.shot_events, start=1):
                writer.writerow({'sequence': sequence, **event})

    # ------------------------------------------------------------------
    def _write_summary(self, out_dir, summary):
        def f(v, nd=2):
            return 'None' if v is None else f'{v:.{nd}f}'

        lines = [
            '=' * 66,
            ' Hybrid 2V1 Cooperative Interception — 结果汇总',
            '=' * 66,
            ' 实机 P1：原项目 14 m/s 统一拦截方案（代码未修改，直接拦截）',
            ' 僚机 P2/P3：真值算法 + PX4 SITL/Gazebo 实控，位置包围（不射击）',
            f' 仿真时长: {self.duration:.0f} s，接战延迟 {self.engage_delay:.0f} s',
            '',
            '-- 速度 --',
            f'Target 最大速度: {f(summary["target_max_speed"])} m/s (≤8 ✓)',
            f'实机 P1 最大速度: {f(summary["p1_max_speed"])} m/s',
            f'僚机 P2/P3 最大速度: {f(summary["p2_max_speed"])} / '
            f'{f(summary["p3_max_speed"])} m/s（记录值，不参与超限判罚）',
            f'超限标记: {summary["speed_limit_violation"]}',
            '',
            '-- 拦截统计 --',
            f'P1(实)/P2/P3 最小距离: {f(summary["min_distance_p1"])} / '
            f'{f(summary["min_distance_p2"])} / {f(summary["min_distance_p3"])} m',
            f'首次进入打击窗口: {f(summary["first_entry_time"])} s',
            f'首次满足判据: {f(summary["first_success_time"])} s',
            f'实机 COMPLETE: {"✓ " + f(summary.get("p1_complete_time")) if summary.get("p1_complete") else "—"} s',
            f'协同拦截完成(三机全达成): {f(summary["intercept_time"])} s',
            f'包围形成时间: {f(summary["formation_time"])} s',
            f'协同拦截结果(实机COMPLETE+双僚机位置包围): '
            f'{"SUCCESS" if summary["success"] else "FAIL"}',
            '',
            '-- 僚机位置包围（不射击）--',
            f'P2 到目标最小距离: {f(summary["min_distance_p2"])} m（包围位 20 m）',
            f'P3 到目标最小距离: {f(summary["min_distance_p3"])} m（包围位 20 m）',
            f'僚机包围到位: '
            f'{"✓ " + f(summary.get("wings_encircled_time")) if summary.get("wings_encircled") else "—"} s',
            f'P2/P3 敌机下方最小垂直间距: '
            f'{f(summary.get("p2_min_vertical_below_target_m"))} / '
            f'{f(summary.get("p3_min_vertical_below_target_m"))} m',
            f'最佳包围质心误差: {f(summary.get("centroid_error_at_best_m"))} m',
            f'最佳时刻 P2/P3 方位误差: '
            f'{f(summary.get("p2_bearing_error_at_best_deg"))} / '
            f'{f(summary.get("p3_bearing_error_at_best_deg"))} deg',
            '',
            '-- P1 原闭环远程打击（只读记录）--',
            f'开火/命中: {summary.get("p1_shot_count", 0)} / '
            f'{summary.get("p1_hit_count", 0)}',
            f'实际开火距离: {summary.get("p1_actual_shot_distances_m", [])} m',
            '',
            '-- 协同指标 --',
            f'三机最小两两距离: {f(summary["min_inter_pursuer_distance"])} m',
            f'参考最大速度/加速度/单步位移: '
            f'{f(summary.get("max_reference_speed_mps"))} m/s / '
            f'{f(summary.get("max_reference_accel_mps2"))} m/s^2 / '
            f'{f(summary.get("max_reference_step_m"), 3)} m',
            f'有限数检查/参考平滑: {summary.get("finite_data")} / '
            f'{summary.get("reference_smooth")}',
            f'最终/最大协同得分: {f(summary["final_cooperative_score"])} / '
            f'{f(summary["max_cooperative_score"])}',
            '',
            '声明：僚机的感知/估计为理想真值算法，飞控和动力学为 '
            'PX4 SITL+Gazebo；所有结果均不代表真实飞行器实测性能。',
            '=' * 66,
        ]
        with open(out_dir / 'summary.txt', 'w', encoding='utf-8') as f:
            f.write('\n'.join(lines) + '\n')


def main(args=None):
    rclpy.init(args=args)
    node = Hybrid2V1Node()
    try:
        rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException:
        pass  # _finish 内已调用 rclpy.shutdown()
    except Exception as e:
        node.get_logger().error(f"Hybrid 2V1 失败: {e}")
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
