"""仿真主节点（Simulator Node）。

组装完整链路并驱动虚拟仿真：
    Target Generator -> Target Truth -> Cooperative Guidance -> P1/P2/P3 参考
    -> SE(3) Controllers -> Virtual Vehicle Dynamics -> State Feedback -> Metrics

运行方式（见 README）：
    colcon build --packages-select cooperative_intercept_fake_sim
    source install/setup.bash
    ros2 launch cooperative_intercept_fake_sim cooperative_intercept.launch.py \
        scenario:=encirclement

仿真为同步单节点流水线（理想通信假设：真值无延迟、无丢包、协同信息即时共享），
50 Hz 虚拟时钟步进；同时把各机状态发布到 ROS2 topic 供实时查看。
结果写入 <package>/results/YYYYMMDD_HHMMSS/。
"""

import json
import os
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64, Header

from .cooperative_guidance import CooperativeGuidance
from .metrics import MetricsCollector
from .se3_controller import SE3Controller
from .target_generator import TargetGenerator
from .utils import quat_from_yaw, quat_to_R, safe_atan2
from .vehicle_model import VirtualVehicle
from .virtual_shooter import VirtualShooter
from .visualization import plot_all

def _pkg_root():
    """定位 <package> 源码根目录（兼容 symlink-install 与普通安装两种构建方式）。"""
    here = Path(__file__).resolve().parents[1]
    if (here / 'setup.py').exists():
        return here            # symlink-install：模块在源码目录
    ws = here.parents[1].parent  # build/<pkg>/<pkg>/simulator.py -> <ws>/build
    alt = ws / 'src' / here.name
    return alt if alt.exists() else here


PKG_ROOT = _pkg_root()


class InterceptSimNode(Node):
    def __init__(self):
        super().__init__('cooperative_intercept_sim')

        # ---- 启动参数（launch 文件传入）----
        self.declare_parameter('scenario', 'encirclement')
        self.declare_parameter('config_file', '')
        self.declare_parameter('time_scale', 1.0)
        self.declare_parameter('results_dir', '')

        self.scenario_name = self.get_parameter('scenario').value
        cfg_file = self.get_parameter('config_file').value
        self.time_scale = float(self.get_parameter('time_scale').value)
        results_override = self.get_parameter('results_dir').value

        # ---- 载入 YAML 配置 ----
        if not cfg_file:
            cfg_file = str(PKG_ROOT / 'config' / 'simulation.yaml')
        if not os.path.isfile(cfg_file):
            from ament_index_python.packages import get_package_share_directory
            share_cfg = os.path.join(
                get_package_share_directory('cooperative_intercept_fake_sim'),
                'config', 'simulation.yaml')
            if os.path.isfile(share_cfg):
                cfg_file = share_cfg
            else:
                raise FileNotFoundError(f"找不到配置文件: {cfg_file}")
        self.cfg = yaml.safe_load(open(cfg_file, encoding='utf-8'))

        sim_cfg = self.cfg['simulation']
        self.dt = float(sim_cfg['dt'])
        self.frame_id = sim_cfg['frame_id']
        self.stop_on_success = bool(sim_cfg.get('stop_on_success', False))
        self.scenario_cfg = self.cfg['scenarios'][self.scenario_name]
        self.duration = float(self.scenario_cfg['duration_s'])
        self.engage_delay = float(self.scenario_cfg.get('engage_delay_s', 0.0))
        self.results_root = Path(results_override) if results_override \
            else PKG_ROOT / 'results'

        # ---- 构建仿真对象 ----
        init = self.cfg['initial']
        origin = [init['target']['x'], init['target']['y'], init['target']['z']]
        self.target_gen = TargetGenerator(self.cfg['target'],
                                          self.scenario_cfg, origin)
        self.guidance = CooperativeGuidance(self.cfg['roles'], self.dt)
        self.ctrl = {k: SE3Controller(self.cfg['pursuer'])
                     for k in ('p1', 'p2', 'p3')}

        self.vehicles = {}
        for k in ('p1', 'p2', 'p3'):
            ip = init[k]
            # 初始机头朝向目标起点
            yaw = safe_atan2(origin[1] - ip['y'], origin[0] - ip['x'],
                             fallback=0.0)
            self.vehicles[k] = VirtualVehicle(
                k.upper(), self.cfg['pursuer'],
                [ip['x'], ip['y'], ip['z']], yaw)

        # ---- 虚拟预测射击器（体现原项目"枪打击方案"效果）----
        self.shooters = {k: VirtualShooter(self.cfg['shooting'], k, self.dt)
                         for k in ('p1', 'p2', 'p3')}
        self.shot_events = []   # 全部射击/命中事件 (t, key, kind, range)

        self.metrics = MetricsCollector(
            self.cfg['score'], self.cfg['success'], self.cfg['shooting'],
            float(self.cfg['metrics']['entry_range_m']))

        # ---- ROS2 话题发布（供实时查看）----
        self.odom_pubs = {}
        for k in ('target', 'p1', 'p2', 'p3'):
            self.odom_pubs[k] = self.create_publisher(
                Odometry, f'/intercept/{k}/odom', 10)
        self.score_pub = self.create_publisher(
            Float64, '/intercept/cooperative_score', 10)
        self.pred_pub = self.create_publisher(
            PointStamped, '/intercept/p2/predicted_point', 10)

        # ---- 轨迹记录（供 CSV / 绘图）----
        self.traj = {'target': [], 'p1': [], 'p2': [], 'p3': []}
        self.vel = {'target': [], 'p1': [], 'p2': [], 'p3': []}
        self.acc = {'target': [], 'p1': [], 'p2': [], 'p3': []}
        self.quat = {'p1': [], 'p2': [], 'p3': []}
        self.refs_log = {'p1': [], 'p2': [], 'p3': []}
        self.pred_log = []          # 每步的预测截击点（供前视误差统计）
        self.t_list = []

        self.get_logger().info(
            f"[Synthetic Cooperative Interception Sim] scenario={self.scenario_name} "
            f"duration={self.duration:.1f}s dt={self.dt}s time_scale={self.time_scale}")

    # ------------------------------------------------------------------
    def _hold_refs(self, t, p_t):
        """接战延迟期间的悬停参考：原地保持，机头对准目标。"""
        refs = {}
        for k in ('p1', 'p2', 'p3'):
            veh = self.vehicles[k]
            refs[k] = {
                'p_ref': veh.p.copy(),
                'v_ref': np.zeros(3),
                'a_ref': np.zeros(3),
                'yaw_ref': safe_atan2(p_t[1] - veh.p[1], p_t[0] - veh.p[0],
                                      fallback=0.0),
            }
        return refs

    # ------------------------------------------------------------------
    def _publish_odom(self, key, p, v, q, stamp):
        """发布单机真值 Odometry（虚拟仿真中的理想状态估计）。"""
        msg = Odometry()
        msg.header = Header(stamp=stamp, frame_id=self.frame_id)
        msg.child_frame_id = key
        msg.pose.pose.position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
        msg.pose.pose.orientation.w = float(q[0])
        msg.pose.pose.orientation.x = float(q[1])
        msg.pose.pose.orientation.y = float(q[2])
        msg.pose.pose.orientation.z = float(q[3])
        msg.twist.twist.linear.x = float(v[0])
        msg.twist.twist.linear.y = float(v[1])
        msg.twist.twist.linear.z = float(v[2])
        self.odom_pubs[key].publish(msg)

    # ------------------------------------------------------------------
    def run(self):
        """驱动整个仿真循环（同步流水线）。"""
        t = 0.0
        while t <= self.duration + 1e-9:
            wc_t0 = time.monotonic()

            # ---- 1. 目标真值 ----
            p_t, v_t, a_t = self.target_gen.sample(t)

            # ---- 2. 协同制导：三机参考轨迹 + 预测截击点 ----
            # 接战延迟期间：拦截机原地悬停（模拟"发现目标后起飞"的节奏），
            # 延迟结束后才按三机角色进入协同制导。
            pursuer_states = {k: {'p': self.vehicles[k].p}
                              for k in ('p1', 'p2', 'p3')}
            if t >= self.engage_delay:
                refs = self.guidance.compute(t, p_t, v_t, a_t, pursuer_states)
            else:
                refs = self._hold_refs(t, p_t)

            # ---- 3. SE(3) 控制器 + 虚拟动力学（每机独立）----
            for k in ('p1', 'p2', 'p3'):
                veh = self.vehicles[k]
                ref = refs[k]
                cmd = self.ctrl[k].compute(
                    veh.p, veh.v, quat_to_R(veh.q),
                    ref['p_ref'], ref['v_ref'], ref['a_ref'], ref['yaw_ref'])
                veh.step(self.dt, cmd['a_T'], cmd['omega_cmd'])
                self.refs_log[k].append(ref)

            # ---- 4. 虚拟预测射击（枪打击方案，仅接战后启用）----
            if t >= self.engage_delay:
                for k in ('p1', 'p2', 'p3'):
                    veh = self.vehicles[k]
                    evs = self.shooters[k].update(t, veh.p, p_t, v_t)
                    for kind, et, erange in evs:
                        self.shot_events.append((et, k, kind, erange))

            # ---- 5. 指标收集 ----
            st = {k: (self.vehicles[k].p, self.vehicles[k].v,
                      self.vehicles[k].a_applied, self.vehicles[k].q)
                  for k in ('p1', 'p2', 'p3')}
            self.metrics.update(t, self.dt, (p_t, v_t, a_t), st,
                                self.guidance.pred_point, self.shooters,
                                active=(t >= self.engage_delay))
            if self.guidance.pred_point is not None:
                # 记录 (时刻, 预测点)，避免接战延迟期时间轴错配
                self.pred_log.append((t, self.guidance.pred_point.copy()))

            # ---- 5. 记录与发布 ----
            self.t_list.append(t)
            self.traj['target'].append(p_t)
            self.vel['target'].append(v_t)
            self.acc['target'].append(a_t)
            for k in ('p1', 'p2', 'p3'):
                veh = self.vehicles[k]
                self.traj[k].append(veh.p)
                self.vel[k].append(veh.v)
                self.acc[k].append(veh.a_applied)
                self.quat[k].append(veh.q.copy())

            stamp = rclpy.clock.ROSClock().now().to_msg()
            # 目标姿态由其速度航向合成（理想真值）
            q_t = quat_from_yaw(safe_atan2(v_t[1], v_t[0], fallback=0.0))
            self._publish_odom('target', p_t, v_t, q_t, stamp)
            for k in ('p1', 'p2', 'p3'):
                veh = self.vehicles[k]
                self._publish_odom(k, veh.p, veh.v, veh.q, stamp)
            sc = Float64()
            sc.data = float(self.metrics.score[-1])
            self.score_pub.publish(sc)
            if self.guidance.pred_point is not None:
                ps = PointStamped()
                ps.header = Header(stamp=stamp, frame_id=self.frame_id)
                pp = self.guidance.pred_point
                ps.point = Point(x=float(pp[0]), y=float(pp[1]), z=float(pp[2]))
                self.pred_pub.publish(ps)

            t += self.dt
            # 墙钟节流：time_scale=0 时不等待（尽可能快）
            if self.time_scale > 0.0:
                wall = self.dt / self.time_scale - (time.monotonic() - wc_t0)
                if wall > 0:
                    time.sleep(wall)

            if self.stop_on_success and self.metrics.success:
                self.get_logger().info(
                    f"拦截成功（t={self.metrics.stable_success_time:.2f}s），提前停止")
                break

        self._finish()

    # ------------------------------------------------------------------
    def _finish(self):
        """汇总指标、保存结果与图片。"""
        summary = self.metrics.finalize()
        series = self.metrics.series()
        series['traj'] = {k: np.asarray(self.traj[k]) for k in self.traj}
        series['vel'] = {k: np.asarray(self.vel[k]) for k in self.vel}
        series['acc'] = {k: np.asarray(self.acc[k]) for k in self.acc}
        series['quat'] = {k: np.asarray(self.quat[k]) for k in self.quat}
        series['refs'] = self.refs_log

        # 速度限制硬校验：超限即报错（不允许通过提高速度上限获得拦截）
        if summary['speed_limit_violation']:
            self.get_logger().error(
                "速度限制被违反！请检查仿真实现（不允许提高速度上限获得拦截结果）")

        # ---- 预测截击点前视误差（离线重算，比"点到预测点距离"更有信息量）----
        # 每个 t 时刻的预测点与目标在 t+pred_horizon 时刻的真实位置的误差：
        # 高速巡航时误差≈0，机动段误差反映短时恒速预测模型的有效性。
        horizon = float(self.cfg['roles']['p2']['pred_horizon_s'])
        fwd_err = []
        for t_i, pred in self.pred_log:
            p_future = self.target_gen.sample(t_i + horizon)[0]
            fwd_err.append(float(np.linalg.norm(p_future - pred)))
        if fwd_err:
            summary['prediction_error_mean'] = float(np.mean(fwd_err))
            if summary['intercept_time'] is not None:
                idx = int(np.searchsorted([p[0] for p in self.pred_log],
                                          summary['intercept_time']))
                idx = min(idx, len(fwd_err) - 1)
                summary['prediction_error_at_success'] = float(fwd_err[idx])

        out_dir = self.results_root / datetime.now().strftime('%Y%m%d_%H%M%S')
        out_dir.mkdir(parents=True, exist_ok=True)

        # ---- CSV：完整轨迹 ----
        self._write_csv(out_dir, series)

        # ---- 虚拟射击统计（枪打击方案）----
        shooting_stats = {}
        for k in ('p1', 'p2', 'p3'):
            sh = self.shooters[k]
            shooting_stats[k] = {
                'shots': sh.shot_count,
                'hits': sh.hit_count,
                'first_hit_time': sh.first_hit_time,
                'complete_time': sh.complete_time,
                'complete': sh.complete,
            }
        summary['shooting'] = shooting_stats

        # ---- metrics.json / 参数快照 / summary.txt ----
        json_safe = dict(summary)
        json_safe['intercept_point'] = (
            summary['intercept_point'].tolist() if summary['intercept_point']
            is not None else None)
        json_safe['scenario'] = self.scenario_name
        json_safe['duration_s'] = self.duration
        json_safe['shot_events'] = self.shot_events
        with open(out_dir / 'metrics.json', 'w', encoding='utf-8') as f:
            json.dump(json_safe, f, indent=2, ensure_ascii=False)
        with open(out_dir / 'params_snapshot.yaml', 'w', encoding='utf-8') as f:
            yaml.safe_dump(self.cfg, f, allow_unicode=True, sort_keys=False)

        # ---- 图片 ----
        plot_all(out_dir, series, summary, {'scenario_name': self.scenario_name},
                 self.shot_events)

        self._write_summary(out_dir, summary)
        self._print_summary(summary, out_dir)
        self.get_logger().info(f"结果已保存到: {out_dir}")

    # ------------------------------------------------------------------
    def _write_csv(self, out_dir, series):
        """按时间步写出全部状态与参考量。"""
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
        for k in ('p1', 'p2', 'p3'):
            for c in 'wxyz':
                cols.append(f'{k}_q{c}')
        for k in ('p1', 'p2', 'p3'):
            for c in 'xyz':
                cols.append(f'r{k}_p{c}')
        cols += ['dist_p1', 'dist_p2', 'dist_p3',
                 'escape_coverage', 'range_factor', 'cooperative_score',
                 'pred_error', 'max_forward_gap_deg',
                 'hits_p1', 'hits_p2', 'hits_p3']

        rows = []
        quats = series['quat']
        for i in range(n):
            row = [f'{t[i]:.3f}']
            for k in ('target', 'p1', 'p2', 'p3'):
                row += [f'{x:.3f}' for x in series['traj'][k][i]]
            for k in ('target', 'p1', 'p2', 'p3'):
                row += [f'{x:.3f}' for x in series['vel'][k][i]]
            for k in ('target', 'p1', 'p2', 'p3'):
                row += [f'{x:.3f}' for x in series['acc'][k][i]]
            for k in ('p1', 'p2', 'p3'):
                row += [f'{x:.3f}' for x in quats[k][i]]
            for k in ('p1', 'p2', 'p3'):
                ref = series['refs'][k][i]
                row += [f'{x:.3f}' for x in ref['p_ref']]
            row += [f'{series["dist"][k][i]:.3f}' for k in ('p1', 'p2', 'p3')]
            row += [f'{series["escape_cov"][i]:.3f}',
                    f'{series["range_factor"][i]:.3f}',
                    f'{series["score"][i]:.3f}',
                    f'{series["pred_error"][i]:.3f}',
                    f'{series["max_forward_gap"][i]:.3f}']
            # 命中累计（由 hit 事件重放，保证与时间轴对齐）
            hits = {'p1': 0, 'p2': 0, 'p3': 0}
            for (et, key, kind, _er) in self.shot_events:
                if kind == 'hit' and et <= t[i]:
                    hits[key] += 1
            row += [f'{hits[k]}' for k in ('p1', 'p2', 'p3')]
            rows.append(row)

        import csv
        with open(out_dir / 'trajectory.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(cols)
            writer.writerows(rows)

    # ------------------------------------------------------------------
    def _write_summary(self, out_dir, summary):
        """人类可读的统计报告。"""
        def f(v, nd=2):
            return 'None' if v is None else f'{v:.{nd}f}'

        shooting = summary.get('shooting', {})

        lines = [
            '=' * 68,
            ' Synthetic Cooperative Interception Simulation — 结果汇总',
            '=' * 68,
            f'场景      : {self.scenario_name}  '
            f'（{self.scenario_cfg["description"]}）',
            f'仿真时长  : {self.duration:.1f} s，积分步长 {self.dt} s',
            '',
            '-- 速度限制（硬约束校验）--',
            f'Target 最大速度 : {f(summary["target_max_speed"])} m/s  (上限 8.0)',
            f'P1 最大速度     : {f(summary["p1_max_speed"])} m/s  (上限 16.0)',
            f'P2 最大速度     : {f(summary["p2_max_speed"])} m/s  (上限 16.0)',
            f'P3 最大速度     : {f(summary["p3_max_speed"])} m/s  (上限 16.0)',
            f'超限标记        : {summary["speed_limit_violation"]}',
            '',
            '-- 加速度 --',
            f'Target 最大加速度      : {f(summary["target_max_accel"])} m/s^2',
            f'P1/P2/P3 最大加速度    : {f(summary["p1_max_accel"])} / '
            f'{f(summary["p2_max_accel"])} / {f(summary["p3_max_accel"])} m/s^2',
            '',
            '-- 拦截统计 --',
            f'P1 最小距离: {f(summary["min_distance_p1"])} m',
            f'P2 最小距离: {f(summary["min_distance_p2"])} m',
            f'P3 最小距离: {f(summary["min_distance_p3"])} m',
            f'首次进入有效拦截范围: {f(summary["first_entry_time"])} s',
            f'首次满足拦截判据    : {f(summary["first_success_time"])} s',
            f'拦截稳定形成时间    : {f(summary["intercept_time"])} s',
            f'包围形成时间        : {f(summary["formation_time"])} s',
            f'协同拦截结果        : {"SUCCESS" if summary["success"] else "FAIL"}',
            '',
            '-- 虚拟预测射击（枪打击方案：400 m/s 弹丸、双命中完成）--',
            f'P1 射击/命中/完成: {shooting["p1"]["shots"]} / '
            f'{shooting["p1"]["hits"]} / '
            f'{"✓ " + f(shooting["p1"]["complete_time"]) if shooting["p1"]["complete"] else "—"} s',
            f'P2 射击/命中/完成: {shooting["p2"]["shots"]} / '
            f'{shooting["p2"]["hits"]} / '
            f'{"✓ " + f(shooting["p2"]["complete_time"]) if shooting["p2"]["complete"] else "—"} s',
            f'P3 射击/命中/完成: {shooting["p3"]["shots"]} / '
            f'{shooting["p3"]["hits"]} / '
            f'{"✓ " + f(shooting["p3"]["complete_time"]) if shooting["p3"]["complete"] else "—"} s',
            '',
            '-- 协同指标 --',
            f'三机最小两两距离  : {f(summary["min_inter_pursuer_distance"])} m '
            f'@ {f(summary["min_inter_pursuer_time"])} s',
            f'预测截击点平均误差: {f(summary["prediction_error_mean"])} m',
            f'拦截时刻预测点误差: {f(summary["prediction_error_at_success"])} m',
            f'最终协同得分      : {f(summary["final_cooperative_score"])}',
            f'最大协同得分      : {f(summary["max_cooperative_score"])}',
            '',
            '注意：本结果来自简化虚拟仿真（理想状态、理想通信、简化动力学），',
            '不代表真实飞行器实测性能。',
            '=' * 68,
        ]
        with open(out_dir / 'summary.txt', 'w', encoding='utf-8') as f:
            f.write('\n'.join(lines) + '\n')

    # ------------------------------------------------------------------
    def _print_summary(self, summary, out_dir):
        """终端打印关键指标。"""
        sh = summary.get('shooting', {})
        sh_line = ' | '.join(
            f"{k}:{sh[k]['hits']}/{sh[k]['shots']}"
            + ('✓' if sh[k]['complete'] else '') for k in ('p1', 'p2', 'p3'))
        self.get_logger().info(
            f"===== 结果: {'SUCCESS' if summary['success'] else 'FAIL'} | "
            f"T_max={summary['target_max_speed']:.2f} | "
            f"P1={summary['p1_max_speed']:.2f} "
            f"P2={summary['p2_max_speed']:.2f} "
            f"P3={summary['p3_max_speed']:.2f} | "
            f"拦截时间={summary['intercept_time']} | "
            f"min_d=[{summary['min_distance_p1']:.2f}, "
            f"{summary['min_distance_p2']:.2f}, "
            f"{summary['min_distance_p3']:.2f}] | "
            f"包围形成={summary['formation_time']} | "
            f"命中(P1/P2/P3)={sh_line} | "
            f"score={summary['final_cooperative_score']:.3f} =====")
        self.get_logger().info(f"结果目录: {out_dir}")


def main(args=None):
    rclpy.init(args=args)
    node = InterceptSimNode()
    try:
        node.run()
    except Exception as e:
        node.get_logger().error(f"仿真失败: {e}")
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
