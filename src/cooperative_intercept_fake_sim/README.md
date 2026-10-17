# cooperative_intercept_fake_sim

**Synthetic Cooperative Interception Simulation（3 对 1 多无人机协同拦截虚拟仿真）**

> ⚠️ 本包是**纯虚拟仿真（synthetic simulation）**：理想状态真值、理想通信、简化刚体动力学。
> 所有结果仅用于算法演示与教学展示，**不代表真实飞行器实测性能**。
>
> 包内的 `hybrid_2v1` 是独立混合模式：P2/P3 仅在感知/估计算法层使用真值，
> 起飞、Offboard 控制和动力学由 PX4 SITL + Gazebo 执行，见 §6.1。

在 ROS 2 Humble 中实现 3 架拦截无人机（Pursuer）协同拦截 1 架高速目标无人机（Target）的
完整虚拟仿真：目标真值轨迹 → 协同制导 → 简化 SE(3) 控制器 → 虚拟刚体动力学 →
指标统计与可视化。共 4 种典型场景，可一键切换运行。

---

## 1. 仿真用途

- 演示"3 对 1 协同拦截"的效果：三架拦截机分工（追击 / 预测截击 / 封堵）而不是一起追同一个点；
- 演示高速目标（**8 m/s**）条件下、拦截机（统一 **16 m/s**）通过协同几何缩短目标逃逸空间；
- 演示一条完整的控制链路：**上层协同制导 → 参考轨迹 → SE(3) 控制器 → 虚拟动力学**；
- 输出可重复的轨迹图、距离/速度曲线、协同指标与统计结果，便于展示与对比。

## 2. 架构

```
Target Generator（目标轨迹生成器，4 种场景）
      ↓ 真值 (p, v, a)
Cooperative Guidance（协同制导：P1/P2/P3 分工计算参考轨迹 + 预测截击点）
      ↓ 每机参考 (p_ref, v_ref, a_ref, yaw_ref)
SE(3) Controllers（简化几何控制器：位置环 PD+前馈 → 期望推力/姿态 → 机体角速率指令）
      ↓ (a_T, omega_cmd)
Virtual Vehicle Dynamics（虚拟刚体动力学：推力饱和 + 姿态一阶跟踪 + 四元数积分）
      ↓ (p, v, q, omega)
State Feedback → Metrics（协同指标）/ Visualization（PNG）/ CSV / JSON
```

| 模块 | 文件 | 职责 |
|---|---|---|
| 仿真主节点 | `simulator.py` | 50 Hz 同步流水线，组装全部模块，发布话题，保存结果 |
| 目标生成器 | `target_generator.py` | 弧长参数化的解析目标轨迹（速度严格 ≤ 8 m/s） |
| 协同制导 | `cooperative_guidance.py` | 三机分工与参考轨迹计算 |
| SE(3) 控制器 | `se3_controller.py` | 简化几何 SE(3) 控制（PD + 推力 + 姿态误差） |
| 虚拟动力学 | `vehicle_model.py` | 刚体积分：速度上限硬保护、推力饱和、姿态一阶响应 |
| 指标 | `metrics.py` | 协同得分、拦截判据、全部统计量 |
| 可视化 | `visualization.py` | 基础 5 张 PNG；PX4 hybrid 额外输出两两间距安全图 |
| 配置 | `config/simulation.yaml` | 全部可调参数 |
| 启动 | `launch/cooperative_intercept.launch.py` | 场景/参数/倍速/结果目录切换 |

**坐标系约定**（与原工作空间 `px4_offboard_tracking` 一致）：NED（`map_ned`），
x 前 / y 右 / z 下，重力 `+g` 沿 z 向下；旋转矩阵 `R` 为 body→NED 方向余弦阵；
四元数 Hamilton 约定 `[w,x,y,z]`。仿真为同步单节点流水线——对应"理想通信
（无延迟、无丢包、协同信息即时共享）"假设。

**实时话题**（供 rviz2 / ros2 topic echo 查看）：

```
/intercept/target/odom           # 目标真值（nav_msgs/Odometry, frame map_ned）
/intercept/p1/odom  p2  p3       # 三架拦截机真值
/intercept/cooperative_score     # 协同得分（std_msgs/Float64）
/intercept/p2/predicted_point    # P2 预测截击点（PointStamped）
```

## 3. 三机职责（20 m 等边包围圈）

三机参考点的统一框架：`u` = 目标速度单位方向（低通滤波），`n` = `u` 的水平左法向。
包围圈设计：**等边三角形、半径 20 m**（对应原项目 15~35 m 理想打击窗口、首选 25 m 附近），
三机方位角间隔 120°、两两间距约 34.6 m；三机全部保持在该包围圈上对目标实施虚拟射击。

| 角色 | 参考点公式 | 职责与效果 |
|---|---|---|
| **P1 尾追机** | `p_t − u·20.0`（正后方 20 m，z 高 1 m） | 直接追击目标当前位置，占据目标正后方 20 m 尾追位持续施压 |
| **P2 预测截击机** | 远：`p_t + u·(v·2.0 s)`（前方 16 m 预测点）<br>近：`p_t + u·10.0 + n·17.32`（左前 20 m）<br>按与目标距离混合切换 | 提前 2 s 切向目标未来轨迹，形成明显"截击"弧线；收拢后占据左前 20 m 包围位 |
| **P3 封堵机** | `p_t + u·10.0 + n·(−17.32)`（右前 20 m） | 与 P2 对称占据右前 20 m，封堵目标另一侧逃逸方向，构成前向夹击 |

三机高度各不相同（z = −4.0 / −6.5 / −6.5 m），保证三维分离，避免互相碰撞。
所有偏移量均可通过 YAML 参数调整。

## 3.1 虚拟预测射击（体现原项目"枪打击方案"效果）

三机在 15~35 m 打击窗口内以 **400 m/s** 虚拟弹丸进行**预测射击**（先导角瞄准目标
未来位置，弹丸飞行时间 < 0.1 s），命中半径 0.4 m；每机需完成 **2 次命中**（两次打击
方案），命中后 3 s 冷却。纯几何虚拟判定（理想弹道、无随机散布），仅用于演示
"预测射击 + 双命中"的打击方案效果。命中时刻在三维图与距离图上以黑色 × 标记。

## 4. 场景（`scenario:=` 参数）

| 场景 | 目标轨迹 | 演示重点 |
|---|---|---|
| `straight` | 加速到 8 m/s 后保持高速直线逃逸 | 三机追赶 + P2 提前切入 + P3 侧向封堵 |
| `s_curve` | 8 m/s 高速 S 形机动（平滑凸包交替） | 预测截击点随目标横向摆动、三机重新分配位置 |
| `turn` | 高速直线后 120° 大半径转弯 | 目标转向后三机按新运动方向重新形成包夹 |
| `encirclement` | 直线 + S 形 + 90° 转弯 + 长直线收尾（推荐） | 三机从不同方向逐渐形成完整包围并保持 |

目标轨迹由**弧长参数化**构造：速度剖面（加速/巡航/减速，raised-cosine 平滑）与
路径几何（直线/S 形凸包/圆弧转弯）严格解析，巡航段速度**精确等于 8 m/s**。

## 5. 关键参数（config/simulation.yaml）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `target.max_speed` | 8.0 m/s | 目标速度上限（硬约束） |
| `pursuer.max_speed` | 16.0 m/s | **三机共用**速度上限（硬约束，不允许单机提速） |
| `pursuer.max_thrust_accel` | 20.0 m/s² | 推力方向加速度上限（约 2g，避免水平冲刺时掉高） |
| `pursuer.max_command_accel` | 12.0 m/s² | 指令加速度幅值上限（平滑性限制） |
| `pursuer.max_tilt_rad` | 0.7 rad | 最大倾斜角（约 40°，限水平机动能力） |
| `pursuer.max_body_rate` | 3.0 rad/s | 机体角速率上限（沿用原项目量级） |
| `pursuer.attitude_tau` | 0.35 s | 姿态一阶跟踪时间常数（沿用原项目量级） |
| `pursuer.se3_*_gain` | 2.0 / 1.5 / 2.5 | 位置环 / 速度环 / 姿态环增益 |
| `roles.p2.pred_horizon_s` | 2.0 s | P2 预测时域（8 m/s 时 = 前方 16 m） |
| `roles.p1.behind_offset_m` | 20.0 m | 包围圈半径（P1 尾追位） |
| `roles.p2/p3.forward/lateral` | 10 / ±17.32 m | P2 左前 / P3 右前 20 m 包围位 |
| `shooting.projectile_speed` | 400 m/s | 虚拟弹丸速度（预测射击先导角） |
| `shooting.minimum/maximum_range` | 15 / 35 m | 打击窗口（对应原项目理想窗口） |
| `shooting.hit_radius_m` | 0.4 m | 命中半径 |
| `shooting.required_hits` | 2 | 每机双命中完成（两次打击方案） |
| `score.*` | — | 协同得分权重与阈值（见 §7） |
| `success.*` | — | 拦截成功判据（见 §7） |
| `scenarios.*.engage_delay_s` | 14~16 s | 目标先高速飞行，拦截机延迟起飞（演示节奏） |
| `scenarios.*.duration_s` | 38~54 s | 场景时长 |

## 6. 运行方法

```bash
cd ~/UAV_ws
colcon build --packages-select cooperative_intercept_fake_sim --symlink-install
source install/setup.bash

# 实时模式（默认 encirclement 场景，建议先看这个）
ros2 launch cooperative_intercept_fake_sim cooperative_intercept.launch.py

# 切换场景
ros2 launch cooperative_intercept_fake_sim cooperative_intercept.launch.py scenario:=straight
ros2 launch cooperative_intercept_fake_sim cooperative_intercept.launch.py scenario:=s_curve
ros2 launch cooperative_intercept_fake_sim cooperative_intercept.launch.py scenario:=turn
ros2 launch cooperative_intercept_fake_sim cooperative_intercept.launch.py scenario:=encirclement
```

launch 可选参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `scenario` | `encirclement` | 场景名 |
| `config` | 包内 `config/simulation.yaml` | 自定义参数文件 |
| `time_scale` | `1.0` | 墙钟加速比：`1.0` 实时，`10.0` 十倍速，`0.0` 尽可能快 |
| `results_dir` | `<package>/results/时间戳/` | 结果输出目录 |

一键运行全部场景（快速验证）：

```bash
bash ~/UAV_ws/src/cooperative_intercept_fake_sim/scripts/run_all_scenarios.sh
```

> 注：`--symlink-install` 非必需，普通 `colcon build` 亦可；结果目录与默认配置在
> 两种安装方式下都会正确回退到源码目录。

### 6.1 PX4 僚机混合模式与自动循环

5 m/s 直线迎头是当前已确认的平滑基线。P1 完全复用原
`px4_offboard_tracking` 闭环（零修改）；P2/P3 直接用目标真值生成参考，
但由独立 PX4 instance 2/3 执行。控制和CSV保持NED；所有正式图的垂向
统一显示 `h=-z_NED`。P2/P3在P1进入35 m后，以P1实时方位为基准占据
±120°包围趋势，水平半径18–25 m；NED-z贴近P1并钳在敌机下方15–25 m。
参考点有一阶平滑、速度/加速度限幅、P1方位角速率限制与2 s最近接近预测避让。

```bash
cd ~/UAV_ws && source install/setup.bash

# 单轮平滑基线
HEAD_ON_MODE=true TARGET_SPEED_MPS=5.0 \
  bash src/cooperative_intercept_fake_sim/scripts/run_hybrid_2v1.sh straight

# 默认3轮：运行→清理→解析→平滑门槛→汇总→优选图册
python3 src/cooperative_intercept_fake_sim/scripts/run_hybrid_batch.py \
  --repeats 3 --target-speeds 5.0 --scenarios straight --head-on

# 扩展速度矩阵（尚需逐批验证）
python3 src/cooperative_intercept_fake_sim/scripts/run_hybrid_batch.py \
  --repeats 3 --target-speeds 5.0,7.0,8.0 --scenarios straight --head-on
```

自动脚本支持 `--resume` / `--rerun-failed` / `--dry-run`、
`--max-infra-retries` 和
`--curate-existing <results...>`。每轮增量保存 `records.json`、
`批次汇总.csv`/`批次汇总.md`；平滑门槛默认要求 P2/P3 全程距目标不小于
15 m，三机最小两两间距不小于 15 m。通过案例会按质量排序复制到
`优选案例/`，生成带 `案例01/案例02/...` 和 `01–08` 图号的 HTML/Markdown 图册。

## 7. 输出结果

每次运行自动建立独立目录（不覆盖历史结果）：

```
results/<YYYYMMDD_HHMMSS>/
├── trajectory.csv        # 全轨迹：位置/速度/加速度/姿态四元数/参考点/距离/协同指标（50 Hz）
├── metrics.json          # 全部统计量（机器可读）
├── params_snapshot.yaml  # 本次运行的参数快照
├── summary.txt           # 人类可读统计报告
├── p1_shot_events.csv    # P1原闭环开火与独立裁判结果
├── 01_三机三维轨迹_高度向上.png
├── 02_XY平面等边三角形包围.png
├── 03_高度曲线与敌机垂直间距.png
├── 04_P1敌机距离与开火事件.png
├── 05_P2P3敌机距离.png
├── 06_三架拦截机两两安全距离.png
├── 07_包围质心误差与方位角误差.png
└── 08_关键时刻三机包围快照.png
```

**协同得分定义**（每步计算，随时间记录）：

```
escape_coverage = 1 − (目标速度方向 ±90° 半球内最大方位角空隙) / 180°
range_factor    = clamp(1 − |三机平均距离 − 20 m| / 8 m, 0, 1)   # 以 20 m 包围圈为参考
cooperative_score = 0.6 × escape_coverage + 0.4 × range_factor
```

目标低速/停止时改用全向覆盖（停止的目标任何方向都可逃逸；等边包围时三空隙各 120°，
全向覆盖 = 0.67，score ≈ 0.80）。

**拦截成功判据**（协同式 + 枪打击方案）：

```
P1原闭环完成2次独立裁判HIT；P2/P3在敌机下方且接近P1高度层，
三维距离20~40 m、分居不同侧形成包围趋势并保持1.0 s；
任务窗口内任意拦截机间距≥15 m，且无NaN和参考跳变 → SUCCESS
```

P1由原闭环控制，因此质心误差与相对±120°方位误差只用于绘图和质量排序，
不作为硬门槛；不会为了规则三角形修改P1。

**主要统计量**（summary.txt / metrics.json）：

- Target 与 P1/P2/P3 的最大速度与最大加速度（超限即标记为仿真实现错误）
- 三机对目标的最小距离（约 20 m 包围圈半径）；首次进入打击窗口时间；
  首次/稳定拦截时间；包围形成时间
- 三机之间最小两两距离（防碰撞统计）
- **虚拟射击统计**：每机射击/命中次数、首次命中与双命中完成时间、命中事件时间线
- 预测截击点前视误差（t 时刻预测点 vs 目标在 t+2 s 的真实位置，均值与拦截时刻值）
- 最终/最大协同得分；协同拦截成功/失败

## 8. 仿真边界（重要声明）

本仿真为**纯虚拟演示**，与 PX4 / Gazebo / UE / 真实飞行器动力学**不一致**。主要简化：

1. **理想状态估计**：使用目标真值，无测量噪声/延迟/遮挡。
2. **理想通信**：单节点同步流水线，协同信息即时共享、无丢包。
3. **简化动力学**：平移为受推力/倾斜/速度上限约束的质点积分；
   姿态为一阶角速率跟踪 + 四元数运动学；无电机/气动模型。
4. **人工目标轨迹**：目标按解析路径飞行，不会对拦截做回避机动。
5. **人工协同策略**：三机分工由解析参考点公式决定，非最优规划。
6. **指标与成功判据为演示定义**：非真实交战评估标准。

因此：**所有轨迹图、指标与统计仅代表本虚拟仿真的行为，不代表真实飞行器实测性能。**

---

## 快速验证结果（默认参数，encirclement 场景，20 m 包围圈 + 虚拟射击）

| 指标 | 数值 |
|---|---|
| Target 最大速度 | 8.00 m/s（≤ 8 上限 ✓） |
| P1 / P2 / P3 最大速度 | 16.00 / 16.00 / 16.00 m/s（≤ 16 上限 ✓） |
| 拦截稳定形成时间 | 31.94 s |
| P1 / P2 / P3 最小距离 | 18.92 / 15.63 / 17.65 m（20 m 包围圈 ✓） |
| 虚拟射击（P1/P2/P3 命中） | **2/2 · 2/2 · 2/2**（双命中完成 ✓） |
| 三机完成双命中时间 | 32.36 / 30.50 / 29.86 s |
| 包围形成时间 | 34.68 s |
| 三机最小两两距离 | 8.07 m（无碰撞 ✓） |
| 预测截击点平均误差 | 0.80 m |
| 最终 / 最大协同得分 | 0.80 / 0.80 |
| 协同拦截结果 | **SUCCESS** |

四个场景（straight / s_curve / turn / encirclement）全部 SUCCESS：
三机最小距离 14.7~19.3 m（均落在 20 m 包围圈附近）、三机全部双命中完成、
速度限制全部满足。
