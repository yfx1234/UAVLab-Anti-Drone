# tools/sim —— 仿真运行与分析脚本集

> 2026-10-15 自 WSL 家目录迁入；内部路径已统一指向 `~/interceptor_ws`。
> 结果数据仍在 `~/netgun_logs/`（不入库）。用法均为
> `bash tools/sim/<脚本>.sh [参数]`（在仓库根执行，脚本自带 source）。

## 运行器（跑仿真）

| 脚本 | 用途 |
|---|---|
| `run_aim_smoke.sh <tag> <场景> <速度> <north> <半径> [额外launch参数]` | **单轮全链冒烟**（最常用；输出 ~/netgun_logs/aim_smoke/） |
| `run_matrix_aim4.sh` | 当前配置 40 轮矩阵（终验模板，复制改 OUT 即新矩阵） |
| `run_matrix_aim15hz.sh` | 15Hz 实机口径 40 轮矩阵 |
| `run_matrix_geo2.sh` / `run_matrix_15hz.sh` | 基线矩阵复跑（30/15Hz 对照） |
| `run_matrix_aim.sh`/`aim2`/`aim3`、`run_matrix_full.sh` | 历史轮留档 |
| `run_yaw_ab.sh`、`vis_run.sh`、`spin_bag_run.sh` | A/B 对比、轨迹 bag 采集、SPIN 质量 bag 采集 |
| `rc_gated_test.sh`/`test2`/`rc_exit_test.sh` | rc_gated 门控假飞手验证（§八-9） |
| `recoil_*.sh` | 后坐注入测试（gz-sim 8.14 阻塞期的基础设施验证） |

## 分析器（不跑仿真，读日志/bag）

| 脚本 | 用途 |
|---|---|
| `matrix_summary.py <results.txt>` | 矩阵逐格聚合（触发/命中/miss） |
| `aim_stats.py <矩阵目录>` | AIM 介入统计（进入轮数/开火时延/命中对比几何路径） |
| `scan_badinit.sh` | 坏 boot 扫描（obs 全拒收且 init=0 的轮） |
| `check_headon_aim.sh` / `diag_circle.sh` | 按族查 AIM 介入与状态流 |
| `analyze_spin_bag.py` / `spin_profile.py` | SPIN 漂移分析 / 时间剖面 |
| `analyze_yaw_bag.py` / `bag_yaw.sh` | 安装 A/B 的 u/v 角离线重建 |
| `replay_bearing.py` / `run_replay_bo.sh` | 剥深度重放装置（bearing-only 压力测试） |
| `plot_traj_v2.py` | 轨迹图（中文字体已修复版） |
| `dbg_sample.sh` / `dbg_diag.sh` / `debug_check.sh` | 调试话题采样/诊断/验证 |

## 典型工作流

```bash
# 单轮验证
bash tools/sim/run_aim_smoke.sh tag1 straight 3.0 0.0 20.0
# 矩阵 + 聚合
bash tools/sim/run_matrix_aim4.sh
python3 tools/sim/matrix_summary.py ~/netgun_logs/matrix_aim4/results.txt
python3 tools/sim/aim_stats.py ~/netgun_logs/matrix_aim4
```
