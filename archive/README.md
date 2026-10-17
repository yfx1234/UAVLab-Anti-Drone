# archive/ — 仓库归档区

2026-10-07 结构整理建立的归档区，收纳**已退出维护但需要保留实体**的历史文件。
归档区纳入 Git 跟踪、不参与 colcon 构建；`tools/repository_audit.py`
对活树（`archive/` 以外）单独统计，归档区只计入汇总行。

归档只做搬运，**内容一字未改**。目录内保留文件在原包中的相对路径，
恢复时按下方对应关系移回即可，或直接
`git checkout 96b44f9 -- <原路径>` 从初始导入提交取回。

## snapshots/ — 带时间戳/修复标记的历史快照（27 个）

来源：各包内的 `.bak_*`、`.prediction_before_*`、`.orig` 文件及乱码备份文档。
文件名后缀即当时的修复语境（如 `heightfix`、`holdfix`、`radius`、`p1fix`、
`prediction_before`），是排查现场问题时的对照材料。Git 初始导入已含全部内容。

| 原位置（ros2_ws/src/ 下） | 快照数 |
|---|---|
| `falcon_contact/`（含根目录乱码备份文档） | 9 |
| `falcon_intercept/` | 13 |
| `falcon_intercept_deepstream/` | 3 |
| `offboard_route_offset/`（`CMakeLists.txt.orig`） | 1 |
| `px4_offboard_tracking/`（`.bak_p1fix_20260905`，P1 修复前快照） | 1 |

## dead_code/ — 全仓库零引用的死文件（9 个）

移入前逐项核实过：全仓库（含 launch / 脚本 / 文档 / 构建声明）无引用，
且不被对应包的 CMakeLists / setup.py 安装或编译。

| 原路径（ros2_ws/src/ 下） | 判定依据 |
|---|---|
| `falcon_intercept/falcon_intercept/fake_target_publisher.py` | 仅 setup.py 注册入口，无 launch/脚本引用；入口已同步移除 |
| `falcon_intercept/falcon_intercept/logger_utils.py` | 全仓库无 import |
| `falcon_intercept/launch/perception_v0.launch.py` | 全仓库零引用 |
| `falcon_intercept/launch/decision_v0.launch.py` | 全仓库零引用（`full_v0`/`real_virtual_target_v0` 未动，仍引用 config 版参数） |
| `falcon_intercept_deepstream/src/deepstream_three_camera_v0.launch.py` | src/ 内旧副本；CMake 只安装 `launch/` 目录，外场 launch 均指向 `launch/` 版 |
| `px4_offboard_tracking/launch/dual_tracking.launch_new.py` | 名带 `_new` 实为旧版；生效的是不带后缀版，仍被上游批处理脚本引用（未动） |
| `falcon_intercept_control/control_v0.yaml`（包根） | CMake 只安装 `config/`、`launch/` 目录；生效版在 `config/` |
| `falcon_intercept_control/intercept_fsm_node.cpp`（包根） | CMake 只编译 `src/` 版本，包根版与 src/ 版内容不同、不参与构建 |
| `falcon_intercept_control/safety_node.cpp`（包根） | 同上 |

注意：`falcon_intercept/launch/stereo_cameras_v0.launch.py` 交接材料中列为
零引用，实际被 `falcon_intercept_deepstream/tools/camera_mapping_contract_check.py`
引用，**保留在原位**。

## deprecated_packages/ — 退出活动链的整包（1 个）

| 包 | 说明 |
|---|---|
| `px4_interceptor_bridge/` | 旧消息桥，上游自带 `COLCON_IGNORE`，台账注明"仅历史参考"。整包搬离 `ros2_ws/src/`，`ros2_ws` 现为 12 个可构建包。读取该包源码的 `falcon_intercept_control/tools/generate_upstream_parameter_matrix.ps1` 已增加 `-ArchiveRoot` 参数 |

## 恢复方法

```bash
# 查看某文件在归档区的位置后，搬回原位：
git mv archive/snapshots/ros2_ws/src/<路径> ros2_ws/src/<路径>
# 或从初始导入提交直接取回（与归档内容逐字节一致）：
git checkout 96b44f9 -- ros2_ws/src/<路径>
```
