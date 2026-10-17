# Milestone 0：开发基线

记录日期：2026-07-13  
功能分支：`feat/z-axis-visual-capture`  
基线来源：`uavws`  
基线提交：`f74ff53b23038d781c397882ac41a128419b44fc`

## 基线范围

当前包已经具备：

- 单机 PX4 Offboard 起飞和悬停；
- 双机 Target-Pursuer 节点；
- `velocity`、`attitude` 和 `body_rate` 三种控制层；
- 默认 `attitude` 姿态/推力控制；
- PD 加速度制导；
- 期望加速度到姿态/推力的几何转换；
- Target 状态超时保护；
- 速度、加速度、姿态推力和角速度限幅的基础单元测试。

## 当前双机行为

- Target 使用位置设定值，在约 5 m 高度执行半径 12 m、速度 2 m/s 的圆轨迹。
- Pursuer 默认使用姿态/推力指令跟随 Target。
- 当前制导包含 `standoff_distance_m=3.0`，属于保持距离的伴随跟踪，不是接触捕获。
- Target 状态由自身 `VehicleLocalPosition` 直接发布；Pursuer 将两架 PX4 的本地 NED
  状态直接用于相对位置计算。

## 已知限制

1. 两架 PX4 的原始 `VehicleLocalPosition` 没有转换到经过验证的公共 NED 坐标，不能作为
   后续视觉捕获功能的相对位置基准。
2. 目标状态接口没有显式的 `valid`、协方差、创新量和数据年龄契约。
3. 尚无向上相机模型、可配置相机外参、观测噪声与延迟模型。
4. 尚无六状态相对 EKF、接近走廊、预测 LOS 对准、质量门、捕获窗口和任务状态机。
5. `body_rate` 仅保留为现有对照能力；本阶段应先完成并验证 `attitude` 模式。
6. 本次 Milestone 0 只验证编译和单元测试，没有重新启动 Gazebo/PX4 运行仿真。

## 编译结果

执行命令：

```bash
cd ~/UAV_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select px4_offboard_tracking --symlink-install
```

结果：`px4_offboard_tracking` 编译成功，0 个失败包。

## 单元测试结果

执行命令：

```bash
cd ~/UAV_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select px4_offboard_tracking --event-handlers console_direct+
colcon test-result --verbose --test-result-base build/px4_offboard_tracking
```

结果：

- 3 个 GTest 测试套件；
- 7 个 GTest 用例全部通过；
- `colcon test-result`：8 tests，0 errors，0 failures，0 skipped。

## 当前启动命令

终端 1：

```bash
MicroXRCEAgent udp4 -p 8888
```

终端 2：

```bash
~/px4sim/startpx4_dual.sh
```

终端 3：

```bash
source /opt/ros/humble/setup.bash
source ~/UAV_ws/install/setup.bash
ros2 topic hz /fmu/out/vehicle_local_position
ros2 topic hz /px4_1/fmu/out/vehicle_local_position
ros2 launch px4_offboard_tracking dual_tracking.launch.py
```

## Milestone 0 验收

- 当前分支为 `feat/z-axis-visual-capture`；
- 分支起点与 `uavws` 的基线提交一致；
- 包可以独立编译；
- 现有单元测试无失败；
- 已明确记录公共坐标系缺失等后续必须消除的技术风险；
- 未修改 PX4 源码、PX4 子模块、系统环境文件或真实硬件输出代码；
- 尚未开始 Milestone 1。
