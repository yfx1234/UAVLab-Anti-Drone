# zed_tracking — ZED X Mini 目标跟踪链路

替代旧三相机链路(搜索相机 + IMX477 双目)的感知与跟踪部分。
任务定义:搜索检测目标机,接近后在目标**斜下方 7~8 m** 保持跟踪,
机头指向目标,满足条件触发虚拟射击事件,由虚拟弹丸裁判判定命中。
不再撞击。

## 架构(话题流水线)

```
ZED X Mini(GMSL2, 4mm, 朝天, 基线沿机身纵轴)
  └─ zed_perception_node(C++)
       左目 1080p@30 → 竖版 YOLO 736x1280 TensorRT 15Hz
       深度 NEURAL_LIGHT, 框内中值 → 相机系→机体系三维
       ├─ /zed_perception_node/detections_json  (兼容旧合同)
       ├─ /zed_perception_node/target_json      (深度/伪测距双模式观测)
       ├─ /zed_perception_node/status           (1Hz 状态)
       └─ SVO 录像(LOSSLESS;H264/H265 在本机尾部损坏,勿用)
  └─ zed_track_filter_node.py
       观测+机体姿态/位置 → 绝对 NED → 六状态 CV 卡尔曼
       (复用 falcon_intercept.target_kf_node,双模观测方差,
        过程噪声 16, NIS 门限 21.7)
       └─ /zed_tracking/track_json(20Hz,位置+速度+健康度)
  └─ track_hold_node.py
       状态机 SEARCH→APPROACH→HOLD→AIM→FIRE / LOST
       斜下方保持(hold_back 2.5m + 垂直 7.07m = 7.5m)
       过近后退(<6.5m),加速度限幅 6 m/s²(≈31° 倾斜)
       └─ /upstream/fmu/in/*(速度+yaw,走旧安全阀门)
       └─ /zed_tracking/shot_event(距离 7~8m + 指向 <8°)
  └─ shot_event_bridge.py + virtual_interception_referee
       视线方向虚拟弹丸 → /evaluation/virtual_solver_hit_result
       (physical 轴=机头水平,朝天目标必 MISS,仅作参考)
```

安全阀门(upstream_gate_node + px4_upstream_adapter_node)一行未改,
继续提供:白名单指令类、0.1s 新鲜度、飞手接管锁存、shadow/live 模式。

## 运行

```bash
# 一键(默认 shadow,只记录不实控):
bash src/zed_tracking/tools/field_flight_zed.sh check|start|status|stop
# 实控: export FALCON_ADAPTER_MODE=live(需现场确认位流程)

# 单节点:
ros2 run zed_tracking zed_perception_node --ros-args \
  -p engine_path:=~/Jetson_ROS2_V0/weights/candidates/platform_exp_20260909/exp_portrait_1280x736_opt3.engine

# 闭环仿真(含安全阀门+裁判):
bash src/zed_tracking/tools/run_sim_test.sh

# SVO 回放(重打时间戳,配合 mock 姿态):
ros2 run zed_tracking zed_perception_node --ros-args \
  -p input_svo:=<file.svo2> -p restamp_output:=true ...
```

注意:相机独占,运行前先停 zed_preview 图传预览。

## 已完成验证(2026-10-07)

- 单元测试 61 个(几何/深度统计/检测解析/滤波/控制),全过。
- 台架:15Hz 检测精确,深度一致性 ±5%,SVO(LOSSLESS)可回放,
  竖版引擎 736x1280 输入无黑边。
- 闭环仿真(mock感知→滤波→控制→安全阀门 live→mock机体→裁判):
  - 悬停目标:稳态保持误差 RMS 0.05~0.17 m,指向 <1°,
    射击时距离 7.3~7.7 m,裁判视线轴全部 HIT(miss 0.02~0.16 m)。
  - 绕圈目标(1.5 m/s 机动):保持可用(峰值 ~10 m 拉回),可射击并 HIT。
- 回放:SVO→滤波 21.9Hz 输出,位置跳变 95 分位 0.7 cm。

## 关键设计决定(含教训)

1. 画面竖(朝天+基线沿纵轴)→ 导出竖版引擎而非旋转预处理。
2. 滤波必须滤**绝对位置**(观测加自身位置补偿),否则速度估计被
   机体自身运动污染(仿真曾偏差 1.2 m)。
3. 保持点取斜下方(hold_back>0):正下方时视线竖直,yaw 无参考。
4. H264/H265 SVO 在本机尾部损坏(SDK 5.5 兼容问题),用 LOSSLESS。
5. 裁判 physical 轴假定枪沿机体 -Z;本链路枪在机头(+X),
   已加 attack_axis_body 参数;虚拟命中以视线轴(solver)为准。
6. 真实枪架若无俯仰,朝天目标物理上无法命中(miss=垂直距离)。

## 待办(外场)

- 外参标定(占位:q=[1,0,0,0],t=[0.10,0,0.12]m),伪测距参考值标定。
- NEURAL_LIGHT vs PERFORMANCE 深度精度/算力实测(8% 丢帧率下的表现)。
- 20 m 检出率实测(30 cm 目标约 17 px @1080p),必要时升 HD2K 或补训练。
- IMN/CA 多模型滤波(当前 CV,过程噪声 16),真机 yaw/速度响应整定。
- 比赛若提供目标真值源,替换 bridge 的 truth_source。
