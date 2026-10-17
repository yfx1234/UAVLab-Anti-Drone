# virtual_interception_referee

独立仿真裁判包。它订阅控制器的 `/tracking/virtual_shot_event`、仿真真值
`/evaluation/relative_truth`、起桨点任务原点、拦截机本地位置和姿态，传播
400 m/s 虚拟弹丸并发布：

```text
/evaluation/virtual_hit_result
/evaluation/virtual_solver_hit_result
/evaluation/mission_success
```

`virtual_hit_result` 按发射时刻飞机实际 Body -Z 攻击轴判定，决定任务是否命中；
`virtual_solver_hit_result` 按解算器给出的理想射线判定，只用于区分预测解算误差和
姿态跟踪误差。两条弹道均使用起桨点任务 NED；真值跨发射窗口断流超过默认
`0.1 s` 时结果为 `INVALID`，不再误记为 `MISS`。

本包只能在仿真验收时启动，不得成为 `px4_offboard_tracking` 或实机 launch 的依赖。
裁判结果不得反馈给控制器。

任务命中默认采用实测打击效果锥：35 m处有效直径1.7 m，对应半角约1.391°。
裁判按弹丸从发射点传播的距离动态计算有效半径，例如15/25/35 m处约为
0.364/0.607/0.850 m。`hit_radius_m=0.4`只作为显式关闭效果锥时的旧固定半径回退，
不再决定默认任务结果。该效果数据若后续重新实测，必须通过
`effect_reference_range_m`与`effect_reference_diameter_m`成对更新并保留依据。

裁判独立统计两次有效命中，只有两次命中对应的射击时间戳间隔至少为 `3 s` 才把
`/evaluation/mission_success` 置为 `true`。该结果仍不反馈飞行控制器。
