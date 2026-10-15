# virtual_interception_interfaces

本包只定义两个中立 ROS 2 消息：

- `VirtualShotEvent`：真值隔离控制器依据相机、IMM-EKF 和本机 PX4 状态发布；
- `VirtualHitResult`：独立仿真裁判依据真值计算，仅供仿真验收和绘图使用。

控制器不得订阅 `VirtualHitResult`。实机环境只需要安装本消息包以发布
`VirtualShotEvent`，不需要安装或运行仿真裁判。

固定话题：

```text
/tracking/virtual_shot_event
/evaluation/virtual_hit_result
```

第二个话题只允许出现在仿真评估侧。
