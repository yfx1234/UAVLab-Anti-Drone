# 旧搜索相机的尺寸证据

此目录为只读历史源码参考，不是独立ROS包，也不在当前构建路径中。

- 主DeepStream启动入口选用 `platform_exp_20260909/exp_opt3.engine`。
- USB搜索相机代码原始帧为2560×1440；mux默认1280×720。
- 同一路径现用模型清单实测输入NCHW [1,3,736,1280]，即宽1280、高736，输出[1,300,6]。见 models 目录清单。
- maintain-aspect-ratio=1 / symmetric-padding=1；最终网络尺寸由引擎决定。
- 更早Ultralytics试验入口有imgsz=640的默认值，不能将该默认值当成后来的TensorRT主链路尺寸。

这是源码与现用模型的证据，不保证每次历史运行都没有用命令行覆盖参数。
