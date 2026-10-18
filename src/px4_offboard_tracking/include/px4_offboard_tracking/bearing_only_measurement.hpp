#pragma once

#include <array>

#include "px4_offboard_tracking/camera_model.hpp"

namespace px4_offboard_tracking
{

/// H-1：bearing-only 伪量测构造结果。
struct BearingOnlyMeasurement
{
  Vector3d position_ned{};
  std::array<double, 9> covariance_ned{};
  double nominal_range_m{};  // 实际采用的名义距离（诊断/测试）
};

/// 由像素坐标构造经典 bearing-only 伪量测（无 ROS 依赖，节点与 gtest
/// 共用，语义与原 relative_estimator 内联实现逐位一致）：
/// - 视线方向 = 像素 → optical 单位向量 → NED；
/// - 距离 = 滤波器预测位置在视线上的投影（>1 m 时），否则用先验名义
///   距离（未初始化时的 12 m 先验即由此传入）；
/// - 协方差在 optical 系取横向 σ=角σ×距离、距离 σ=大参数值，旋转到
///   NED 后只修正横向、距离通道几乎不动。
BearingOnlyMeasurement make_bearing_only_measurement(
  const CameraModel & camera_model,
  const CameraIntrinsics & intrinsics,
  const Quaterniond & q_body_to_ned,
  double pixel_u, double pixel_v,
  double pixel_stddev_px,
  double nominal_range_prior_m,
  double range_sigma_m,
  const Vector3d & predicted_position_ned);

}  // namespace px4_offboard_tracking
