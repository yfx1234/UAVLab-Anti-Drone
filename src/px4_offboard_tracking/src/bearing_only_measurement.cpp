#include "px4_offboard_tracking/bearing_only_measurement.hpp"

#include <algorithm>
#include <cmath>

namespace px4_offboard_tracking
{

namespace
{

// 与 camera_model 内部实现一致的四元数旋转（body→target 系约定同款）。
Vector3d rotate_quat(const Quaterniond & q, const Vector3d & v)
{
  const double w = q.w, x = q.x, y = q.y, z = q.z;
  return {
    (1.0 - 2.0 * (y * y + z * z)) * v.x + 2.0 * (x * y - z * w) * v.y +
      2.0 * (x * z + y * w) * v.z,
    2.0 * (x * y + z * w) * v.x + (1.0 - 2.0 * (x * x + z * z)) * v.y +
      2.0 * (y * z - x * w) * v.z,
    2.0 * (x * z - y * w) * v.x + 2.0 * (y * z + x * w) * v.y +
      (1.0 - 2.0 * (x * x + y * y)) * v.z};
}

}  // namespace

BearingOnlyMeasurement make_bearing_only_measurement(
  const CameraModel & camera_model,
  const CameraIntrinsics & intrinsics,
  const Quaterniond & q_body_to_ned,
  double pixel_u, double pixel_v,
  double pixel_stddev_px,
  double nominal_range_prior_m,
  double range_sigma_m,
  const Vector3d & predicted_position_ned)
{
  const double nx = (pixel_u - intrinsics.cx) / intrinsics.fx;
  const double ny = (pixel_v - intrinsics.cy) / intrinsics.fy;
  Vector3d los_optical{nx, ny, 1.0};
  const double los_norm = std::sqrt(nx * nx + ny * ny + 1.0);
  los_optical.x /= los_norm;
  los_optical.y /= los_norm;
  los_optical.z /= los_norm;

  double nominal_range = nominal_range_prior_m;
  const Quaterniond q_camera_to_ned = camera_model.q_camera_to_ned(q_body_to_ned);
  const Vector3d los_ned = rotate_quat(q_camera_to_ned, los_optical);
  const double los_dot_predicted =
    los_ned.x * predicted_position_ned.x +
    los_ned.y * predicted_position_ned.y +
    los_ned.z * predicted_position_ned.z;
  if (std::isfinite(los_dot_predicted) && los_dot_predicted > 1.0) {
    nominal_range = los_dot_predicted;
  }

  BearingOnlyMeasurement out;
  out.nominal_range_m = nominal_range;
  out.position_ned = camera_model.camera_position_to_relative_ned(
    Vector3d{
      los_optical.x * nominal_range, los_optical.y * nominal_range,
      los_optical.z * nominal_range},
    q_body_to_ned);

  const double angular_sigma =
    std::max(0.5, pixel_stddev_px) / std::min(intrinsics.fx, intrinsics.fy);
  const double lateral_sigma = std::max(1e-3, angular_sigma * nominal_range);
  const double camera_variance[3] = {
    lateral_sigma * lateral_sigma, lateral_sigma * lateral_sigma,
    range_sigma_m * range_sigma_m};
  // optical→NED：R 的第 k 列 = optical 第 k 轴在 NED 的单位向量；
  // C_ned = R C Rᵀ，C 为对角阵。
  const Vector3d axes_optical[3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
  double rotation[9];
  for (int k = 0; k < 3; ++k) {
    const Vector3d column = rotate_quat(q_camera_to_ned, axes_optical[k]);
    rotation[k] = column.x;
    rotation[3 + k] = column.y;
    rotation[6 + k] = column.z;
  }
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      double sum = 0.0;
      for (int k = 0; k < 3; ++k) {
        sum += rotation[3 * row + k] * camera_variance[k] *
          rotation[3 * column + k];
      }
      out.covariance_ned[3 * row + column] = sum;
    }
    out.covariance_ned[3 * row + row] =
      std::max(out.covariance_ned[3 * row + row], 1e-6);
  }
  return out;
}

}  // namespace px4_offboard_tracking
