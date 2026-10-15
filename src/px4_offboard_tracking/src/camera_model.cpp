#include "px4_offboard_tracking/camera_model.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace px4_offboard_tracking
{

CameraModel::CameraModel(
  const CameraIntrinsics & intrinsics, const CameraExtrinsics & extrinsics,
  const CameraFieldOfView & field_of_view)
: intrinsics_(intrinsics), extrinsics_(extrinsics), field_of_view_(field_of_view)
{
  if (!std::isfinite(intrinsics_.fx) || !std::isfinite(intrinsics_.fy) ||
    intrinsics_.fx <= 0.0 || intrinsics_.fy <= 0.0 || intrinsics_.width <= 0 ||
    intrinsics_.height <= 0 || intrinsics_.minimum_depth_m <= 0.0)
  {
    throw std::invalid_argument("invalid pinhole camera intrinsics");
  }
  if (field_of_view_.horizontal_half_angle_rad < 0.0 ||
    field_of_view_.vertical_half_angle_rad < 0.0 ||
    field_of_view_.horizontal_half_angle_rad >= M_PI_2 ||
    field_of_view_.vertical_half_angle_rad >= M_PI_2)
  {
    throw std::invalid_argument("invalid camera field of view");
  }
  extrinsics_.q_body_to_optical = normalize(extrinsics_.q_body_to_optical);
}

Quaterniond CameraModel::default_upward_optical_quaternion()
{
  // 实物安装（2026-10-13 定案）：相机竖直向上，长边（宽轴 H74°/图像
  // u 轴）沿机头方向。optical +Z = body -Z（朝上），optical +X =
  // body +X（机头/宽轴），右手系约束下 optical +Y = body -Y（v 轴
  // 指向机体左舷）。旧值 (0,√2/2,√2/2,0) 的宽轴沿机体右舷，与实物
  // 相反——宽轴对准航线时窄轴吃掉了主导误差方向（沿航线跟踪滞后）。
  return {0.0, 1.0, 0.0, 0.0};
}

Quaterniond CameraModel::default_search_optical_quaternion()
{
  // Body FRD -> optical for a camera tilted 10 deg from body -Z toward body +X.
  return {
    -0.06162841671621935, 0.7044160264027587,
    0.7044160264027587, 0.06162841671621935};
}

Quaterniond CameraModel::normalize(const Quaterniond & quaternion)
{
  const double norm = std::sqrt(
    quaternion.w * quaternion.w + quaternion.x * quaternion.x +
    quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  if (!std::isfinite(norm) || norm < 1e-9) {
    throw std::invalid_argument("invalid camera or vehicle quaternion");
  }
  return {
    quaternion.w / norm, quaternion.x / norm, quaternion.y / norm, quaternion.z / norm};
}

Quaterniond CameraModel::conjugate(const Quaterniond & quaternion)
{
  return {quaternion.w, -quaternion.x, -quaternion.y, -quaternion.z};
}

Quaterniond CameraModel::multiply(const Quaterniond & first, const Quaterniond & second)
{
  return {
    first.w * second.w - first.x * second.x - first.y * second.y - first.z * second.z,
    first.w * second.x + first.x * second.w + first.y * second.z - first.z * second.y,
    first.w * second.y - first.x * second.z + first.y * second.w + first.z * second.x,
    first.w * second.z + first.x * second.y - first.y * second.x + first.z * second.w};
}

Vector3d CameraModel::rotate(const Quaterniond & quaternion, const Vector3d & vector)
{
  const Quaterniond q = normalize(quaternion);
  const Vector3d q_vector{q.x, q.y, q.z};
  const Vector3d twice_cross{
    2.0 * (q_vector.y * vector.z - q_vector.z * vector.y),
    2.0 * (q_vector.z * vector.x - q_vector.x * vector.z),
    2.0 * (q_vector.x * vector.y - q_vector.y * vector.x)};
  const Vector3d second_cross{
    q_vector.y * twice_cross.z - q_vector.z * twice_cross.y,
    q_vector.z * twice_cross.x - q_vector.x * twice_cross.z,
    q_vector.x * twice_cross.y - q_vector.y * twice_cross.x};
  return {
    vector.x + q.w * twice_cross.x + second_cross.x,
    vector.y + q.w * twice_cross.y + second_cross.y,
    vector.z + q.w * twice_cross.z + second_cross.z};
}

PinholeProjection CameraModel::project(
  const Vector3d & target_relative_ned, const Quaterniond & q_body_to_ned) const
{
  const Quaterniond q_ned_to_body = conjugate(normalize(q_body_to_ned));
  const Vector3d target_relative_body = rotate(q_ned_to_body, target_relative_ned);
  const Vector3d target_from_camera_body{
    target_relative_body.x - extrinsics_.translation_body_frd_m.x,
    target_relative_body.y - extrinsics_.translation_body_frd_m.y,
    target_relative_body.z - extrinsics_.translation_body_frd_m.z};
  const Vector3d target_camera = rotate(
    extrinsics_.q_body_to_optical, target_from_camera_body);

  PinholeProjection output;
  output.position_camera = target_camera;
  output.depth = target_camera.z;
  output.range_m = std::sqrt(
    target_camera.x * target_camera.x + target_camera.y * target_camera.y +
    target_camera.z * target_camera.z);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  output.pixel_u = nan;
  output.pixel_v = nan;

  if (!std::isfinite(target_camera.x) || !std::isfinite(target_camera.y) ||
    !std::isfinite(target_camera.z) || target_camera.z <= intrinsics_.minimum_depth_m)
  {
    return output;
  }

  output.pixel_u = intrinsics_.fx * target_camera.x / target_camera.z + intrinsics_.cx;
  output.pixel_v = intrinsics_.fy * target_camera.y / target_camera.z + intrinsics_.cy;
  const double horizontal_angle = std::atan2(target_camera.x, target_camera.z);
  const double vertical_angle = std::atan2(target_camera.y, target_camera.z);
  const bool angular_fov_ok =
    (field_of_view_.horizontal_half_angle_rad <= 0.0 ||
    std::abs(horizontal_angle) <= field_of_view_.horizontal_half_angle_rad) &&
    (field_of_view_.vertical_half_angle_rad <= 0.0 ||
    std::abs(vertical_angle) <= field_of_view_.vertical_half_angle_rad);
  output.visible = angular_fov_ok &&
    output.pixel_u >= 0.0 && output.pixel_u < intrinsics_.width &&
    output.pixel_v >= 0.0 && output.pixel_v < intrinsics_.height;
  return output;
}

Quaterniond CameraModel::q_camera_to_ned(const Quaterniond & q_body_to_ned) const
{
  return normalize(multiply(
    normalize(q_body_to_ned), conjugate(extrinsics_.q_body_to_optical)));
}

Vector3d CameraModel::camera_position_to_relative_ned(
  const Vector3d & position_camera, const Quaterniond & q_body_to_ned) const
{
  const Vector3d position_body = rotate(
    conjugate(extrinsics_.q_body_to_optical), position_camera);
  const Vector3d target_relative_body{
    position_body.x + extrinsics_.translation_body_frd_m.x,
    position_body.y + extrinsics_.translation_body_frd_m.y,
    position_body.z + extrinsics_.translation_body_frd_m.z};
  return rotate(normalize(q_body_to_ned), target_relative_body);
}

BearingCameraModel::BearingCameraModel(
  const CameraIntrinsics & intrinsics, const CameraExtrinsics & extrinsics,
  const CameraFieldOfView & field_of_view)
: projection_model_(intrinsics, extrinsics, field_of_view)
{
}

BearingObservation BearingCameraModel::observe(
  const Vector3d & target_relative_ned, const Quaterniond & q_body_to_ned) const
{
  const auto projection = projection_model_.project(target_relative_ned, q_body_to_ned);
  BearingObservation output;
  output.pixel_u = projection.pixel_u;
  output.pixel_v = projection.pixel_v;
  output.visible = projection.visible;
  if (projection.range_m <= 1e-9 || !std::isfinite(projection.range_m)) {
    return output;
  }
  output.horizontal_angle_rad = std::atan2(
    projection.position_camera.x, projection.position_camera.z);
  output.vertical_angle_rad = std::atan2(
    projection.position_camera.y, projection.position_camera.z);
  output.unit_line_of_sight_optical = {
    projection.position_camera.x / projection.range_m,
    projection.position_camera.y / projection.range_m,
    projection.position_camera.z / projection.range_m};
  return output;
}

}  // namespace px4_offboard_tracking
