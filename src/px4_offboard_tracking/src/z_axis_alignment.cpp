#include "px4_offboard_tracking/z_axis_alignment.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4_offboard_tracking
{
namespace
{

double norm(const std::array<double, 3> & vector)
{
  return std::sqrt(
    vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
}

std::array<double, 3> normalize(const std::array<double, 3> & vector)
{
  const double length = norm(vector);
  if (length < 1e-9) {return {};}
  return {vector[0] / length, vector[1] / length, vector[2] / length};
}

Quaternion normalize_quaternion(const Quaternion & quaternion)
{
  const double length = std::sqrt(
    quaternion.w * quaternion.w + quaternion.x * quaternion.x +
    quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  if (!std::isfinite(length) || length < 1e-9) {return {};}
  return {quaternion.w / length, quaternion.x / length,
    quaternion.y / length, quaternion.z / length};
}

std::array<double, 3> rotate(
  const Quaternion & input_quaternion, const std::array<double, 3> & vector)
{
  const auto quaternion = normalize_quaternion(input_quaternion);
  const std::array<double, 3> q{quaternion.x, quaternion.y, quaternion.z};
  const std::array<double, 3> twice_cross{
    2.0 * (q[1] * vector[2] - q[2] * vector[1]),
    2.0 * (q[2] * vector[0] - q[0] * vector[2]),
    2.0 * (q[0] * vector[1] - q[1] * vector[0])};
  const std::array<double, 3> second_cross{
    q[1] * twice_cross[2] - q[2] * twice_cross[1],
    q[2] * twice_cross[0] - q[0] * twice_cross[2],
    q[0] * twice_cross[1] - q[1] * twice_cross[0]};
  return {vector[0] + quaternion.w * twice_cross[0] + second_cross[0],
    vector[1] + quaternion.w * twice_cross[1] + second_cross[1],
    vector[2] + quaternion.w * twice_cross[2] + second_cross[2]};
}

}  // namespace

ZAxisAlignment::ZAxisAlignment(const ZAxisAlignmentConfig & config)
: config_(config)
{
  if (config_.prediction_time_s < 0.0 || config_.maximum_tilt_rad <= 0.0 ||
    config_.maximum_tilt_rad >= 1.4 || config_.minimum_target_above_m <= 0.0 ||
    config_.alignment_entry_horizontal_distance_m <= 0.0 ||
    config_.alignment_entry_relative_speed_mps <= 0.0)
  {
    throw std::invalid_argument("invalid z-axis alignment configuration");
  }
}

std::array<double, 3> ZAxisAlignment::body_minus_z_axis_ned(
  const Quaternion & body_to_ned)
{
  return normalize(rotate(body_to_ned, {0.0, 0.0, -1.0}));
}

ZAxisAlignmentResult ZAxisAlignment::update(
  const std::array<double, 3> & relative_position_ned,
  const std::array<double, 3> & relative_velocity_ned,
  const Quaternion & current_body_to_ned) const
{
  ZAxisAlignmentResult output;
  for (int axis = 0; axis < 3; ++axis) {
    if (!std::isfinite(relative_position_ned[axis]) ||
      !std::isfinite(relative_velocity_ned[axis]))
    {
      return output;
    }
    output.predicted_los_ned[axis] =
      relative_position_ned[axis] + config_.prediction_time_s * relative_velocity_ned[axis];
  }
  if (output.predicted_los_ned[2] >= -config_.minimum_target_above_m ||
    norm(output.predicted_los_ned) < 1e-6)
  {
    return output;
  }

  std::array<double, 3> limited_los = output.predicted_los_ned;
  const double horizontal = std::hypot(limited_los[0], limited_los[1]);
  const double maximum_horizontal = -limited_los[2] * std::tan(config_.maximum_tilt_rad);
  if (horizontal > maximum_horizontal && horizontal > 1e-9) {
    const double scale = maximum_horizontal / horizontal;
    limited_los[0] *= scale;
    limited_los[1] *= scale;
    output.tilt_limited = true;
  }
  output.desired_camera_axis_ned = normalize(limited_los);
  output.current_camera_axis_ned = body_minus_z_axis_ned(current_body_to_ned);
  const double dot = std::clamp(
    output.desired_camera_axis_ned[0] * output.current_camera_axis_ned[0] +
    output.desired_camera_axis_ned[1] * output.current_camera_axis_ned[1] +
    output.desired_camera_axis_ned[2] * output.current_camera_axis_ned[2], -1.0, 1.0);
  output.alignment_error_rad = std::acos(dot);
  const double horizontal_distance = std::hypot(
    relative_position_ned[0], relative_position_ned[1]);
  const double horizontal_relative_speed = std::hypot(
    relative_velocity_ned[0], relative_velocity_ned[1]);
  const double distance_weight = std::clamp(
    1.0 - horizontal_distance / config_.alignment_entry_horizontal_distance_m, 0.0, 1.0);
  const double speed_weight = std::clamp(
    1.0 - horizontal_relative_speed / config_.alignment_entry_relative_speed_mps, 0.0, 1.0);
  output.alignment_weight = distance_weight * speed_weight;
  output.valid = std::isfinite(output.alignment_error_rad);
  return output;
}

}  // namespace px4_offboard_tracking
