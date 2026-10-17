#include "px4_offboard_tracking/body_rate_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4_offboard_tracking
{
namespace
{
Quaternion normalize(const Quaternion & quaternion)
{
  const double length = std::sqrt(
    quaternion.w * quaternion.w + quaternion.x * quaternion.x +
    quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  if (!std::isfinite(length) || length < 1e-9) {
    throw std::invalid_argument("invalid attitude quaternion");
  }
  return {quaternion.w / length, quaternion.x / length,
    quaternion.y / length, quaternion.z / length};
}

Quaternion multiply(const Quaternion & first, const Quaternion & second)
{
  return {
    first.w * second.w - first.x * second.x - first.y * second.y - first.z * second.z,
    first.w * second.x + first.x * second.w + first.y * second.z - first.z * second.y,
    first.w * second.y - first.x * second.z + first.y * second.w + first.z * second.x,
    first.w * second.z + first.x * second.y - first.y * second.x + first.z * second.w};
}
}  // namespace

BodyRateController::BodyRateController(const BodyRateControllerConfig & config)
: config_(config)
{
  if (config_.roll_pitch_gain <= 0.0 || config_.yaw_gain <= 0.0 ||
    config_.maximum_roll_pitch_rate_rad_s <= 0.0 || config_.maximum_yaw_rate_rad_s <= 0.0)
  {
    throw std::invalid_argument("invalid body-rate controller configuration");
  }
}

BodyRateThrustCommand BodyRateController::update(
  const Quaternion & current_body_to_ned, const AttitudeThrustCommand & desired) const
{
  const auto current = normalize(current_body_to_ned);
  const auto target = normalize(desired.attitude);
  const Quaternion current_conjugate{current.w, -current.x, -current.y, -current.z};
  const auto error = multiply(current_conjugate, target);
  const double sign = error.w >= 0.0 ? 1.0 : -1.0;
  BodyRateThrustCommand output;
  output.roll_rate = std::clamp(
    2.0 * config_.roll_pitch_gain * sign * error.x,
    -config_.maximum_roll_pitch_rate_rad_s, config_.maximum_roll_pitch_rate_rad_s);
  output.pitch_rate = std::clamp(
    2.0 * config_.roll_pitch_gain * sign * error.y,
    -config_.maximum_roll_pitch_rate_rad_s, config_.maximum_roll_pitch_rate_rad_s);
  output.yaw_rate = std::clamp(
    2.0 * config_.yaw_gain * sign * error.z,
    -config_.maximum_yaw_rate_rad_s, config_.maximum_yaw_rate_rad_s);
  output.thrust = desired.thrust;
  return output;
}

}  // namespace px4_offboard_tracking
