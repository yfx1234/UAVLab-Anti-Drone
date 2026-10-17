#include "px4_offboard_tracking/vertical_speed_limiter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4_offboard_tracking
{

VerticalSpeedLimiter::VerticalSpeedLimiter(
  const VerticalSpeedLimiterConfig & config)
: config_(config)
{
  if (!std::isfinite(config_.maximum_speed_mps) || config_.maximum_speed_mps <= 0.0 ||
    !std::isfinite(config_.soft_zone_mps) || config_.soft_zone_mps <= 0.0 ||
    config_.soft_zone_mps >= config_.maximum_speed_mps ||
    !std::isfinite(config_.overspeed_braking_gain) ||
    config_.overspeed_braking_gain <= 0.0 ||
    !std::isfinite(config_.maximum_braking_acceleration_mps2) ||
    config_.maximum_braking_acceleration_mps2 <= 0.0)
  {
    throw std::invalid_argument("invalid vertical speed limiter configuration");
  }
}

VerticalSpeedLimiterResult VerticalSpeedLimiter::apply(
  double requested_acceleration_down_mps2,
  double velocity_down_mps) const
{
  VerticalSpeedLimiterResult output;
  output.acceleration_down_mps2 = requested_acceleration_down_mps2;
  if (!std::isfinite(requested_acceleration_down_mps2) ||
    !std::isfinite(velocity_down_mps))
  {
    return output;
  }

  const double speed = std::abs(velocity_down_mps);
  if (speed <= 1.0e-9) {
    return output;
  }
  const double direction = velocity_down_mps > 0.0 ? 1.0 : -1.0;
  const double outward_acceleration =
    requested_acceleration_down_mps2 * direction;
  double desired_outward_acceleration = outward_acceleration;
  const double soft_zone_start = config_.maximum_speed_mps - config_.soft_zone_mps;

  if (speed > config_.maximum_speed_mps) {
    const double braking = std::min(
      config_.maximum_braking_acceleration_mps2,
      config_.overspeed_braking_gain * (speed - config_.maximum_speed_mps));
    desired_outward_acceleration = std::min(outward_acceleration, -braking);
    output.overspeed_braking_active = true;
  } else if (speed > soft_zone_start && outward_acceleration > 0.0) {
    const double scale = std::clamp(
      (config_.maximum_speed_mps - speed) / config_.soft_zone_mps, 0.0, 1.0);
    desired_outward_acceleration = outward_acceleration * scale;
    output.outward_acceleration_limited = true;
  }
  output.acceleration_down_mps2 = desired_outward_acceleration * direction;
  return output;
}

}  // namespace px4_offboard_tracking
