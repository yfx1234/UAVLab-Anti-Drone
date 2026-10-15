#include "px4_offboard_tracking/horizontal_speed_limiter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4_offboard_tracking
{

HorizontalSpeedLimiter::HorizontalSpeedLimiter(const HorizontalSpeedLimiterConfig & config)
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
    throw std::invalid_argument("invalid horizontal speed limiter configuration");
  }
}

HorizontalSpeedLimiterResult HorizontalSpeedLimiter::apply(
  const AccelerationCommand & acceleration,
  double velocity_north_mps,
  double velocity_east_mps) const
{
  HorizontalSpeedLimiterResult output;
  output.acceleration = acceleration;
  if (!std::isfinite(velocity_north_mps) || !std::isfinite(velocity_east_mps) ||
    !std::isfinite(acceleration.ax) || !std::isfinite(acceleration.ay))
  {
    return output;
  }

  const double speed = std::hypot(velocity_north_mps, velocity_east_mps);
  output.horizontal_speed_mps = speed;
  if (speed <= 1e-9) {return output;}

  const double unit_north = velocity_north_mps / speed;
  const double unit_east = velocity_east_mps / speed;
  const double parallel_acceleration =
    acceleration.ax * unit_north + acceleration.ay * unit_east;
  double desired_parallel_acceleration = parallel_acceleration;
  const double soft_zone_start = config_.maximum_speed_mps - config_.soft_zone_mps;

  if (speed > config_.maximum_speed_mps) {
    const double braking = std::min(
      config_.maximum_braking_acceleration_mps2,
      config_.overspeed_braking_gain * (speed - config_.maximum_speed_mps));
    desired_parallel_acceleration = std::min(parallel_acceleration, -braking);
    output.overspeed_braking_active = true;
  } else if (speed > soft_zone_start && parallel_acceleration > 0.0) {
    const double outward_scale = std::clamp(
      (config_.maximum_speed_mps - speed) / config_.soft_zone_mps, 0.0, 1.0);
    desired_parallel_acceleration = parallel_acceleration * outward_scale;
    output.outward_acceleration_limited = true;
  }

  const double correction = desired_parallel_acceleration - parallel_acceleration;
  output.acceleration.ax += correction * unit_north;
  output.acceleration.ay += correction * unit_east;
  return output;
}

}  // namespace px4_offboard_tracking
