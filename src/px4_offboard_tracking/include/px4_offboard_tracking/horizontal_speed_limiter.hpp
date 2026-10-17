#pragma once

#include "px4_offboard_tracking/acceleration_guidance.hpp"

namespace px4_offboard_tracking
{

struct HorizontalSpeedLimiterConfig
{
  double maximum_speed_mps{4.0};
  double soft_zone_mps{0.5};
  double overspeed_braking_gain{2.0};
  double maximum_braking_acceleration_mps2{2.0};
};

struct HorizontalSpeedLimiterResult
{
  AccelerationCommand acceleration{};
  double horizontal_speed_mps{};
  bool outward_acceleration_limited{false};
  bool overspeed_braking_active{false};
};

// Shapes acceleration before attitude generation. It progressively removes
// acceleration along the velocity vector inside a soft speed zone and commands
// bounded braking above the configured speed. Lateral steering authority is kept.
class HorizontalSpeedLimiter
{
public:
  explicit HorizontalSpeedLimiter(const HorizontalSpeedLimiterConfig & config);

  HorizontalSpeedLimiterResult apply(
    const AccelerationCommand & acceleration,
    double velocity_north_mps,
    double velocity_east_mps) const;

  const HorizontalSpeedLimiterConfig & config() const {return config_;}

private:
  HorizontalSpeedLimiterConfig config_;
};

}  // namespace px4_offboard_tracking
