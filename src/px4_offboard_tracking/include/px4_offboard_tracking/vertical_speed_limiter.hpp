#pragma once

namespace px4_offboard_tracking
{

struct VerticalSpeedLimiterConfig
{
  double maximum_speed_mps{1.0};
  double soft_zone_mps{0.2};
  double overspeed_braking_gain{2.0};
  double maximum_braking_acceleration_mps2{2.0};
};

struct VerticalSpeedLimiterResult
{
  double acceleration_down_mps2{};
  bool outward_acceleration_limited{false};
  bool overspeed_braking_active{false};
};

// Applies the same symmetric speed-barrier principle to PX4 NED vertical
// motion. Positive velocity/acceleration is down; negative is up.
class VerticalSpeedLimiter
{
public:
  explicit VerticalSpeedLimiter(const VerticalSpeedLimiterConfig & config);

  VerticalSpeedLimiterResult apply(
    double requested_acceleration_down_mps2,
    double velocity_down_mps) const;

  const VerticalSpeedLimiterConfig & config() const {return config_;}

private:
  VerticalSpeedLimiterConfig config_;
};

}  // namespace px4_offboard_tracking
