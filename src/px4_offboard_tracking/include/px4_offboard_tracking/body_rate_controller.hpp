#pragma once

#include "px4_offboard_tracking/geometric_control.hpp"

namespace px4_offboard_tracking
{

struct BodyRateControllerConfig
{
  double roll_pitch_gain{4.0};
  double yaw_gain{2.0};
  double maximum_roll_pitch_rate_rad_s{2.5};
  double maximum_yaw_rate_rad_s{1.5};
};

class BodyRateController
{
public:
  explicit BodyRateController(const BodyRateControllerConfig & config);

  BodyRateThrustCommand update(
    const Quaternion & current_body_to_ned,
    const AttitudeThrustCommand & desired) const;

private:
  BodyRateControllerConfig config_;
};

}  // namespace px4_offboard_tracking
