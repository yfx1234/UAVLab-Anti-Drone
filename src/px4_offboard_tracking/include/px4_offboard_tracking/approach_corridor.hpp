#pragma once

#include <array>

#include "px4_offboard_tracking/acceleration_guidance.hpp"

namespace px4_offboard_tracking
{

struct ApproachCorridorConfig
{
  double desired_vertical_separation_m{7.0};
  double prediction_time_s{0.8};
  double maximum_lead_distance_m{2.0};
  double horizontal_position_gain{0.6};
  double horizontal_velocity_gain{1.0};
  double maximum_horizontal_acceleration_mps2{2.0};
  double minimum_target_above_m{0.5};
};

struct ApproachCorridorResult
{
  std::array<double, 3> predicted_relative_position_ned{};
  std::array<double, 3> corridor_error_ned{};
  AccelerationCommand acceleration{};
  bool valid{false};
};

class ApproachCorridor
{
public:
  explicit ApproachCorridor(const ApproachCorridorConfig & config);

  ApproachCorridorResult update(
    const std::array<double, 3> & relative_position_ned,
    const std::array<double, 3> & relative_velocity_ned) const;

private:
  ApproachCorridorConfig config_;
};

}  // namespace px4_offboard_tracking
