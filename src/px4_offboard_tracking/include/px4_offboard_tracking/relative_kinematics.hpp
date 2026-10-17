#pragma once

#include <array>

namespace px4_offboard_tracking
{

struct RelativeKinematicsResult
{
  double range_m{};
  std::array<double, 3> relative_velocity_mps{};
  double closing_speed_mps{};
  std::array<double, 3> los_rate_rad_s{};
  double time_to_closest_approach_s{};
  double predicted_miss_distance_m{};
  bool valid{false};
};

RelativeKinematicsResult compute_relative_kinematics(
  const std::array<double, 3> & relative_position_m,
  const std::array<double, 3> & relative_velocity_mps);

}  // namespace px4_offboard_tracking
