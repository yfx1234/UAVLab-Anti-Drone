#include "px4_offboard_tracking/relative_kinematics.hpp"

#include <algorithm>
#include <cmath>

namespace px4_offboard_tracking
{

RelativeKinematicsResult compute_relative_kinematics(
  const std::array<double, 3> & relative_position_m,
  const std::array<double, 3> & relative_velocity_mps)
{
  RelativeKinematicsResult output;
  output.relative_velocity_mps = relative_velocity_mps;
  for (int axis = 0; axis < 3; ++axis) {
    if (!std::isfinite(relative_position_m[axis]) ||
      !std::isfinite(relative_velocity_mps[axis]))
    {
      return output;
    }
  }

  const double rx = relative_position_m[0];
  const double ry = relative_position_m[1];
  const double rz = relative_position_m[2];
  const double vx = relative_velocity_mps[0];
  const double vy = relative_velocity_mps[1];
  const double vz = relative_velocity_mps[2];
  const double range_squared = rx * rx + ry * ry + rz * rz;
  if (range_squared < 1e-8) {return output;}

  output.range_m = std::sqrt(range_squared);
  const double radial_dot = rx * vx + ry * vy + rz * vz;
  output.closing_speed_mps = -radial_dot / output.range_m;
  output.los_rate_rad_s = {
    (ry * vz - rz * vy) / range_squared,
    (rz * vx - rx * vz) / range_squared,
    (rx * vy - ry * vx) / range_squared};

  const double velocity_squared = vx * vx + vy * vy + vz * vz;
  output.time_to_closest_approach_s = velocity_squared > 1e-8 ?
    std::max(0.0, -radial_dot / velocity_squared) : 0.0;
  const double closest_x = rx + vx * output.time_to_closest_approach_s;
  const double closest_y = ry + vy * output.time_to_closest_approach_s;
  const double closest_z = rz + vz * output.time_to_closest_approach_s;
  output.predicted_miss_distance_m = std::sqrt(
    closest_x * closest_x + closest_y * closest_y + closest_z * closest_z);
  output.valid = true;
  return output;
}

}  // namespace px4_offboard_tracking
