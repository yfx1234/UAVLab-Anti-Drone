#include "px4_offboard_tracking/approach_corridor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4_offboard_tracking
{

ApproachCorridor::ApproachCorridor(const ApproachCorridorConfig & config)
: config_(config)
{
  if (config_.desired_vertical_separation_m <= 0.0 || config_.prediction_time_s < 0.0 ||
    config_.maximum_lead_distance_m < 0.0 || config_.horizontal_position_gain < 0.0 ||
    config_.horizontal_velocity_gain < 0.0 ||
    config_.maximum_horizontal_acceleration_mps2 <= 0.0 ||
    config_.minimum_target_above_m <= 0.0)
  {
    throw std::invalid_argument("invalid approach corridor configuration");
  }
}

ApproachCorridorResult ApproachCorridor::update(
  const std::array<double, 3> & relative_position_ned,
  const std::array<double, 3> & relative_velocity_ned) const
{
  ApproachCorridorResult output;
  for (int axis = 0; axis < 3; ++axis) {
    if (!std::isfinite(relative_position_ned[axis]) ||
      !std::isfinite(relative_velocity_ned[axis]))
    {
      return output;
    }
  }
  if (relative_position_ned[2] >= -config_.minimum_target_above_m) {return output;}

  std::array<double, 3> lead{
    relative_velocity_ned[0] * config_.prediction_time_s,
    relative_velocity_ned[1] * config_.prediction_time_s,
    relative_velocity_ned[2] * config_.prediction_time_s};
  const double lead_norm = std::sqrt(
    lead[0] * lead[0] + lead[1] * lead[1] + lead[2] * lead[2]);
  if (lead_norm > config_.maximum_lead_distance_m && lead_norm > 1e-9) {
    const double scale = config_.maximum_lead_distance_m / lead_norm;
    for (double & component : lead) {component *= scale;}
  }
  for (int axis = 0; axis < 3; ++axis) {
    output.predicted_relative_position_ned[axis] = relative_position_ned[axis] + lead[axis];
  }

  // Desired geometry is directly below the predicted target in common NED.
  output.corridor_error_ned = {
    output.predicted_relative_position_ned[0],
    output.predicted_relative_position_ned[1],
    output.predicted_relative_position_ned[2] + config_.desired_vertical_separation_m};
  output.acceleration.ax =
    config_.horizontal_position_gain * output.corridor_error_ned[0] +
    config_.horizontal_velocity_gain * relative_velocity_ned[0];
  output.acceleration.ay =
    config_.horizontal_position_gain * output.corridor_error_ned[1] +
    config_.horizontal_velocity_gain * relative_velocity_ned[1];
  const double horizontal_acceleration = std::hypot(
    output.acceleration.ax, output.acceleration.ay);
  if (horizontal_acceleration > config_.maximum_horizontal_acceleration_mps2) {
    const double scale = config_.maximum_horizontal_acceleration_mps2 / horizontal_acceleration;
    output.acceleration.ax *= scale;
    output.acceleration.ay *= scale;
  }
  output.acceleration.yaw = std::hypot(
    output.predicted_relative_position_ned[0],
    output.predicted_relative_position_ned[1]) > 1e-6 ?
    std::atan2(
      output.predicted_relative_position_ned[1],
      output.predicted_relative_position_ned[0]) : 0.0;
  output.valid = true;
  return output;
}

}  // namespace px4_offboard_tracking

