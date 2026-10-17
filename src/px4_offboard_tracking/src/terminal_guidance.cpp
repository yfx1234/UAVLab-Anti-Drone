#include "px4_offboard_tracking/terminal_guidance.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4_offboard_tracking
{
namespace
{

double dot(const std::array<double, 3> & first, const std::array<double, 3> & second)
{
  return first[0] * second[0] + first[1] * second[1] + first[2] * second[2];
}

std::array<double, 3> cross(
  const std::array<double, 3> & first, const std::array<double, 3> & second)
{
  return {
    first[1] * second[2] - first[2] * second[1],
    first[2] * second[0] - first[0] * second[2],
    first[0] * second[1] - first[1] * second[0]};
}

}  // namespace

TerminalGuidance::TerminalGuidance(const TerminalGuidanceConfig & config)
: config_(config)
{
  if (config_.navigation_constant <= 0.0 || config_.minimum_closing_speed_mps < 0.0 ||
    config_.line_of_sight_acceleration_mps2 <= 0.0 ||
    config_.acceleration_feedforward_gain < 0.0 ||
    config_.maximum_horizontal_acceleration_mps2 <= 0.0 ||
    config_.maximum_vertical_acceleration_mps2 <= 0.0)
  {
    throw std::invalid_argument("invalid terminal guidance configuration");
  }
}

TerminalGuidanceResult TerminalGuidance::update(
  const std::array<double, 3> & relative_position_ned,
  const std::array<double, 3> & relative_velocity_ned,
  const std::array<double, 3> & relative_acceleration_ned) const
{
  TerminalGuidanceResult output;
  for (int axis = 0; axis < 3; ++axis) {
    if (!std::isfinite(relative_position_ned[axis]) ||
      !std::isfinite(relative_velocity_ned[axis]) ||
      !std::isfinite(relative_acceleration_ned[axis]))
    {
      return output;
    }
  }

  const double range = std::sqrt(dot(relative_position_ned, relative_position_ned));
  if (range <= 1e-3) {return output;}
  const std::array<double, 3> line_of_sight{
    relative_position_ned[0] / range,
    relative_position_ned[1] / range,
    relative_position_ned[2] / range};
  output.closing_speed_mps = -dot(relative_velocity_ned, line_of_sight);
  const auto angular_momentum = cross(relative_position_ned, relative_velocity_ned);
  const std::array<double, 3> line_of_sight_rate_vector{
    angular_momentum[0] / (range * range),
    angular_momentum[1] / (range * range),
    angular_momentum[2] / (range * range)};
  output.line_of_sight_rate_rad_s =
    std::sqrt(dot(line_of_sight_rate_vector, line_of_sight_rate_vector));

  const auto normal_direction = cross(line_of_sight_rate_vector, line_of_sight);
  const double effective_closing_speed =
    std::max(config_.minimum_closing_speed_mps, output.closing_speed_mps);
  const double relative_acceleration_along_los =
    dot(relative_acceleration_ned, line_of_sight);
  std::array<double, 3> command{};
  for (int axis = 0; axis < 3; ++axis) {
    const double transverse_relative_acceleration =
      relative_acceleration_ned[axis] -
      relative_acceleration_along_los * line_of_sight[axis];
    command[axis] =
      config_.line_of_sight_acceleration_mps2 * line_of_sight[axis] +
      config_.navigation_constant * effective_closing_speed * normal_direction[axis] +
      config_.acceleration_feedforward_gain * transverse_relative_acceleration;
  }

  const double horizontal_norm = std::hypot(command[0], command[1]);
  if (horizontal_norm > config_.maximum_horizontal_acceleration_mps2) {
    const double scale = config_.maximum_horizontal_acceleration_mps2 / horizontal_norm;
    command[0] *= scale;
    command[1] *= scale;
    output.acceleration_limited = true;
  }
  const double unclamped_vertical = command[2];
  command[2] = std::clamp(
    command[2], -config_.maximum_vertical_acceleration_mps2,
    config_.maximum_vertical_acceleration_mps2);
  output.acceleration_limited =
    output.acceleration_limited || command[2] != unclamped_vertical;
  output.acceleration.ax = command[0];
  output.acceleration.ay = command[1];
  output.acceleration.az = command[2];
  output.acceleration.yaw =
    std::hypot(line_of_sight[0], line_of_sight[1]) > 1e-6 ?
    std::atan2(line_of_sight[1], line_of_sight[0]) : 0.0;
  output.valid = true;
  return output;
}

}  // namespace px4_offboard_tracking
