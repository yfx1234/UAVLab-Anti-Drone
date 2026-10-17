#include "px4_offboard_tracking/launch_position_planner.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4_offboard_tracking
{
namespace
{

constexpr double gravity_mps2 = 9.80665;

bool finite(const std::array<double, 3> & value)
{
  return std::isfinite(value[0]) && std::isfinite(value[1]) &&
         std::isfinite(value[2]);
}

double norm(const std::array<double, 3> & value)
{
  return std::sqrt(
    value[0] * value[0] + value[1] * value[1] + value[2] * value[2]);
}

std::array<double, 3> limited(
  const std::array<double, 3> & value, double maximum)
{
  const double length = norm(value);
  if (length <= maximum || length < 1.0e-9) {
    return value;
  }
  const double scale = maximum / length;
  return {scale * value[0], scale * value[1], scale * value[2]};
}

}  // namespace

LaunchPositionPlanner::LaunchPositionPlanner(
  const LaunchPositionPlannerConfig & config)
: config_(config)
{
  if (config_.minimum_horizon_s <= 0.0 ||
    config_.maximum_horizon_s < config_.minimum_horizon_s ||
    config_.horizon_step_s <= 0.0 ||
    config_.minimum_launch_range_m <= 0.0 ||
    config_.maximum_launch_range_m < config_.minimum_launch_range_m ||
    config_.preferred_launch_range_m < config_.minimum_launch_range_m ||
    config_.preferred_launch_range_m > config_.maximum_launch_range_m ||
    config_.launch_range_step_m <= 0.0 ||
    config_.maximum_horizontal_acceleration_mps2 <= 0.0 ||
    config_.maximum_vertical_acceleration_mps2 <= 0.0 ||
    config_.maximum_speed_mps <= 0.0 ||
    config_.maximum_tilt_rad <= 0.0 ||
    config_.maximum_tilt_rad >= 1.5707963267948966 ||
    config_.immediate_launch_maximum_axis_error_rad <= 0.0 ||
    config_.immediate_launch_maximum_axis_error_rad >= 1.5707963267948966 ||
    config_.maximum_target_acceleration_mps2 < 0.0 ||
    config_.time_weight < 0.0 || config_.acceleration_weight < 0.0 ||
    config_.acceleration_change_weight < 0.0 || config_.range_weight < 0.0 ||
    config_.terminal_relative_speed_weight < 0.0 ||
    config_.terminal_relative_speed_scale_mps <= 0.0 ||
    config_.closing_speed_reserve_mps < 0.0 ||
    config_.excess_speed_weight < 0.0 ||
    config_.excess_speed_scale_mps <= 0.0)
  {
    throw std::invalid_argument("invalid launch-position planner configuration");
  }
}

LaunchPositionPlannerResult LaunchPositionPlanner::plan(
  const LaunchPositionPlannerInput & input) const
{
  LaunchPositionPlannerResult best;
  if (!finite(input.relative_position_ned) ||
    !finite(input.relative_velocity_ned) ||
    !finite(input.target_acceleration_ned) ||
    !finite(input.pursuer_velocity_ned) ||
    !finite(input.previous_acceleration_ned) ||
    !std::isfinite(input.target_acceleration_confidence) ||
    input.target_acceleration_confidence < 0.0 ||
    input.target_acceleration_confidence > 1.0 ||
    std::isnan(input.maximum_executable_horizontal_speed_mps) ||
    input.maximum_executable_horizontal_speed_mps <= 0.0 ||
    std::isnan(input.maximum_executable_vertical_speed_mps) ||
    input.maximum_executable_vertical_speed_mps <= 0.0)
  {
    return best;
  }

  // A receding-horizon search must not step over a launch opportunity that
  // already exists at the current sample.  The aircraft is below the target
  // when relative D is negative.  In that narrow corridor, hold inertial
  // acceleration at zero and let the existing ballistic solver/timing gate
  // decide whether to fire; do not chase a later common-position solution.
  const double current_range = norm(input.relative_position_ned);
  const double current_axis_error = std::atan2(
    std::hypot(input.relative_position_ned[0], input.relative_position_ned[1]),
    -input.relative_position_ned[2]);
  if (input.relative_position_ned[2] < 0.0 &&
    current_range >= config_.minimum_launch_range_m &&
    current_range <= config_.maximum_launch_range_m &&
    current_axis_error <= config_.immediate_launch_maximum_axis_error_rad)
  {
    best.acceleration_ned = {};
    best.predicted_target_displacement_ned = input.relative_position_ned;
    best.predicted_pursuer_displacement_ned = {};
    best.predicted_relative_position_ned = input.relative_position_ned;
    best.predicted_relative_velocity_ned = input.relative_velocity_ned;
    best.horizon_s = 0.0;
    best.launch_range_m = current_range;
    best.predicted_speed_mps = norm(input.pursuer_velocity_ned);
    best.required_tilt_rad = 0.0;
    best.cost = 0.0;
    best.feasible_candidate_count = 1U;
    best.valid = true;
    return best;
  }

  auto target_acceleration = limited(
    input.target_acceleration_ned,
    config_.maximum_target_acceleration_mps2);
  for (double & component : target_acceleration) {
    component *= input.target_acceleration_confidence;
  }

  const double horizontal_acceleration_scale =
    config_.maximum_horizontal_acceleration_mps2;
  const double vertical_acceleration_scale =
    config_.maximum_vertical_acceleration_mps2;
  const std::array<double, 3> target_velocity{
    input.pursuer_velocity_ned[0] + input.relative_velocity_ned[0],
    input.pursuer_velocity_ned[1] + input.relative_velocity_ned[1],
    input.pursuer_velocity_ned[2] + input.relative_velocity_ned[2]};
  const double target_horizontal_speed = std::hypot(
    target_velocity[0], target_velocity[1]);
  const double executable_horizontal_speed = std::min(
    config_.maximum_speed_mps,
    input.maximum_executable_horizontal_speed_mps);
  const double preferred_horizontal_speed = std::min(
    executable_horizontal_speed,
    target_horizontal_speed + config_.closing_speed_reserve_mps);
  const double range_span = std::max(
    1.0, config_.maximum_launch_range_m - config_.minimum_launch_range_m);

  const std::size_t horizon_count = static_cast<std::size_t>(std::floor(
    (config_.maximum_horizon_s - config_.minimum_horizon_s) /
    config_.horizon_step_s + 1.0e-9));
  const std::size_t range_count = static_cast<std::size_t>(std::floor(
    (config_.maximum_launch_range_m - config_.minimum_launch_range_m) /
    config_.launch_range_step_m + 1.0e-9));

  for (std::size_t horizon_index = 0; horizon_index <= horizon_count; ++horizon_index) {
    const double horizon = std::min(
      config_.maximum_horizon_s,
      config_.minimum_horizon_s +
      static_cast<double>(horizon_index) * config_.horizon_step_s);
    const double horizon_squared = horizon * horizon;
    for (std::size_t range_index = 0; range_index <= range_count; ++range_index) {
      const double launch_range = std::min(
        config_.maximum_launch_range_m,
        config_.minimum_launch_range_m +
        static_cast<double>(range_index) * config_.launch_range_step_m);

      std::array<double, 3> target_displacement{};
      for (std::size_t axis = 0; axis < 3; ++axis) {
        const double target_velocity =
          input.pursuer_velocity_ned[axis] + input.relative_velocity_ned[axis];
        target_displacement[axis] = input.relative_position_ned[axis] +
          target_velocity * horizon +
          0.5 * target_acceleration[axis] * horizon_squared;
      }
      // NED D grows downward. A positive D offset places the pursuer below
      // the predicted target, leaving target-minus-pursuer = [0, 0, -range].
      std::array<double, 3> launch_displacement = target_displacement;
      launch_displacement[2] += launch_range;

      std::array<double, 3> acceleration{};
      for (std::size_t axis = 0; axis < 3; ++axis) {
        acceleration[axis] = 2.0 * (
          launch_displacement[axis] -
          input.pursuer_velocity_ned[axis] * horizon) / horizon_squared;
      }
      if (!finite(acceleration)) {
        continue;
      }
      const double horizontal_acceleration = std::hypot(
        acceleration[0], acceleration[1]);
      if (horizontal_acceleration >
        config_.maximum_horizontal_acceleration_mps2 + 1.0e-9 ||
        std::abs(acceleration[2]) >
        config_.maximum_vertical_acceleration_mps2 + 1.0e-9)
      {
        continue;
      }
      const double vertical_specific_force = gravity_mps2 - acceleration[2];
      if (vertical_specific_force <= 1.0e-6) {
        continue;
      }
      const double required_tilt = std::atan2(
        horizontal_acceleration, vertical_specific_force);
      if (required_tilt > config_.maximum_tilt_rad + 1.0e-9) {
        continue;
      }

      std::array<double, 3> predicted_pursuer_velocity{};
      for (std::size_t axis = 0; axis < 3; ++axis) {
        predicted_pursuer_velocity[axis] =
          input.pursuer_velocity_ned[axis] + acceleration[axis] * horizon;
      }
      const double predicted_speed = norm(predicted_pursuer_velocity);
      const double predicted_horizontal_speed = std::hypot(
        predicted_pursuer_velocity[0], predicted_pursuer_velocity[1]);
      if (predicted_speed > config_.maximum_speed_mps + 1.0e-9 ||
        predicted_horizontal_speed > executable_horizontal_speed + 1.0e-9 ||
        std::abs(predicted_pursuer_velocity[2]) >
        input.maximum_executable_vertical_speed_mps + 1.0e-9)
      {
        continue;
      }

      std::array<double, 3> predicted_relative_velocity{};
      for (std::size_t axis = 0; axis < 3; ++axis) {
        predicted_relative_velocity[axis] =
          input.relative_velocity_ned[axis] +
          target_acceleration[axis] * horizon - acceleration[axis] * horizon;
      }
      const std::array<double, 3> predicted_relative_position{
        0.0, 0.0, -launch_range};

      const double horizontal_effort =
        horizontal_acceleration / horizontal_acceleration_scale;
      const double vertical_effort =
        acceleration[2] / vertical_acceleration_scale;
      const std::array<double, 3> acceleration_change{
        acceleration[0] - input.previous_acceleration_ned[0],
        acceleration[1] - input.previous_acceleration_ned[1],
        acceleration[2] - input.previous_acceleration_ned[2]};
      const double change_effort = norm(acceleration_change) /
        std::max(horizontal_acceleration_scale, vertical_acceleration_scale);
      const double range_error =
        (launch_range - config_.preferred_launch_range_m) / range_span;
      const double terminal_relative_speed = norm(predicted_relative_velocity) /
        config_.terminal_relative_speed_scale_mps;
      const double excess_speed = std::max(
        0.0, predicted_horizontal_speed - preferred_horizontal_speed) /
        config_.excess_speed_scale_mps;
      const double cost =
        config_.time_weight * horizon +
        config_.acceleration_weight *
        (horizontal_effort * horizontal_effort + vertical_effort * vertical_effort) +
        config_.acceleration_change_weight * change_effort * change_effort +
        config_.range_weight * range_error * range_error +
        config_.terminal_relative_speed_weight *
        terminal_relative_speed * terminal_relative_speed +
        config_.excess_speed_weight * excess_speed * excess_speed;
      ++best.feasible_candidate_count;
      if (!std::isfinite(cost) || cost >= best.cost) {
        continue;
      }

      best.acceleration_ned = acceleration;
      best.predicted_target_displacement_ned = target_displacement;
      best.predicted_pursuer_displacement_ned = launch_displacement;
      best.predicted_relative_position_ned = predicted_relative_position;
      best.predicted_relative_velocity_ned = predicted_relative_velocity;
      best.horizon_s = horizon;
      best.launch_range_m = launch_range;
      best.predicted_speed_mps = predicted_speed;
      best.required_tilt_rad = required_tilt;
      best.cost = cost;
      best.valid = true;
    }
  }
  return best;
}

}  // namespace px4_offboard_tracking
