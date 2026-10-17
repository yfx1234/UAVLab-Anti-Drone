#pragma once

#include <array>
#include <cstddef>
#include <limits>

namespace px4_offboard_tracking
{

struct LaunchPositionPlannerConfig
{
  double minimum_horizon_s{0.5};
  double maximum_horizon_s{8.0};
  double horizon_step_s{0.1};
  double minimum_launch_range_m{17.0};
  double maximum_launch_range_m{30.0};
  double preferred_launch_range_m{25.0};
  double launch_range_step_m{1.0};
  double maximum_horizontal_acceleration_mps2{2.5};
  double maximum_vertical_acceleration_mps2{1.5};
  double maximum_speed_mps{12.0};
  double maximum_tilt_rad{0.52};
  double immediate_launch_maximum_axis_error_rad{0.03490658503988659};
  double maximum_target_acceleration_mps2{4.0};
  double time_weight{0.05};
  double acceleration_weight{1.0};
  double acceleration_change_weight{0.2};
  double range_weight{0.01};
  // Zero deliberately means that a launch opportunity does not require
  // target-speed matching. A positive value is available for later studies.
  double terminal_relative_speed_weight{0.0};
  // Cost normalization must not use maximum_speed_mps: the latter is a hard
  // capability constraint and changing it must not silently retune the cost.
  double terminal_relative_speed_scale_mps{4.0};
  // Prefer only the speed needed to follow the estimated target plus a fixed
  // closure reserve. This is a soft cost, not a common-speed requirement, so
  // a time-critical encounter can still use the full aircraft capability.
  double closing_speed_reserve_mps{4.0};
  double excess_speed_weight{0.2};
  double excess_speed_scale_mps{2.0};
};

struct LaunchPositionPlannerInput
{
  // Target minus pursuer in common NED.
  std::array<double, 3> relative_position_ned{};
  std::array<double, 3> relative_velocity_ned{};
  std::array<double, 3> target_acceleration_ned{};
  std::array<double, 3> pursuer_velocity_ned{};
  std::array<double, 3> previous_acceleration_ned{};
  double target_acceleration_confidence{0.0};
  // Runtime limits describe what the downstream controller can execute. They
  // are intentionally separate from the planner's airframe capability limit.
  double maximum_executable_horizontal_speed_mps{
    std::numeric_limits<double>::infinity()};
  double maximum_executable_vertical_speed_mps{
    std::numeric_limits<double>::infinity()};
};

struct LaunchPositionPlannerResult
{
  std::array<double, 3> acceleration_ned{};
  std::array<double, 3> predicted_target_displacement_ned{};
  std::array<double, 3> predicted_pursuer_displacement_ned{};
  std::array<double, 3> predicted_relative_position_ned{};
  std::array<double, 3> predicted_relative_velocity_ned{};
  double horizon_s{};
  double launch_range_m{};
  double predicted_speed_mps{};
  double required_tilt_rad{};
  double cost{std::numeric_limits<double>::infinity()};
  std::size_t feasible_candidate_count{};
  bool valid{false};
};

// Receding-horizon planner for the aircraft launch position. It predicts a
// target state, places the pursuer below that future target at a valid firing
// range, and solves the constant-acceleration reachability equation. It does
// not constrain terminal relative velocity, so inbound, crossing and tail
// chase encounters share the same algorithm.
class LaunchPositionPlanner
{
public:
  explicit LaunchPositionPlanner(const LaunchPositionPlannerConfig & config);

  LaunchPositionPlannerResult plan(
    const LaunchPositionPlannerInput & input) const;

  const LaunchPositionPlannerConfig & config() const {return config_;}

private:
  LaunchPositionPlannerConfig config_;
};

}  // namespace px4_offboard_tracking
