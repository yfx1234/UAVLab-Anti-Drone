#pragma once

#include <array>

#include "px4_offboard_tracking/acceleration_guidance.hpp"

namespace px4_offboard_tracking
{

struct MpcGuidanceConfig
{
  double time_step_s{0.1};
  int horizon_steps{25};
  double position_weight{1.0};
  double velocity_weight{2.0};
  double acceleration_weight{0.4};
  double terminal_weight_multiplier{4.0};
  double prediction_lookahead_s{0.5};
  double maximum_horizontal_acceleration_mps2{1.5};
  double minimum_target_above_m{0.5};
  double desired_vertical_separation_m{30.0};
  double maximum_vertical_acceleration_mps2{1.0};
  double vertical_position_weight{0.25};
  double vertical_velocity_weight{6.0};
  double vertical_acceleration_weight{4.0};
};

struct MpcGuidanceResult
{
  AccelerationCommand acceleration{};
  std::array<double, 2> predicted_terminal_error_ne{};
  bool acceleration_limited{false};
  bool vertical_acceleration_limited{false};
  bool valid{false};
};

// Finite-horizon linear MPC for relative position and velocity.
// The target state is advanced over a bounded look-ahead using the
// IMM-EKF-estimated velocity and acceleration before horizon feedback.
// Applying pursuer acceleration u gives r(k+1)=r(k)+v(k)dt-0.5*u*dt^2.
// Horizontal axes converge to zero offset; Down converges to the configured
// negative vertical separation (target above pursuer).
class MpcGuidance
{
public:
  explicit MpcGuidance(const MpcGuidanceConfig & config);

  MpcGuidanceResult update(
    const std::array<double, 3> & relative_position_ned,
    const std::array<double, 3> & relative_velocity_ned,
    const std::array<double, 3> & relative_acceleration_ned = {0.0, 0.0, 0.0}) const;

private:
  std::array<double, 2> feedback_gain(
    double position_weight, double velocity_weight, double acceleration_weight) const;

  MpcGuidanceConfig config_;
};

}  // namespace px4_offboard_tracking
