#include "px4_offboard_tracking/mpc_guidance.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4_offboard_tracking
{

MpcGuidance::MpcGuidance(const MpcGuidanceConfig & config)
: config_(config)
{
  if (config_.time_step_s <= 0.0 || config_.horizon_steps < 1 ||
    config_.position_weight <= 0.0 || config_.velocity_weight <= 0.0 ||
    config_.acceleration_weight <= 0.0 || config_.terminal_weight_multiplier < 1.0 ||
    config_.prediction_lookahead_s < 0.0 ||
    config_.maximum_horizontal_acceleration_mps2 <= 0.0 ||
    config_.minimum_target_above_m <= 0.0 || config_.desired_vertical_separation_m <= 0.0 ||
    config_.maximum_vertical_acceleration_mps2 <= 0.0 ||
    config_.vertical_position_weight <= 0.0 || config_.vertical_velocity_weight <= 0.0 ||
    config_.vertical_acceleration_weight <= 0.0)
  {
    throw std::invalid_argument("invalid MPC guidance configuration");
  }
}

std::array<double, 2> MpcGuidance::feedback_gain(
  double position_weight, double velocity_weight, double acceleration_weight) const
{
  const double dt = config_.time_step_s;
  const double b0 = -0.5 * dt * dt;
  const double b1 = -dt;
  double p00 = config_.terminal_weight_multiplier * position_weight;
  double p01 = 0.0;
  double p11 = config_.terminal_weight_multiplier * velocity_weight;
  std::array<double, 2> gain{};

  // Backward Riccati recursion for the finite prediction horizon. The same
  // model and weights apply on North and East, so one scalar-input gain is reused.
  for (int step = config_.horizon_steps - 1; step >= 0; --step) {
    const double bt_p_0 = b0 * p00 + b1 * p01;
    const double bt_p_1 = b0 * p01 + b1 * p11;
    const double denominator = acceleration_weight +
      bt_p_0 * b0 + bt_p_1 * b1;
    const double bt_p_a0 = bt_p_0;
    const double bt_p_a1 = bt_p_0 * dt + bt_p_1;
    gain = {bt_p_a0 / denominator, bt_p_a1 / denominator};

    const double pa00 = p00;
    const double pa01 = p00 * dt + p01;
    const double pa11 = p01 * dt + p11;
    const double atpa00 = pa00;
    const double atpa01 = pa01;
    const double atpa11 = dt * pa01 + pa11;
    const double atpb0 = bt_p_a0;
    const double atpb1 = dt * bt_p_0 + bt_p_1;
    p00 = position_weight + atpa00 - atpb0 * gain[0];
    p01 = atpa01 - atpb0 * gain[1];
    p11 = velocity_weight + atpa11 - atpb1 * gain[1];
  }
  return gain;
}

MpcGuidanceResult MpcGuidance::update(
  const std::array<double, 3> & relative_position_ned,
  const std::array<double, 3> & relative_velocity_ned,
  const std::array<double, 3> & relative_acceleration_ned) const
{
  MpcGuidanceResult output;
  for (int axis = 0; axis < 3; ++axis) {
    if (!std::isfinite(relative_position_ned[axis]) ||
      !std::isfinite(relative_velocity_ned[axis]))
    {
      return output;
    }
    if (!std::isfinite(relative_acceleration_ned[axis])) {return output;}
  }
  if (relative_position_ned[2] >= -config_.minimum_target_above_m) {return output;}

  const auto gain = feedback_gain(
    config_.position_weight, config_.velocity_weight, config_.acceleration_weight);
  const auto vertical_gain = feedback_gain(
    config_.vertical_position_weight, config_.vertical_velocity_weight,
    config_.vertical_acceleration_weight);
  // Use the IMM acceleration estimate to move the regulation point ahead of
  // a turning target. Limit the look-ahead so noisy acceleration cannot
  // dominate the feedback state.
  const double intercept_time = std::clamp(
    config_.prediction_lookahead_s, 0.0,
    config_.time_step_s * static_cast<double>(config_.horizon_steps));
  std::array<double, 3> predicted_position{};
  std::array<double, 3> predicted_velocity{};
  for (int axis = 0; axis < 3; ++axis) {
    predicted_position[axis] = relative_position_ned[axis] +
      relative_velocity_ned[axis] * intercept_time +
      0.5 * relative_acceleration_ned[axis] * intercept_time * intercept_time;
    predicted_velocity[axis] = relative_velocity_ned[axis] +
      relative_acceleration_ned[axis] * intercept_time;
  }
  output.acceleration.ax =
    -(gain[0] * predicted_position[0] + gain[1] * predicted_velocity[0]);
  output.acceleration.ay =
    -(gain[0] * predicted_position[1] + gain[1] * predicted_velocity[1]);
  const double vertical_error =
    predicted_position[2] + config_.desired_vertical_separation_m;
  output.acceleration.az =
    -(vertical_gain[0] * vertical_error + vertical_gain[1] * predicted_velocity[2]);
  const double acceleration_norm = std::hypot(
    output.acceleration.ax, output.acceleration.ay);
  if (acceleration_norm > config_.maximum_horizontal_acceleration_mps2) {
    const double scale = config_.maximum_horizontal_acceleration_mps2 / acceleration_norm;
    output.acceleration.ax *= scale;
    output.acceleration.ay *= scale;
    output.acceleration_limited = true;
  }
  const double unclamped_vertical_acceleration = output.acceleration.az;
  output.acceleration.az = std::clamp(
    output.acceleration.az, -config_.maximum_vertical_acceleration_mps2,
    config_.maximum_vertical_acceleration_mps2);
  output.vertical_acceleration_limited =
    output.acceleration.az != unclamped_vertical_acceleration;

  const double horizon_s = config_.time_step_s * config_.horizon_steps;
  output.predicted_terminal_error_ne = {
    relative_position_ned[0] + relative_velocity_ned[0] * horizon_s -
      0.5 * output.acceleration.ax * horizon_s * horizon_s,
    relative_position_ned[1] + relative_velocity_ned[1] * horizon_s -
      0.5 * output.acceleration.ay * horizon_s * horizon_s};
  const double predicted_n = relative_position_ned[0] +
    relative_velocity_ned[0] * std::min(horizon_s, 0.8);
  const double predicted_e = relative_position_ned[1] +
    relative_velocity_ned[1] * std::min(horizon_s, 0.8);
  output.acceleration.yaw = std::hypot(predicted_n, predicted_e) > 1e-6 ?
    std::atan2(predicted_e, predicted_n) : 0.0;
  output.valid = true;
  return output;
}

}  // namespace px4_offboard_tracking
