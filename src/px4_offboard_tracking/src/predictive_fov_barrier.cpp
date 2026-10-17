#include "px4_offboard_tracking/predictive_fov_barrier.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4_offboard_tracking
{

PredictiveFovBarrier::PredictiveFovBarrier(
  const PredictiveFovBarrierConfig & config)
: config_(config)
{
  if (config_.normal_boundary_rad <= 0.0 ||
    config_.centering_boundary_rad <= config_.normal_boundary_rad ||
    config_.hard_boundary_rad <= config_.centering_boundary_rad ||
    config_.recovery_exit_rad <= config_.normal_boundary_rad ||
    config_.recovery_exit_rad >= config_.centering_boundary_rad ||
    config_.prediction_horizon_s <= 0.0 ||
    config_.angular_rate_filter_time_constant_s <= 0.0)
  {
    throw std::invalid_argument("invalid predictive FOV barrier configuration");
  }
}

PredictiveFovBarrierResult PredictiveFovBarrier::update(
  const PredictiveFovBarrierInput & input)
{
  PredictiveFovBarrierResult output;
  if (!input.depth_visible) {
    zone_ = input.imm_prediction_usable ?
      FovBarrierZone::IMM_COAST : FovBarrierZone::LOST;
    output.zone = zone_;
    return output;
  }
  if (!std::isfinite(input.now_s) ||
    !std::isfinite(input.horizontal_error_rad) ||
    !std::isfinite(input.vertical_error_rad))
  {
    zone_ = FovBarrierZone::LOST;
    output.zone = zone_;
    return output;
  }

  if (initialized_ && input.observation_sequence != 0 &&
    input.observation_sequence != last_sequence_)
  {
    const double dt = input.now_s - last_time_s_;
    if (dt > 1.0e-4 && dt < 1.0) {
      const double alpha = dt /
        (config_.angular_rate_filter_time_constant_s + dt);
      const double horizontal_rate =
        (input.horizontal_error_rad - last_horizontal_error_rad_) / dt;
      const double vertical_rate =
        (input.vertical_error_rad - last_vertical_error_rad_) / dt;
      horizontal_rate_rad_s_ +=
        alpha * (horizontal_rate - horizontal_rate_rad_s_);
      vertical_rate_rad_s_ +=
        alpha * (vertical_rate - vertical_rate_rad_s_);
    }
  }
  if (!initialized_ || input.observation_sequence != last_sequence_) {
    last_time_s_ = input.now_s;
    last_horizontal_error_rad_ = input.horizontal_error_rad;
    last_vertical_error_rad_ = input.vertical_error_rad;
    last_sequence_ = input.observation_sequence;
    initialized_ = true;
  }

  output.horizontal_rate_rad_s = horizontal_rate_rad_s_;
  output.vertical_rate_rad_s = vertical_rate_rad_s_;
  output.predicted_horizontal_error_rad =
    input.horizontal_error_rad +
    config_.prediction_horizon_s * horizontal_rate_rad_s_;
  output.predicted_vertical_error_rad =
    input.vertical_error_rad +
    config_.prediction_horizon_s * vertical_rate_rad_s_;
  output.predicted_maximum_error_rad = std::max({
    std::abs(input.horizontal_error_rad),
    std::abs(input.vertical_error_rad),
    std::abs(output.predicted_horizontal_error_rad),
    std::abs(output.predicted_vertical_error_rad)});

  if (zone_ == FovBarrierZone::RECOVERY &&
    output.predicted_maximum_error_rad >= config_.recovery_exit_rad)
  {
    zone_ = FovBarrierZone::RECOVERY;
  } else if (output.predicted_maximum_error_rad >=
    config_.centering_boundary_rad)
  {
    zone_ = FovBarrierZone::RECOVERY;
  } else if (output.predicted_maximum_error_rad >=
    config_.normal_boundary_rad)
  {
    zone_ = FovBarrierZone::CENTERING;
  } else {
    zone_ = FovBarrierZone::NORMAL;
  }

  output.zone = zone_;
  output.centering_strength = std::clamp(
    (output.predicted_maximum_error_rad - config_.normal_boundary_rad) /
    (config_.centering_boundary_rad - config_.normal_boundary_rad),
    0.0, 1.0);
  output.pause_approach =
    zone_ == FovBarrierZone::RECOVERY ||
    zone_ == FovBarrierZone::IMM_COAST ||
    zone_ == FovBarrierZone::LOST;
  output.depth_admission_allowed =
    zone_ == FovBarrierZone::NORMAL ||
    zone_ == FovBarrierZone::CENTERING;
  output.recovery_exit_allowed =
    output.predicted_maximum_error_rad < config_.recovery_exit_rad;
  return output;
}

void PredictiveFovBarrier::reset()
{
  zone_ = FovBarrierZone::LOST;
  initialized_ = false;
  horizontal_rate_rad_s_ = 0.0;
  vertical_rate_rad_s_ = 0.0;
  last_sequence_ = 0;
}

const PredictiveFovBarrierConfig & PredictiveFovBarrier::config() const
{
  return config_;
}

}  // namespace px4_offboard_tracking
