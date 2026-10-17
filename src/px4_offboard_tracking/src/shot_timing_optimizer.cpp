#include "px4_offboard_tracking/shot_timing_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4_offboard_tracking
{

ShotTimingOptimizer::ShotTimingOptimizer(const ShotTimingConfig & config)
: config_(config)
{
  if (!std::isfinite(config_.preferred_axis_error_rad) ||
    !std::isfinite(config_.hard_axis_error_rad) ||
    !std::isfinite(config_.prediction_horizon_s) ||
    !std::isfinite(config_.error_rate_filter_time_constant_s) ||
    !std::isfinite(config_.maximum_absolute_error_rate_rad_s) ||
    !std::isfinite(config_.minimum_ready_hold_s) ||
    config_.preferred_axis_error_rad <= 0.0 ||
    config_.hard_axis_error_rad < config_.preferred_axis_error_rad ||
    config_.prediction_horizon_s < 0.0 ||
    config_.error_rate_filter_time_constant_s <= 0.0 ||
    config_.maximum_absolute_error_rate_rad_s <= 0.0 ||
    config_.minimum_ready_hold_s < 0.0)
  {
    throw std::invalid_argument("invalid shot timing configuration");
  }
}

void ShotTimingOptimizer::reset()
{
  initialized_ = false;
  previous_time_s_ = 0.0;
  previous_error_rad_ = 0.0;
  filtered_error_rate_rad_s_ = 0.0;
  ready_since_s_ = -1.0;
}

ShotTimingResult ShotTimingOptimizer::update(const ShotTimingInput & input)
{
  ShotTimingResult output;
  output.axis_error_rad = input.axis_error_rad;
  if (!std::isfinite(input.axis_error_rad) || input.axis_error_rad < 0.0 ||
    !std::isfinite(input.now_s))
  {
    reset();
    return output;
  }

  if (!initialized_) {
    initialized_ = true;
    previous_time_s_ = input.now_s;
    previous_error_rad_ = input.axis_error_rad;
  } else {
    const double dt = input.now_s - previous_time_s_;
    if (dt > 1e-4 && dt < 0.5) {
      const double raw_rate = (input.axis_error_rad - previous_error_rad_) / dt;
      const double alpha = dt / (config_.error_rate_filter_time_constant_s + dt);
      filtered_error_rate_rad_s_ +=
        alpha * (raw_rate - filtered_error_rate_rad_s_);
    } else if (dt < 0.0 || dt >= 0.5) {
      filtered_error_rate_rad_s_ = 0.0;
      ready_since_s_ = -1.0;
    }
    previous_time_s_ = input.now_s;
    previous_error_rad_ = input.axis_error_rad;
  }

  output.valid = true;
  output.axis_error_rate_rad_s = filtered_error_rate_rad_s_;
  output.predicted_axis_error_rad = std::max(
    0.0,
    input.axis_error_rad +
    filtered_error_rate_rad_s_ * config_.prediction_horizon_s);
  output.score = std::clamp(
    1.0 - output.predicted_axis_error_rad /
    config_.preferred_axis_error_rad,
    0.0, 1.0);

  const bool preferred_window =
    input.base_gate_passed &&
    input.axis_error_rad <= config_.preferred_axis_error_rad &&
    output.predicted_axis_error_rad <= config_.preferred_axis_error_rad &&
    std::abs(filtered_error_rate_rad_s_) <=
    config_.maximum_absolute_error_rate_rad_s;
  if (!preferred_window) {
    ready_since_s_ = -1.0;
    return output;
  }
  if (ready_since_s_ < 0.0) {
    ready_since_s_ = input.now_s;
  }
  output.ready =
    input.now_s - ready_since_s_ >= config_.minimum_ready_hold_s;
  return output;
}

}  // namespace px4_offboard_tracking
