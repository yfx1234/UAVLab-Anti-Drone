#include "px4_offboard_tracking/capture_window.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4_offboard_tracking
{

CaptureWindow::CaptureWindow(const CaptureWindowConfig & config)
: config_(config)
{
  if (config_.maximum_pixel_error_px <= 0.0 ||
    config_.maximum_alignment_error_rad <= 0.0 ||
    config_.maximum_los_rate_rad_s <= 0.0 ||
    config_.maximum_predicted_miss_distance_m <= 0.0 ||
    config_.maximum_time_to_closest_approach_s < 0.0 || config_.required_hold_time_s <= 0.0 ||
    config_.minimum_distinct_observation_frames <= 0)
  {
    throw std::invalid_argument("invalid capture window configuration");
  }
}

void CaptureWindow::reset()
{
  reset_hold();
  timestamp_initialized_ = false;
  last_timestamp_s_ = 0.0;
}

void CaptureWindow::reset_hold()
{
  holding_ = false;
  observation_timestamp_initialized_ = false;
  hold_started_s_ = 0.0;
  last_observation_timestamp_s_ = 0.0;
  distinct_observation_frames_ = 0;
}

CaptureWindowResult CaptureWindow::update(const CaptureWindowInput & input)
{
  CaptureWindowResult output;
  const bool timestamp_ok = std::isfinite(input.timestamp_s) &&
    (!timestamp_initialized_ || input.timestamp_s >= last_timestamp_s_);
  if (!timestamp_ok) {
    reset();
    return output;
  }
  last_timestamp_s_ = input.timestamp_s;
  timestamp_initialized_ = true;

  const bool observation_timestamp_valid = input.observation_valid &&
    std::isfinite(input.observation_timestamp_s) && input.observation_timestamp_s > 0.0;
  const bool observation_timestamp_ordered = observation_timestamp_valid &&
    (!observation_timestamp_initialized_ ||
    input.observation_timestamp_s >= last_observation_timestamp_s_);
  if (holding_ && !observation_timestamp_ordered) {
    reset_hold();
    return output;
  }

  output.quality_ok = input.quality_gate_passed;
  output.pixel_ok = std::isfinite(input.pixel_error_px) && input.pixel_error_px >= 0.0 &&
    input.pixel_error_px <= config_.maximum_pixel_error_px;
  output.alignment_ok = std::isfinite(input.alignment_error_rad) &&
    input.alignment_error_rad >= 0.0 &&
    input.alignment_error_rad <= config_.maximum_alignment_error_rad;
  // The terminal attack geometry is checked in common NED using horizontal
  // distance and vertical separation. Three-dimensional range is diagnostic only.
  output.range_ok = input.attack_geometry_valid;
  output.los_rate_ok = std::isfinite(input.los_rate_rad_s) && input.los_rate_rad_s >= 0.0 &&
    input.los_rate_rad_s <= config_.maximum_los_rate_rad_s;
  // Relative speed is intentionally not a capture gate in terminal interception.
  output.relative_speed_ok = true;
  output.prediction_ok = std::isfinite(input.predicted_miss_distance_m) &&
    std::isfinite(input.time_to_closest_approach_s) &&
    input.predicted_miss_distance_m >= 0.0 &&
    input.predicted_miss_distance_m <= config_.maximum_predicted_miss_distance_m &&
    input.time_to_closest_approach_s >= 0.0 &&
    input.time_to_closest_approach_s <= config_.maximum_time_to_closest_approach_s;
  output.instantaneous_pass = observation_timestamp_valid && output.quality_ok &&
    output.pixel_ok && output.alignment_ok &&
    output.range_ok && output.los_rate_ok && output.relative_speed_ok && output.prediction_ok;

  if (!output.instantaneous_pass) {
    reset_hold();
    return output;
  }
  if (!holding_) {
    holding_ = true;
    hold_started_s_ = input.timestamp_s;
  }
  constexpr double timestamp_epsilon_s = 1e-9;
  if (!observation_timestamp_initialized_ ||
    input.observation_timestamp_s > last_observation_timestamp_s_ + timestamp_epsilon_s)
  {
    ++distinct_observation_frames_;
    last_observation_timestamp_s_ = input.observation_timestamp_s;
    observation_timestamp_initialized_ = true;
  }
  output.hold_elapsed_s = std::max(0.0, input.timestamp_s - hold_started_s_);
  output.distinct_observation_frames = distinct_observation_frames_;
  output.frame_count_ok = distinct_observation_frames_ >=
    config_.minimum_distinct_observation_frames;
  constexpr double duration_epsilon_s = 1e-9;
  output.held = output.hold_elapsed_s + duration_epsilon_s >=
    config_.required_hold_time_s && output.frame_count_ok;
  return output;
}

}  // namespace px4_offboard_tracking
