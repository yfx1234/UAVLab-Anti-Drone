#include "px4_offboard_tracking/safety_manager.hpp"

#include <cmath>
#include <stdexcept>

namespace px4_offboard_tracking
{

SafetyManager::SafetyManager(const SafetyManagerConfig & config)
: config_(config)
{
  if (config_.minimum_range_m <= 0.0 || config_.maximum_tilt_rad <= 0.0 ||
    config_.maximum_speed_mps <= 0.0 || config_.maximum_altitude_error_m <= 0.0 ||
    config_.recovery_maximum_tilt_rad < 0.0 || config_.recovery_maximum_speed_mps < 0.0 ||
    config_.recovery_maximum_altitude_error_m < 0.0 ||
    config_.recovery_required_hold_time_s <= 0.0 ||
    config_.recovery_maximum_tilt_rad > config_.maximum_tilt_rad ||
    config_.recovery_maximum_speed_mps > config_.maximum_speed_mps ||
    config_.recovery_maximum_altitude_error_m > config_.maximum_altitude_error_m)
  {
    throw std::invalid_argument("invalid safety manager configuration");
  }
}

void SafetyManager::reset()
{
  timestamp_initialized_ = false;
  previous_timestamp_s_ = 0.0;
  recovery_hold_active_ = false;
  recovery_hold_started_s_ = 0.0;
}

SafetyManagerResult SafetyManager::update(const SafetyManagerInput & input)
{
  SafetyManagerResult output;
  const bool timestamp_ok = std::isfinite(input.timestamp_s) &&
    (!timestamp_initialized_ || input.timestamp_s >= previous_timestamp_s_);
  if (timestamp_ok) {
    timestamp_initialized_ = true;
    previous_timestamp_s_ = input.timestamp_s;
  }

  output.local_state_ok = input.local_state_valid;
  output.attitude_ok = input.attitude_valid;
  output.range_ok = !input.range_valid ||
    (std::isfinite(input.range_m) && input.range_m >= config_.minimum_range_m);
  output.tilt_ok = std::isfinite(input.tilt_rad) &&
    input.tilt_rad <= config_.maximum_tilt_rad;
  output.speed_ok = std::isfinite(input.speed_mps) &&
    input.speed_mps <= config_.maximum_speed_mps;
  output.altitude_ok = std::isfinite(input.altitude_error_m) &&
    input.altitude_error_m <= config_.maximum_altitude_error_m;

  const bool safety_pass = timestamp_ok && output.local_state_ok && output.attitude_ok &&
    output.range_ok && output.tilt_ok && output.speed_ok && output.altitude_ok;
  output.abort_requested = input.monitoring_active && !safety_pass;
  if (!timestamp_ok) {
    output.reason = "non-monotonic or invalid timestamp";
  } else if (!output.local_state_ok) {
    output.reason = "local state invalid or stale";
  } else if (!output.attitude_ok) {
    output.reason = "attitude invalid or stale";
  } else if (!output.range_ok) {
    output.reason = "minimum range violated";
  } else if (!output.tilt_ok) {
    output.reason = "tilt limit exceeded";
  } else if (!output.speed_ok) {
    output.reason = "speed limit exceeded";
  } else if (!output.altitude_ok) {
    output.reason = "altitude error limit exceeded";
  } else {
    output.reason = input.monitoring_active ? "safety conditions satisfied" : "monitoring inactive";
  }

  output.recovery_instantaneous_pass = timestamp_ok && output.local_state_ok &&
    output.attitude_ok && output.range_ok && std::isfinite(input.tilt_rad) &&
    input.tilt_rad <= config_.recovery_maximum_tilt_rad &&
    std::isfinite(input.speed_mps) &&
    input.speed_mps <= config_.recovery_maximum_speed_mps &&
    std::isfinite(input.altitude_error_m) &&
    input.altitude_error_m <= config_.recovery_maximum_altitude_error_m;

  if (!input.recovery_active || !output.recovery_instantaneous_pass) {
    recovery_hold_active_ = false;
    recovery_hold_started_s_ = 0.0;
  } else if (!recovery_hold_active_) {
    recovery_hold_active_ = true;
    recovery_hold_started_s_ = input.timestamp_s;
  }
  if (recovery_hold_active_) {
    output.recovery_elapsed_s = input.timestamp_s - recovery_hold_started_s_;
    output.recovery_complete =
      output.recovery_elapsed_s >= config_.recovery_required_hold_time_s;
  }
  return output;
}

}  // namespace px4_offboard_tracking
