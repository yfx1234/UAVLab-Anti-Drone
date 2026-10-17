#include "px4_offboard_tracking/simple_below_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4_offboard_tracking
{
namespace
{

double norm(const std::array<double, 2> & value)
{
  return std::hypot(value[0], value[1]);
}

std::array<double, 2> limited(
  const std::array<double, 2> & value, double maximum_norm)
{
  const double magnitude = norm(value);
  if (!std::isfinite(magnitude) || magnitude <= maximum_norm || magnitude < 1e-12) {
    return value;
  }
  const double scale = maximum_norm / magnitude;
  return {value[0] * scale, value[1] * scale};
}

bool finite(const std::array<double, 2> & value)
{
  return std::isfinite(value[0]) && std::isfinite(value[1]);
}

}  // namespace

SimpleBelowTrackingMode parse_simple_below_tracking_mode(const std::string & value)
{
  if (value == "direct") {
    return SimpleBelowTrackingMode::DIRECT;
  }
  if (value == "predictive") {
    return SimpleBelowTrackingMode::PREDICTIVE;
  }
  throw std::invalid_argument("tracking_mode must be 'direct' or 'predictive'");
}

const char * to_string(SimpleBelowTrackingMode mode)
{
  return mode == SimpleBelowTrackingMode::PREDICTIVE ? "predictive" : "direct";
}

SimpleBelowTracker::SimpleBelowTracker(const SimpleBelowTrackerConfig & config)
: config_(config)
{
  if (!std::isfinite(config_.horizontal_position_gain) ||
    config_.horizontal_position_gain <= 0.0 ||
    !std::isfinite(config_.prediction_horizon_s) ||
    config_.prediction_horizon_s < 0.0 ||
    !std::isfinite(config_.relative_velocity_filter_time_constant_s) ||
    config_.relative_velocity_filter_time_constant_s < 0.0 ||
    !std::isfinite(config_.minimum_observation_dt_s) ||
    config_.minimum_observation_dt_s <= 0.0 ||
    !std::isfinite(config_.maximum_observation_dt_s) ||
    config_.maximum_observation_dt_s <= config_.minimum_observation_dt_s ||
    !std::isfinite(config_.maximum_relative_speed_mps) ||
    config_.maximum_relative_speed_mps <= 0.0 ||
    !std::isfinite(config_.maximum_prediction_distance_m) ||
    config_.maximum_prediction_distance_m < 0.0 ||
    !std::isfinite(config_.maximum_horizontal_speed_mps) ||
    config_.maximum_horizontal_speed_mps <= 0.0 ||
    !std::isfinite(config_.maximum_horizontal_acceleration_mps2) ||
    config_.maximum_horizontal_acceleration_mps2 <= 0.0)
  {
    throw std::invalid_argument("invalid simple below tracker configuration");
  }
}

bool SimpleBelowTracker::update_observation(
  const std::array<double, 2> & relative_position_ned_m,
  double acquisition_time_s)
{
  if (!finite(relative_position_ned_m) || !std::isfinite(acquisition_time_s)) {
    return false;
  }

  if (previous_observation_valid_) {
    const double dt = acquisition_time_s - previous_acquisition_time_s_;
    if (dt <= 0.0) {
      return false;
    }
    if (dt >= config_.minimum_observation_dt_s &&
      dt <= config_.maximum_observation_dt_s)
    {
      std::array<double, 2> measured_velocity{
        (relative_position_ned_m[0] - previous_relative_position_ned_m_[0]) / dt,
        (relative_position_ned_m[1] - previous_relative_position_ned_m_[1]) / dt};
      measured_velocity = limited(measured_velocity, config_.maximum_relative_speed_mps);
      const double alpha = config_.relative_velocity_filter_time_constant_s <= 0.0 ?
        1.0 : dt / (config_.relative_velocity_filter_time_constant_s + dt);
      if (!relative_velocity_valid_) {
        relative_velocity_ned_mps_ = measured_velocity;
      } else {
        relative_velocity_ned_mps_[0] +=
          alpha * (measured_velocity[0] - relative_velocity_ned_mps_[0]);
        relative_velocity_ned_mps_[1] +=
          alpha * (measured_velocity[1] - relative_velocity_ned_mps_[1]);
      }
      relative_velocity_valid_ = true;
    } else {
      relative_velocity_valid_ = false;
      relative_velocity_ned_mps_ = {};
    }
  }

  relative_position_ned_m_ = relative_position_ned_m;
  previous_relative_position_ned_m_ = relative_position_ned_m;
  previous_acquisition_time_s_ = acquisition_time_s;
  observation_valid_ = true;
  previous_observation_valid_ = true;
  return true;
}

SimpleBelowTrackerResult SimpleBelowTracker::step(
  SimpleBelowTrackingMode mode, bool tracking_enabled, double control_dt_s,
  const std::array<double, 2> & observer_velocity_ned_mps)
{
  if (!std::isfinite(control_dt_s) || control_dt_s <= 0.0 ||
    !finite(observer_velocity_ned_mps))
  {
    throw std::invalid_argument("control_dt_s must be finite and positive");
  }

  SimpleBelowTrackerResult result;
  result.observation_valid = observation_valid_;
  result.relative_velocity_valid = relative_velocity_valid_;
  result.relative_position_ned_m = relative_position_ned_m_;
  result.relative_velocity_ned_mps = relative_velocity_ned_mps_;
  if (relative_velocity_valid_) {
    result.estimated_target_velocity_ned_mps = {
      relative_velocity_ned_mps_[0] + observer_velocity_ned_mps[0],
      relative_velocity_ned_mps_[1] + observer_velocity_ned_mps[1]};
  }
  result.predicted_relative_position_ned_m = relative_position_ned_m_;

  if (mode == SimpleBelowTrackingMode::PREDICTIVE && relative_velocity_valid_) {
    auto prediction = limited(
      {result.estimated_target_velocity_ned_mps[0] * config_.prediction_horizon_s,
        result.estimated_target_velocity_ned_mps[1] * config_.prediction_horizon_s},
      config_.maximum_prediction_distance_m);
    result.predicted_relative_position_ned_m[0] += prediction[0];
    result.predicted_relative_position_ned_m[1] += prediction[1];
  }

  std::array<double, 2> requested_velocity{};
  if (tracking_enabled && observation_valid_) {
    requested_velocity = limited(
      {config_.horizontal_position_gain * result.predicted_relative_position_ned_m[0],
        config_.horizontal_position_gain * result.predicted_relative_position_ned_m[1]},
      config_.maximum_horizontal_speed_mps);
  }

  const std::array<double, 2> delta{
    requested_velocity[0] - velocity_setpoint_ned_mps_[0],
    requested_velocity[1] - velocity_setpoint_ned_mps_[1]};
  const auto limited_delta = limited(
    delta, config_.maximum_horizontal_acceleration_mps2 * control_dt_s);
  velocity_setpoint_ned_mps_[0] += limited_delta[0];
  velocity_setpoint_ned_mps_[1] += limited_delta[1];
  result.velocity_setpoint_ned_mps = velocity_setpoint_ned_mps_;
  return result;
}

void SimpleBelowTracker::reset_observation()
{
  observation_valid_ = false;
  previous_observation_valid_ = false;
  relative_velocity_valid_ = false;
  relative_position_ned_m_ = {};
  previous_relative_position_ned_m_ = {};
  relative_velocity_ned_mps_ = {};
  previous_acquisition_time_s_ = 0.0;
}

}  // namespace px4_offboard_tracking
