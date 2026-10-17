#include "px4_offboard_tracking/maneuver_feedforward.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4_offboard_tracking
{
namespace
{

bool finite_vector(const std::array<double, 3> & value)
{
  return std::isfinite(value[0]) && std::isfinite(value[1]) &&
         std::isfinite(value[2]);
}

double smoothstep(double value)
{
  const double x = std::clamp(value, 0.0, 1.0);
  return x * x * (3.0 - 2.0 * x);
}

std::array<double, 3> limited(
  const std::array<double, 3> & value, double maximum_norm)
{
  const double norm = std::sqrt(
    value[0] * value[0] + value[1] * value[1] + value[2] * value[2]);
  if (norm <= maximum_norm || norm <= 1.0e-9) {
    return value;
  }
  const double scale = maximum_norm / norm;
  return {scale * value[0], scale * value[1], scale * value[2]};
}

}  // namespace

ManeuverFeedforward::ManeuverFeedforward(
  const ManeuverFeedforwardConfig & config)
: config_(config)
{
  if (config_.minimum_ca_probability < 0.0 ||
    config_.full_ca_probability <= config_.minimum_ca_probability ||
    config_.full_ca_probability > 1.0 ||
    config_.maximum_acceleration_stddev_mps2 <= 0.0 ||
    config_.filter_time_constant_s < 0.0 ||
    config_.maximum_target_acceleration_mps2 <= 0.0)
  {
    throw std::invalid_argument("invalid maneuver feed-forward configuration");
  }
}

ManeuverFeedforwardResult ManeuverFeedforward::update(
  const std::array<double, 3> & target_acceleration_ned,
  double ca_probability,
  double maximum_acceleration_stddev_mps2,
  bool estimate_fresh,
  double dt_s)
{
  if (!std::isfinite(dt_s) || dt_s <= 0.0) {
    throw std::invalid_argument("maneuver feed-forward dt must be positive");
  }

  const bool input_valid = estimate_fresh && finite_vector(target_acceleration_ned) &&
    std::isfinite(ca_probability) &&
    std::isfinite(maximum_acceleration_stddev_mps2) &&
    maximum_acceleration_stddev_mps2 >= 0.0;
  const auto target = input_valid ?
    limited(target_acceleration_ned, config_.maximum_target_acceleration_mps2) :
    std::array<double, 3>{};
  const double alpha = config_.filter_time_constant_s <= 0.0 ? 1.0 :
    dt_s / (config_.filter_time_constant_s + dt_s);
  for (int axis = 0; axis < 3; ++axis) {
    filtered_acceleration_[axis] +=
      alpha * (target[axis] - filtered_acceleration_[axis]);
  }

  ManeuverFeedforwardResult output;
  output.filtered_target_acceleration_ned = filtered_acceleration_;
  if (!input_valid) {
    return output;
  }

  const double probability_confidence = smoothstep(
    (ca_probability - config_.minimum_ca_probability) /
    (config_.full_ca_probability - config_.minimum_ca_probability));
  const double normalized_stddev =
    maximum_acceleration_stddev_mps2 /
    config_.maximum_acceleration_stddev_mps2;
  const double uncertainty_confidence = std::clamp(
    1.0 - normalized_stddev * normalized_stddev, 0.0, 1.0);
  output.confidence = probability_confidence * uncertainty_confidence;
  output.active = output.confidence > 1.0e-3;
  return output;
}

void ManeuverFeedforward::reset()
{
  filtered_acceleration_ = {};
}

}  // namespace px4_offboard_tracking
