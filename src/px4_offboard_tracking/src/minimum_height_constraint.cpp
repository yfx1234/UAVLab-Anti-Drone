#include "px4_offboard_tracking/minimum_height_constraint.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4_offboard_tracking
{

MinimumHeightConstraint::MinimumHeightConstraint(
  const MinimumHeightConstraintConfig & config)
: config_(config)
{
  if (!std::isfinite(config_.minimum_height_m) ||
    config_.minimum_height_m <= 0.0 ||
    !std::isfinite(config_.prediction_horizon_s) ||
    config_.prediction_horizon_s <= 0.0 ||
    !std::isfinite(config_.maximum_upward_acceleration_mps2) ||
    config_.maximum_upward_acceleration_mps2 <= 0.0 ||
    !std::isfinite(config_.recovery_height_m) ||
    config_.recovery_height_m <= config_.minimum_height_m ||
    !std::isfinite(config_.recovery_gain_s2) ||
    config_.recovery_gain_s2 <= 0.0 ||
    !std::isfinite(config_.recovery_damping_s_inv) ||
    config_.recovery_damping_s_inv <= 0.0)
  {
    throw std::invalid_argument("invalid minimum-height constraint configuration");
  }
}

MinimumHeightConstraintResult MinimumHeightConstraint::apply(
  const MinimumHeightConstraintInput & input) const
{
  if (!std::isfinite(input.height_m) ||
    !std::isfinite(input.vertical_velocity_down_mps) ||
    !std::isfinite(input.requested_acceleration_down_mps2))
  {
    return {};
  }

  const double horizon = config_.prediction_horizon_s;
  const double predicted_height_m =
    input.height_m - input.vertical_velocity_down_mps * horizon -
    0.5 * input.requested_acceleration_down_mps2 * horizon * horizon;

  if (input.height_m >= config_.minimum_height_m &&
    predicted_height_m >= config_.minimum_height_m)
  {
    return {
      input.requested_acceleration_down_mps2,
      input.requested_acceleration_down_mps2,
      false};
  }

  double bounded_limit{};
  if (input.height_m < config_.minimum_height_m) {
    bounded_limit =
      -config_.recovery_gain_s2 *
      (config_.recovery_height_m - input.height_m) -
      config_.recovery_damping_s_inv *
      std::max(0.0, input.vertical_velocity_down_mps);
  } else {
    // h(t)=h0-v_D*t-0.5*a_D*t^2 >= h_min
    bounded_limit = 2.0 *
      (input.height_m - config_.minimum_height_m -
      input.vertical_velocity_down_mps * horizon) /
      (horizon * horizon);
  }
  bounded_limit = std::max(
    -config_.maximum_upward_acceleration_mps2, bounded_limit);
  const double constrained = std::min(
    input.requested_acceleration_down_mps2, bounded_limit);
  return {
    constrained,
    bounded_limit,
    constrained < input.requested_acceleration_down_mps2 - 1e-9};
}

const MinimumHeightConstraintConfig & MinimumHeightConstraint::config() const
{
  return config_;
}

}  // namespace px4_offboard_tracking
